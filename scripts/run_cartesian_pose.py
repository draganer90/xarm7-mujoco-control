"""Independent 6D Cartesian XYZ + orientation planning with numerical IK.

Examples (from project root):
  python scripts/run_cartesian_pose.py --delta-xyz 0.02,0,0 --delta-rotvec-deg 0,0,0
  python scripts/run_cartesian_pose.py --delta-xyz 0.02,0,0 --delta-rotvec-deg 0,0,5 --view

World-frame displacement and world-frame rotation vector, about the link7
BODY ORIGIN (not a calibrated tool-tip). Planned joint path is tracked by PD+G
or joint-space CTC; this is not a direct Cartesian closed-loop controller.
Simulation only. Collision checking and real robot safety are not provided.
"""
import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import sys
import time
from uuid import uuid4

import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from xarm_control.robot import (DEFAULT_GOAL, load_model, get_body_id,
                                body_pose_and_jacobian, orientation_error_world,
                                gravity, mass_and_bias)
from xarm_control.controllers import (GOOD_KP, GOOD_KD, CTC_OMEGA_TUNED_FAST,
                                      CTC_ZETA, clip_torque)
from xarm_control.cartesian_planning import JointSpline, quintic_scale
from xarm_control.cartesian_pose_planning import (plan_cartesian_pose,
                                                    target_pose, validate_pose_spline)


def triple(s):
    try:
        values = np.asarray([float(part.strip()) for part in s.split(',')], dtype=float)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('Expected three comma-separated numbers') from exc
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise argparse.ArgumentTypeError('Expected exactly three finite numbers')
    return values


def joints(s):
    try:
        values = np.asarray([float(part.strip()) for part in s.split(',')], dtype=float)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('Expected seven comma-separated numbers') from exc
    if values.shape != (7,) or not np.all(np.isfinite(values)):
        raise argparse.ArgumentTypeError('Expected exactly seven finite angles [rad]')
    return values


def joint_limits(model, q):
    lo, hi = np.full(model.nv, -np.inf), np.full(model.nv, np.inf)
    for i in range(model.njnt):
        if model.jnt_limited[i]:
            dof = int(model.jnt_dofadr[i])
            lo[dof], hi[dof] = model.jnt_range[i]
    if np.any(q < lo) or np.any(q > hi):
        raise ValueError('Initial configuration violates joint limits')
    return lo, hi


def rmse(values):
    values = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(values**2)))


def simulate(args):
    if not (np.isfinite(args.duration) and np.isfinite(args.move_duration)
            and args.duration > args.move_duration > 0):
        raise ValueError('Simulation duration must exceed positive move duration')
    if not (.005 <= args.planner_dt <= .10):
        raise ValueError('Planner dt must lie in [0.005, 0.10] s')
    model = load_model(ROOT)
    bid = get_body_id(model, args.ee_body)
    q0 = DEFAULT_GOAL.copy() if args.start_q is None else args.start_q.copy()
    lo, hi = joint_limits(model, q0)
    kin = mujoco.MjData(model)

    def fk_jac(q):
        kin.qpos[:] = q
        kin.qvel[:] = 0
        mujoco.mj_forward(model, kin)
        return body_pose_and_jacobian(model, kin, bid, full_pose=True)

    p0, R0, _ = fk_jac(q0)
    p1 = p0 + args.delta_xyz
    delta_rotvec = np.deg2rad(args.delta_rotvec_deg)
    R1 = Rotation.from_rotvec(delta_rotvec).as_matrix() @ R0
    if np.linalg.norm(args.delta_xyz) > .1:
        raise ValueError('Demonstration allows <=100 mm Cartesian displacement')
    if np.linalg.norm(delta_rotvec) > np.deg2rad(15):
        raise ValueError('Demonstration allows <=15 deg total orientation change')

    print('Independent Cartesian 6D pose / xArm7 (WORLD coordinates)')
    print('Body-frame origin used:', args.ee_body)
    print('Initial q [rad]:', q0)
    print('Start XYZ [m]:', p0)
    print('Target XYZ [m]:', p1)
    print('World rotation vector [deg]:', args.delta_rotvec_deg)
    print('Orientation IS explicitly constrained in 6D IK.')
    plan = plan_cartesian_pose(fk_jac, q0, p1, R1, lo, hi,
                                duration=args.move_duration, sample_dt=args.planner_dt)
    reference = JointSpline(plan)
    dense_p, dense_r = validate_pose_spline(reference, fk_jac, p0, R0, p1, R1, lo, hi)
    print(f'6D IK preflight PASS: {len(plan.t)} waypoints, '
          f'worst waypoint xyz={plan.position_residual_max_m*1000:.4f} mm, '
          f'orientation={plan.orientation_residual_max_rad:.6f} rad')
    print(f'Dense reference xyz={dense_p*1000:.4f} mm, '
          f'orientation={dense_r:.6f} rad')

    output = ROOT/'results'/'cartesian_pose'/(
        datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_'+uuid4().hex[:6])
    output.mkdir(parents=True, exist_ok=False)
    with (output/'planned_waypoints.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['t_s', 'px_m', 'py_m', 'pz_m', 'q_w', 'q_x', 'q_y', 'q_z'] +
                        [f'joint_{j+1}_rad' for j in range(7)])
        for ti, xyz, Ri, qi in zip(plan.t, plan.xyz, plan.rotations, plan.q):
            xyzw = Rotation.from_matrix(Ri).as_quat()
            writer.writerow([float(ti), *xyz.tolist(), float(xyzw[3]),
                             *xyzw[:3].tolist(), *qi.tolist()])

    data, grav_data = mujoco.MjData(model), mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[:] = q0
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    records = []
    nsat = 0
    viewer = None
    try:
        if args.view:
            viewer = mujoco.viewer.launch_passive(model, data)
        while data.time < args.duration and (viewer is None or viewer.is_running()):
            mujoco.mj_forward(model, data)
            q_des, v_des, a_des = reference.evaluate(data.time)
            q, v = data.qpos.copy(), data.qvel.copy()
            if args.controller == 'pd_g':
                tau_raw = GOOD_KP*(q_des-q) + GOOD_KD*(v_des-v) + gravity(model, grav_data, q)
            else:
                omega = CTC_OMEGA_TUNED_FAST
                a_cmd = a_des + 2*CTC_ZETA*omega*(v_des-v) + omega**2*(q_des-q)
                M, bias = mass_and_bias(model, data)
                tau_raw = M @ a_cmd + bias - data.qfrc_passive.copy()
            if not np.all(np.isfinite(tau_raw)):
                raise RuntimeError(f'Nonfinite control at t={data.time:.3f}s')
            tau, clipped = clip_torque(model, tau_raw)
            nsat += int(clipped)
            data.ctrl[:] = tau
            mujoco.mj_step(model, data)
            if not (np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))):
                raise RuntimeError(f'Nonfinite robot state at t={data.time:.3f}s')
            if np.max(np.abs(data.qvel)) > 30:
                raise RuntimeError(f'Excessive joint speed at t={data.time:.3f}s')
            mujoco.mj_forward(model, data)
            p_actual, R_actual, _ = body_pose_and_jacobian(model, data, bid, True)
            qr, _, _ = reference.evaluate(data.time)
            pr, Rr = target_pose(p0, R0, p1, R1,
                                 float(quintic_scale(data.time, args.move_duration)))
            epos = float(np.linalg.norm(pr-p_actual))
            erot = float(np.linalg.norm(orientation_error_world(Rr, R_actual)))
            row = {
                't_s':float(data.time), 'tcp_err_m':epos,
                'orientation_err_rad':erot,
                'joint_tracking_err_rad':float(np.linalg.norm(qr-data.qpos)),
                'peak_tau_Nm':float(np.max(np.abs(tau))), 'saturated':int(clipped),
            }
            for i, axis in enumerate(('x','y','z')):
                row[f'{axis}_target_m'] = float(pr[i])
                row[f'{axis}_actual_m'] = float(p_actual[i])
            row['orientation_target_world_rotvec_norm_rad'] = float(
                np.linalg.norm(Rotation.from_matrix(Rr @ R0.T).as_rotvec()))
            row['orientation_actual_world_rotvec_norm_rad'] = float(
                np.linalg.norm(Rotation.from_matrix(R_actual @ R0.T).as_rotvec()))
            for i in range(7):
                row[f'joint{i+1}_ref_rad'] = float(qr[i])
                row[f'joint{i+1}_actual_rad'] = float(data.qpos[i])
                row[f'tau{i+1}_Nm'] = float(tau[i])
            records.append(row)
            if viewer is not None:
                viewer.sync()
                time.sleep(model.opt.timestep)
    finally:
        if viewer is not None:
            viewer.close()
    if not records:
        raise RuntimeError('No simulation samples')

    move = [r for r in records if r['t_s'] <= args.move_duration]
    hold = [r for r in records if r['t_s'] > args.move_duration]
    summary = {
        'controller':args.controller,
        'samples':len(records), 'sim_time_s':records[-1]['t_s'],
        'move_duration_s':args.move_duration,
        'IK_waypoint_position_max_mm':1000*plan.position_residual_max_m,
        'IK_waypoint_orientation_max_rad':plan.orientation_residual_max_rad,
        'IK_dense_position_max_mm':1000*dense_p,
        'IK_dense_orientation_max_rad':dense_r,
        'TCP_rmse_full_mm':1000*rmse([r['tcp_err_m'] for r in records]),
        'TCP_rmse_move_mm':1000*rmse([r['tcp_err_m'] for r in move]),
        'TCP_rmse_hold_mm':1000*rmse([r['tcp_err_m'] for r in hold]) if hold else None,
        'TCP_final_error_mm':1000*records[-1]['tcp_err_m'],
        'orientation_rmse_full_rad':rmse([r['orientation_err_rad'] for r in records]),
        'orientation_rmse_move_rad':rmse([r['orientation_err_rad'] for r in move]),
        'orientation_rmse_hold_rad':rmse([r['orientation_err_rad'] for r in hold]) if hold else None,
        'orientation_final_error_rad':records[-1]['orientation_err_rad'],
        'joint_tracking_rmse_rad':rmse([r['joint_tracking_err_rad'] for r in records]),
        'peak_tau_Nm':max(r['peak_tau_Nm'] for r in records),
        'saturated_fraction':nsat/len(records),
        'output_dir':str(output),
    }
    with (output/'time_series.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)
    with (output/'config_and_summary.json').open('w', encoding='utf-8') as f:
        json.dump({
            'summary':summary,
            'start_q_rad':q0.tolist(), 'target_xyz_m':p1.tolist(),
            'delta_rotvec_world_deg':args.delta_rotvec_deg.tolist(),
            'body_frame':args.ee_body,
            'reference':'straight XYZ + shortest world-frame rotation with quintic time law',
            'tracking':'joint-space reference from 6D IK tracked via joint PD+G/CTC',
            'limitation':'No collision checking or real robot safety guarantee',
            'model':'models/xarm7_actuated.xml',
            'model_dt_s':float(model.opt.timestep),
            'mujoco_version':mujoco.__version__,
        }, f, indent=2, ensure_ascii=False)
    try:
        from matplotlib import pyplot as plt
        ts = [r['t_s'] for r in records]
        for col, label, file in [
                ('tcp_err_m','TCP position error [mm]','tcp_error.png'),
                ('orientation_err_rad','Orientation error [rad]','orientation_error.png')]:
            ys = [r[col]*(1000 if col=='tcp_err_m' else 1) for r in records]
            fig, ax = plt.subplots(figsize=(9,4.5))
            ax.plot(ts, ys)
            ax.axvline(args.move_duration, color='gray', linestyle='--', label='Move ends')
            ax.grid(True); ax.legend()
            ax.set(xlabel='Time [s]', ylabel=label, title=f'6D planned pose: {label}')
            fig.tight_layout(); fig.savefig(output/file, dpi=160); plt.close(fig)
        fig, ax = plt.subplots(figsize=(9,5))
        for name in ('x','y','z'):
            ax.plot(ts, [r[f'{name}_target_m'] for r in records], '--', label=f'{name} target')
            ax.plot(ts, [r[f'{name}_actual_m'] for r in records], label=f'{name} actual')
        ax.set(xlabel='Time [s]', ylabel='World position [m]', title='6D reference XYZ vs actual')
        ax.grid(True); ax.legend(ncol=3); fig.tight_layout()
        fig.savefig(output/'xyz_reference_actual.png', dpi=160); plt.close(fig)
    except ImportError:
        print('Matplotlib unavailable; CSV and JSON were saved')
    for key, value in summary.items():
        print(f'{key}: {value}')
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--delta-xyz', type=triple, default=np.array([.02, 0., 0.]),
                   help='World XYZ displacement [m], e.g. 0.02,0,0')
    p.add_argument('--delta-rotvec-deg', type=triple, default=np.zeros(3),
                   help='World rotation vector [degrees], e.g. 0,0,5 (NOT Euler angles)')
    p.add_argument('--start-q', type=joints, default=None,
                   help='Initial seven joint angles [rad], default DEFAULT_GOAL')
    p.add_argument('--ee-body', default='link7')
    p.add_argument('--controller', choices=('pd_g','ctc_tuned_fast'), default='pd_g')
    p.add_argument('--move-duration', type=float, default=3.0)
    p.add_argument('--duration', type=float, default=8.0)
    p.add_argument('--planner-dt', type=float, default=.02)
    p.add_argument('--view', action='store_true')
    simulate(p.parse_args())


if __name__ == '__main__':
    main()
