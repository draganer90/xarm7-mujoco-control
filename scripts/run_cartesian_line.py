"""Simulated xArm7: independent 3D Cartesian straight line + numerical IK.

Run from project root, e.g.:
  python scripts/run_cartesian_line.py --delta-xyz 0.02,0,0 --controller pd_g

The position reference is generated in WORLD coordinates, independently of any
predefined joint-space trajectory. 'TCP' here means link7 BODY ORIGIN, not a
calibrated tool tip. Orientation is monitored, not controlled. Simulation only.
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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from xarm_control.robot import (DEFAULT_GOAL, load_model, get_body_id,
                                body_pose_and_jacobian, orientation_error_world,
                                gravity, mass_and_bias)
from xarm_control.controllers import (GOOD_KP, GOOD_KD, CTC_OMEGA_TUNED_FAST,
                                      CTC_ZETA, clip_torque)
from xarm_control.cartesian_planning import (plan_cartesian_line, JointSpline,
                                            validate_spline, quintic_scale)


def xyz_arg(s):
    try:
        v = np.array([float(x.strip()) for x in s.split(',')], dtype=float)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('Expected three comma-separated numbers') from exc
    if v.shape != (3,) or not np.all(np.isfinite(v)):
        raise argparse.ArgumentTypeError('Expected three finite numbers [m]')
    return v


def joints_arg(s):
    try:
        v = np.array([float(x.strip()) for x in s.split(',')], dtype=float)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('Expected seven comma-separated joint angles [rad]') from exc
    if v.shape != (7,) or not np.all(np.isfinite(v)):
        raise argparse.ArgumentTypeError('Expected seven finite joint angles [rad]')
    return v


def check_initial_limits(model, q):
    lo, hi = np.full(model.nv, -np.inf), np.full(model.nv, np.inf)
    for i in range(model.njnt):
        if model.jnt_limited[i]:
            dof = int(model.jnt_dofadr[i])
            lo[dof], hi[dof] = model.jnt_range[i]
    if np.any(q < lo) or np.any(q > hi):
        raise ValueError('Initial q is outside model joint limits')
    return lo, hi


def rmse(values):
    vals = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(vals*vals)))


def simulate(args):
    if not (args.duration > args.move_duration > 0.0):
        raise ValueError('duration must exceed move-duration, both positive')
    if not (0.005 <= args.planner_dt <= 0.10):
        raise ValueError('planner-dt must be between 0.005 and 0.10 seconds')
    model = load_model(ROOT)
    body_id = get_body_id(model, args.ee_body)
    q0 = DEFAULT_GOAL.copy() if args.start_q is None else args.start_q.copy()
    lower, upper = check_initial_limits(model, q0)

    # One FK callback used ONLY for offline planning and validation.
    kin = mujoco.MjData(model)
    def fk_jac(q):
        kin.qpos[:] = q
        kin.qvel[:] = 0.0
        mujoco.mj_forward(model, kin)
        p, _, Jp = body_pose_and_jacobian(model, kin, body_id, full_pose=False)
        return p, Jp

    start_xyz, _ = fk_jac(q0)
    target_xyz = (start_xyz + args.delta_xyz if args.target_xyz is None
                  else args.target_xyz.copy())
    if np.linalg.norm(target_xyz-start_xyz) > 0.10:
        raise ValueError('For this first demonstration, Cartesian displacement must be <= 0.10 m')

    print('Independent Cartesian 3D line / xArm7 (world frame)')
    print('Initial joint angles [rad]:', q0)
    print('Start link7 origin [m]:', start_xyz)
    print('Target link7 origin [m]:', target_xyz)
    print('Displacement [mm]:', 1000*(target_xyz-start_xyz))
    print('Planning position-only IK; orientation is NOT constrained.')
    plan = plan_cartesian_line(fk_jac, q0, target_xyz, lower, upper,
                               duration=args.move_duration,
                               sample_dt=args.planner_dt)
    reference = JointSpline(plan)
    preflight_err_m = validate_spline(reference, fk_jac, start_xyz, target_xyz,
                                      lower, upper, max_path_error_m=0.001)
    print(f'IK preflight PASS: {len(plan.t)} waypoints; max waypoint residual='
          f'{1000*plan.ik_max_residual_m:.4f} mm; '
          f'max dense-path residual={1000*preflight_err_m:.4f} mm')

    # Generate unique output only after offline planning has passed.
    output = ROOT/'results'/'cartesian_line'/(
        datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_'+uuid4().hex[:6])
    output.mkdir(parents=True, exist_ok=False)
    with (output/'planned_waypoints.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['t_s','x_target_m','y_target_m','z_target_m'] +
                        [f'q_ref_{j+1}_rad' for j in range(7)])
        for t, p, q in zip(plan.t, plan.xyz, plan.q):
            writer.writerow([float(t), *p.tolist(), *q.tolist()])

    data = mujoco.MjData(model)
    grav_data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[:] = q0
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    _, initial_R, _ = body_pose_and_jacobian(model, data, body_id, True)
    records = []
    n_sat = 0
    viewer_obj = None
    try:
        if args.view:
            viewer_obj = mujoco.viewer.launch_passive(model, data)
        while data.time < args.duration and (viewer_obj is None or viewer_obj.is_running()):
            mujoco.mj_forward(model, data)
            q_des, v_des, a_des = reference.evaluate(data.time)
            q, v = data.qpos.copy(), data.qvel.copy()
            if args.controller == 'pd_g':
                raw = (GOOD_KP*(q_des-q) + GOOD_KD*(v_des-v) +
                       gravity(model, grav_data, q))
            else:
                omega = CTC_OMEGA_TUNED_FAST
                a_cmd = a_des + 2*CTC_ZETA*omega*(v_des-v) + omega**2*(q_des-q)
                M, bias = mass_and_bias(model, data)
                raw = M @ a_cmd + bias - data.qfrc_passive.copy()
            if not np.all(np.isfinite(raw)):
                raise RuntimeError(f'Nonfinite command at t={data.time:.3f}s')
            tau, clipped = clip_torque(model, raw)
            n_sat += int(clipped)
            data.ctrl[:] = tau
            mujoco.mj_step(model, data)
            if not (np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))):
                raise RuntimeError(f'Nonfinite robot state at t={data.time:.3f}s')
            if np.max(np.abs(data.qvel)) > 30:
                raise RuntimeError(f'Excessive joint speed at t={data.time:.3f}s')
            mujoco.mj_forward(model, data)
            actual_xyz, actual_R, _ = body_pose_and_jacobian(model, data, body_id, True)
            q_des_after, _, _ = reference.evaluate(data.time)
            desired_xyz = start_xyz + quintic_scale(data.time, args.move_duration)*(target_xyz-start_xyz)
            e_tcp = float(np.linalg.norm(desired_xyz-actual_xyz))
            e_ori = float(np.linalg.norm(orientation_error_world(initial_R, actual_R)))
            row = {'t_s':float(data.time), 'tcp_err_m':e_tcp,
                   'orientation_drift_rad':e_ori,
                   'joint_tracking_err_rad':float(np.linalg.norm(q_des_after-data.qpos)),
                   'peak_tau_Nm':float(np.max(np.abs(tau))),
                   'saturated':int(clipped)}
            for j, axis in enumerate(('x','y','z')):
                row[f'{axis}_ref_m'] = float(desired_xyz[j])
                row[f'{axis}_actual_m'] = float(actual_xyz[j])
            for j in range(7):
                row[f'q{j+1}_actual_rad'] = float(data.qpos[j])
                row[f'q{j+1}_ref_rad'] = float(q_des_after[j])
                row[f'tau{j+1}_Nm'] = float(tau[j])
            records.append(row)
            if viewer_obj is not None:
                viewer_obj.sync()
                time.sleep(model.opt.timestep)
    finally:
        if viewer_obj is not None:
            viewer_obj.close()
    if not records:
        raise RuntimeError('Simulation viewer closed before first sample')

    moving = [r for r in records if r['t_s'] <= args.move_duration]
    holding = [r for r in records if r['t_s'] > args.move_duration]
    summary = {
        'controller':args.controller,
        'samples':len(records),
        'completed_duration_s':records[-1]['t_s'],
        'move_duration_s':args.move_duration,
        'ik_max_waypoint_error_mm':1000*plan.ik_max_residual_m,
        'planned_line_max_dense_error_mm':1000*preflight_err_m,
        'tcp_rmse_full_mm':1000*rmse([r['tcp_err_m'] for r in records]),
        'tcp_rmse_move_mm':1000*rmse([r['tcp_err_m'] for r in moving]),
        'tcp_rmse_hold_mm':1000*rmse([r['tcp_err_m'] for r in holding]) if holding else None,
        'tcp_final_error_mm':1000*records[-1]['tcp_err_m'],
        'orientation_drift_rmse_rad':rmse([r['orientation_drift_rad'] for r in records]),
        'joint_ref_rmse_rad':rmse([r['joint_tracking_err_rad'] for r in records]),
        'peak_tau_Nm':max(r['peak_tau_Nm'] for r in records),
        'saturated_fraction':n_sat/len(records),
        'output_dir':str(output),
    }
    with (output/'time_series.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(records[0].keys()))
        writer.writeheader()
        writer.writerows(records)
    provenance = {
        'summary':summary,
        'start_q_rad':q0.tolist(),
        'start_body_origin_m':start_xyz.tolist(),
        'target_body_origin_m':target_xyz.tolist(),
        'body_frame':args.ee_body,
        'world_frame':True,
        'orientation_controlled':False,
        'spline_type':'quintic interpolating joint reference; clamped endpoints',
        'model':'models/xarm7_actuated.xml',
        'model_timestep_s':float(model.opt.timestep),
        'mujo_version':mujoco.__version__,
        'gain_note':'Reuses current model-specific controller gains; simulation only.',
    }
    with (output/'config_and_summary.json').open('w', encoding='utf-8') as f:
        json.dump(provenance, f, indent=2, ensure_ascii=False)
    try:
        from matplotlib import pyplot as plt
        ts = [r['t_s'] for r in records]
        fig, ax = plt.subplots(figsize=(9, 4.5))
        ax.plot(ts, [1000*r['tcp_err_m'] for r in records], label='TCP position error')
        ax.axvline(args.move_duration, ls='--', color='gray', label='Reference finished')
        ax.set(xlabel='Time [s]', ylabel='Error [mm]', title='Cartesian line: position tracking')
        ax.grid(True); ax.legend(); fig.tight_layout()
        fig.savefig(output/'tcp_position_error.png', dpi=160); plt.close(fig)
        fig, ax = plt.subplots(figsize=(9, 5))
        for axis in ('x', 'y', 'z'):
            ax.plot(ts, [r[f'{axis}_ref_m'] for r in records], '--', label=f'{axis} target')
            ax.plot(ts, [r[f'{axis}_actual_m'] for r in records], label=f'{axis} actual')
        ax.set(xlabel='Time [s]', ylabel='World position [m]', title='Cartesian XYZ reference vs actual')
        ax.grid(True); ax.legend(ncol=3); fig.tight_layout()
        fig.savefig(output/'xyz_reference_actual.png', dpi=160); plt.close(fig)
        fig, ax = plt.subplots(figsize=(9, 4.5))
        ax.plot(ts, [r['orientation_drift_rad'] for r in records])
        ax.set(xlabel='Time [s]', ylabel='Orientation drift [rad]',
               title='Orientation NOT controlled (diagnostic only)')
        ax.grid(True); fig.tight_layout()
        fig.savefig(output/'orientation_drift.png', dpi=160); plt.close(fig)
    except ImportError:
        print('Matplotlib not available, CSV and JSON were saved.')
    for key, value in summary.items():
        print(f'{key}: {value}')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--delta-xyz', type=xyz_arg, default=np.array([.02, 0., 0.]),
                       help='Relative XYZ displacement [m], e.g. 0.02,0,0 (default)')
    group.add_argument('--target-xyz', type=xyz_arg, default=None,
                       help='Absolute XYZ target [m] in world frame')
    parser.add_argument('--start-q', type=joints_arg, default=None,
                        help='Seven joint angles [rad]; default: DEFAULT_GOAL')
    parser.add_argument('--ee-body', default='link7')
    parser.add_argument('--controller', choices=('pd_g','ctc_tuned_fast'), default='pd_g')
    parser.add_argument('--move-duration', type=float, default=3.0)
    parser.add_argument('--duration', type=float, default=8.0)
    parser.add_argument('--planner-dt', type=float, default=0.02)
    parser.add_argument('--view', action='store_true')
    args = parser.parse_args()
    simulate(args)


if __name__ == '__main__':
    main()
