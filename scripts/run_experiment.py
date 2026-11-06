"""Run one xArm7 controller with repeatable references and separate result folders.

Run from the project root, for example:
  python scripts/run_experiment.py --controller ctc_tuned_fast --trajectory quintic --traj-duration 3 --view
"""
import argparse
import csv
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from uuid import uuid4

import mujoco
import mujoco.viewer
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from xarm_control.robot import (load_model, DEFAULT_GOAL, get_body_id,
                                body_pose_and_jacobian, orientation_error_world)
from xarm_control.controllers import (Controller, METHODS, clip_torque,
                                      controller_settings)


def _unique_output_dir(method):
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    return ROOT / 'results' / 'experiments' / (f'{stamp}_{method}_{uuid4().hex[:6]}')


def _validate_goal(model, goal):
    goal = np.asarray(DEFAULT_GOAL if goal is None else goal, dtype=float)
    if goal.shape != (7,) or not np.all(np.isfinite(goal)):
        raise ValueError('Goal must contain exactly 7 finite joint angles in radians')
    for j in range(7):
        if model.jnt_limited[j]:
            lo, hi = model.jnt_range[j]
            if not lo <= goal[j] <= hi:
                raise ValueError(f'Goal joint{j+1}={goal[j]} outside [{lo}, {hi}] rad')
    return goal


def experiment(method, duration=8.0, viewer=False, write=True, ee_body='link7',
               trajectory='quintic', traj_duration=None, goal=None, output_dir=None):
    """Returns metrics; backward-compatible with the old benchmark.py positional calls."""
    model = load_model(ROOT)
    goal = _validate_goal(model, goal)
    data = mujoco.MjData(model)
    ref_data = mujoco.MjData(model)
    model_body_id = get_body_id(model, ee_body)
    ctrl = Controller(model, method, ROOT, goal, duration, ee_body,
                      trajectory=trajectory, traj_duration=traj_duration)
    mujoco.mj_resetData(model, data)
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    records = []
    saturated_steps = 0
    viewer_obj = None
    try:
        if viewer:
            viewer_obj = mujoco.viewer.launch_passive(model, data)
        while data.time < duration and (viewer_obj is None or viewer_obj.is_running()):
            mujoco.mj_forward(model, data)
            raw_tau = ctrl.command(data)
            if not np.all(np.isfinite(raw_tau)):
                raise RuntimeError(f'Nonfinite commanded torque at t={data.time:.3f}')
            tau, clipped = clip_torque(model, raw_tau)
            saturated_steps += int(clipped)
            data.ctrl[:] = tau
            mujoco.mj_step(model, data)
            if not (np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))):
                raise RuntimeError(f'Nonfinite state at t={data.time:.3f}')
            if np.max(np.abs(data.qvel)) > 30.0:
                raise RuntimeError(f'Excessive joint speed at t={data.time:.3f}')
            mujoco.mj_forward(model, data)
            qd, vd, ad = ctrl.ref(data.time)
            ref_data.qpos[:], ref_data.qvel[:] = qd, vd
            mujoco.mj_forward(model, ref_data)
            p, R, _ = body_pose_and_jacobian(model, data, model_body_id, True)
            p_ref, R_ref, _ = body_pose_and_jacobian(model, ref_data, model_body_id, True)
            rec = dict(t=float(data.time),
                       joint_err_rad=float(np.linalg.norm(qd-data.qpos)),
                       tcp_err_m=float(np.linalg.norm(p_ref-p)),
                       orient_err_rad=float(np.linalg.norm(orientation_error_world(R_ref, R))),
                       tau_peak_Nm=float(np.max(np.abs(tau))),
                       saturation=int(clipped))
            for i in range(7):
                rec[f'q{i+1}'] = float(data.qpos[i])
                rec[f'qd{i+1}'] = float(qd[i])
                rec[f'v{i+1}'] = float(data.qvel[i])
                rec[f'vd{i+1}'] = float(vd[i])
                rec[f'a_des{i+1}'] = float(ad[i])
                rec[f'e{i+1}'] = float(qd[i]-data.qpos[i])
                rec[f'tau{i+1}'] = float(tau[i])
            records.append(rec)
            if viewer_obj is not None:
                viewer_obj.sync()
                time.sleep(model.opt.timestep)
    finally:
        if viewer_obj is not None:
            viewer_obj.close()
    if not records:
        raise RuntimeError('No samples. Was the viewer closed immediately?')

    def rmse(key):
        x = np.array([r[key] for r in records])
        return float(np.sqrt(np.mean(x*x)))

    summary = dict(method=method, steps=len(records),
                   duration_s=float(duration), trajectory=trajectory,
                   traj_duration_s=float(ctrl.traj_duration),
                   rmse_joint_rad=rmse('joint_err_rad'),
                   rmse_tcp_m=rmse('tcp_err_m'),
                   rmse_orient_rad=rmse('orient_err_rad'),
                   final_joint_err_rad=records[-1]['joint_err_rad'],
                   peak_torque_Nm=float(max(r['tau_peak_Nm'] for r in records)),
                   saturated_fraction=float(saturated_steps/len(records)))
    for i in range(7):
        summary[f'rmse_J{i+1}_rad'] = rmse(f'e{i+1}')
        summary[f'final_J{i+1}_error_rad'] = float(records[-1][f'e{i+1}'])
    if write:
        out = Path(output_dir) if output_dir is not None else _unique_output_dir(method)
        # Never reuse an experiment folder: prevents loss of previous data.
        out.mkdir(parents=True, exist_ok=False)
        with (out/'time_series.csv').open('w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=records[0].keys())
            writer.writeheader()
            writer.writerows(records)
        provenance = dict(summary=summary,
                          goal_rad=goal.tolist(),
                          initial_q_rad=[0.0]*7,
                          controller=controller_settings(method),
                          mujoco_version=mujoco.__version__,
                          model='models/xarm7_actuated.xml',
                          model_timestep_s=float(model.opt.timestep),
                          joint_damping=model.dof_damping.tolist(),
                          joint_frictionloss=model.dof_frictionloss.tolist(),
                          actuator_ctrlrange=model.actuator_ctrlrange.tolist())
        with (out/'config_and_summary.json').open('w', encoding='utf-8') as f:
            json.dump(provenance, f, indent=2, ensure_ascii=False)
        print('CSV:', out/'time_series.csv')
        try:
            from matplotlib import pyplot as plt
            for metric, ylabel, name in (
                    ('joint_err_rad', 'Joint error norm [rad]', 'joint_error'),
                    ('tcp_err_m', 'TCP position error [m]', 'tcp_error'),
                    ('orient_err_rad', 'TCP orientation error [rad]', 'orientation_error')):
                fig, ax = plt.subplots(figsize=(9, 4.5))
                ax.plot([r['t'] for r in records], [r[metric] for r in records])
                ax.set(xlabel='Time [s]', ylabel=ylabel,
                       title=f'{method} — {trajectory} T={ctrl.traj_duration:g}s')
                ax.grid(True)
                fig.tight_layout()
                fig.savefig(out/f'{name}.png', dpi=170)
                plt.close(fig)
            fig, axs = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
            for i in range(7):
                axs[0].plot([r['t'] for r in records], [r[f'q{i+1}'] for r in records],
                            label=f'J{i+1}')
                axs[1].plot([r['t'] for r in records], [r[f'e{i+1}'] for r in records],
                            label=f'J{i+1}')
            axs[0].set_ylabel('Actual q [rad]')
            axs[1].set(xlabel='Time [s]', ylabel='q_des - q [rad]')
            for ax in axs:
                ax.grid(True)
                ax.legend(ncol=4, fontsize=8)
            fig.tight_layout()
            fig.savefig(out/'joint_positions_errors.png', dpi=170)
            plt.close(fig)
        except ImportError:
            print('Matplotlib missing: CSV and metadata saved, plots skipped.')
        summary['output_dir'] = str(out)
    return summary


def _parse_goal(s):
    try:
        vals = [float(x.strip()) for x in s.split(',')]
    except ValueError as exc:
        raise argparse.ArgumentTypeError('Goal must be 7 comma-separated numbers') from exc
    if len(vals) != 7:
        raise argparse.ArgumentTypeError('Goal must contain exactly 7 comma-separated joint angles')
    return vals


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--controller', choices=METHODS, default='pd_g')
    p.add_argument('--duration', type=float, default=8.0)
    p.add_argument('--traj-duration', type=float, default=None)
    p.add_argument('--trajectory', choices=('quintic', 'cubic'), default='quintic')
    p.add_argument('--goal', type=_parse_goal, default=None,
                   help='Seven comma-separated desired joint angles [rad]')
    p.add_argument('--view', action='store_true', help='Show MuJoCo viewer')
    p.add_argument('--ee-body', default='link7')
    args = p.parse_args()
    summary = experiment(args.controller, args.duration, args.view,
                         ee_body=args.ee_body, trajectory=args.trajectory,
                         traj_duration=args.traj_duration, goal=args.goal)
    for k,v in summary.items():
        print(f'{k}: {v}')


if __name__ == '__main__':
    main()
