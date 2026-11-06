"""Fair, repeatable simulation comparison of joint controllers on same xArm7 model.

From project root:
    python scripts/compare_controllers.py
    python scripts/compare_controllers.py --profiles baseline,fast,slow,alternate,cubic
    python scripts/compare_controllers.py --methods pd_g,ctc_tuned,ctc_tuned_fast

Each run gets its own CSV, plots and config in a unique benchmark directory.
"""
import argparse
import csv
import json
import sys
from datetime import datetime
from pathlib import Path
from uuid import uuid4

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.run_experiment import experiment
from xarm_control.controllers import METHODS
from xarm_control.robot import DEFAULT_GOAL


PROFILES = {
    'baseline': dict(trajectory='quintic', traj_duration=3.0, goal=DEFAULT_GOAL.tolist()),
    'fast': dict(trajectory='quintic', traj_duration=1.5, goal=DEFAULT_GOAL.tolist()),
    'slow': dict(trajectory='quintic', traj_duration=5.0, goal=DEFAULT_GOAL.tolist()),
    'cubic': dict(trajectory='cubic', traj_duration=3.0, goal=DEFAULT_GOAL.tolist()),
    'alternate': dict(trajectory='quintic', traj_duration=3.0,
                      goal=[0.0, -0.35, 0.0, 0.7, 0.0, 0.4, 0.0]),
}
DEFAULT_METHODS = ('pd_g', 'ctc', 'ctc_tuned', 'ctc_tuned_fast')


def _selected(s, allowed):
    vals = [x.strip() for x in s.split(',') if x.strip()]
    if not vals:
        raise ValueError('Empty selection')
    bad = [x for x in vals if x not in allowed]
    if bad:
        raise ValueError(f'Unknown selections: {bad}; allowed: {list(allowed)}')
    if len(set(vals)) != len(vals):
        raise ValueError('Duplicate selections')
    return vals


def _write_csv(path, rows, columns):
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def _make_comparison_plots(output, profiles, methods, rows):
    try:
        import matplotlib
        matplotlib.use('Agg')
        from matplotlib import pyplot as plt
    except ImportError:
        print('Matplotlib missing: all CSV files saved; comparison plots skipped.')
        return
    for profile in profiles:
        fig, ax = plt.subplots(figsize=(9, 5))
        for method in methods:
            ts_path = output / f'{profile}__{method}' / 'time_series.csv'
            with ts_path.open('r', newline='', encoding='utf-8') as f:
                ts = list(csv.DictReader(f))
            ax.plot([float(r['t']) for r in ts],
                    [float(r['joint_err_rad']) for r in ts], label=method)
        ax.set(xlabel='Time [s]', ylabel='Joint error norm [rad]',
               title=f'Joint tracking comparison — {profile}')
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig(output / f'compare_{profile}_joint_error.png', dpi=180)
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(max(9, len(profiles)*2.3), 5))
    x = np.arange(len(profiles))
    width = 0.8 / len(methods)
    for i, method in enumerate(methods):
        val = [next(r['rmse_joint_rad'] for r in rows
                    if r['profile'] == profile and r['method'] == method)
               for profile in profiles]
        ax.bar(x + (i-(len(methods)-1)/2)*width, val,
               width=width, label=method)
    ax.set_xticks(x, profiles)
    ax.set(ylabel='Joint norm RMSE [rad]', title='Tracking RMSE across profiles')
    ax.legend()
    ax.grid(axis='y')
    fig.tight_layout()
    fig.savefig(output / 'compare_rmse_all.png', dpi=180)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--methods', default=','.join(DEFAULT_METHODS),
                   help='Comma-separated controller names')
    p.add_argument('--profiles', default='baseline,fast,slow',
                   help='Comma-separated: baseline,fast,slow,cubic,alternate')
    p.add_argument('--duration', default=8.0, type=float,
                   help='Total simulation duration in seconds (>= trajectory duration)')
    p.add_argument('--ee-body', default='link7')
    args = p.parse_args()
    methods = _selected(args.methods, METHODS)
    profiles = _selected(args.profiles, PROFILES)
    if args.duration <= 0 or args.duration < max(PROFILES[x]['traj_duration'] for x in profiles):
        p.error('Simulation duration must cover the longest selected trajectory')

    output = (ROOT / 'results' / 'benchmarks' /
              f"{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}_{uuid4().hex[:6]}")
    output.mkdir(parents=True, exist_ok=False)
    manifest = dict(methods=methods, profiles={k: PROFILES[k] for k in profiles},
                    duration_s=args.duration, ee_body=args.ee_body,
                    note=('Same MJCF model, initial q=0, timestep, and actuator limits '
                          'for all trials. This benchmark does NOT retune each method.'))
    with (output/'benchmark_config.json').open('w', encoding='utf-8') as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)
    rows = []
    print('Benchmark directory:', output)
    for profile in profiles:
        cfg = PROFILES[profile]
        for method in methods:
            print(f'Running {profile} / {method} ...', flush=True)
            try:
                summary = experiment(method, duration=args.duration, viewer=False,
                                     write=True, ee_body=args.ee_body,
                                     trajectory=cfg['trajectory'],
                                     traj_duration=cfg['traj_duration'],
                                     goal=cfg['goal'],
                                     output_dir=output/f'{profile}__{method}')
            except Exception as exc:
                error_message = f'{type(exc).__name__}: {exc}'
                print('ERROR:', error_message)
                with (output/'FAILED.txt').open('a', encoding='utf-8') as f:
                    f.write(f'{profile}/{method}: {error_message}\n')
                raise  # Avoid presenting incomplete comparisons as complete.
            rows.append(dict(profile=profile, **summary))

    columns = ['profile', 'method', 'duration_s', 'trajectory', 'traj_duration_s',
               'steps', 'rmse_joint_rad', 'rmse_tcp_m', 'rmse_orient_rad',
               'final_joint_err_rad', 'peak_torque_Nm', 'saturated_fraction',
               'output_dir']
    columns += [f'rmse_J{i}_rad' for i in range(1, 8)]
    columns += [f'final_J{i}_error_rad' for i in range(1, 8)]
    _write_csv(output/'summary.csv', rows, columns)
    _make_comparison_plots(output, profiles, methods, rows)
    print('\nCOMPLETE. Summary:', output/'summary.csv')
    print('Plot:', output/'compare_rmse_all.png')
    print('All runs share exactly the same model per method/profile.')
    print('Reminder: peak torque and RMSE must be assessed jointly, not one metric alone.')


if __name__ == '__main__':
    main()
