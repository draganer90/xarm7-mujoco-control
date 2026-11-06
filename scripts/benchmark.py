"""Run reproducible headless benchmark using the same initial/goal poses."""
import csv
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.run_experiment import experiment


def main():
    joint = ['zero', 'pd', 'pd_g_calculated', 'pd_g', 'ctc', 'ctc_friction']
    cart = ['cart_pos', 'cart_pose', 'cart_ctc']
    rows = []
    for m in joint + cart:
        print(f'Running {m} ...', flush=True)
        try:
            result = experiment(m, duration=8.0, viewer=False, write=True)
            result['status'] = 'PASS'
            rows.append(result)
        except Exception as exc:
            print(f'FAIL {m}: {type(exc).__name__}: {exc}')
            rows.append(dict(method=m, status=f'FAIL: {type(exc).__name__}: {exc}'))
    out = ROOT/'results'
    out.mkdir(exist_ok=True)
    keys = ['method', 'status', 'steps', 'rmse_joint_rad', 'rmse_tcp_m', 'rmse_orient_rad',
            'final_joint_err_rad', 'peak_torque_Nm', 'saturated_fraction']
    with (out/'benchmark_summary.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)
    print('Saved:', out/'benchmark_summary.csv')
    print('Important: joint-space (rad) and Cartesian (m/rad) errors require separate comparisons.')


if __name__ == '__main__':
    main()
