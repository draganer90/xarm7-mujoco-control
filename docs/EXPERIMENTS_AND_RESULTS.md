# Selected reproduced and user-observed results

These values come from **terminal outputs supplied by the project owner** in October 2026 and the 12-row `summary.csv` archive. This document does not claim independent execution in the author's container. All results are simulation-only.

## Fixed 8-second joint-controller benchmark

| Method | Joint RMSE (rad), 3s motion | `link7` position RMSE (mm), 3s motion | Orientation RMSE (rad), 3s motion |
|---|---:|---:|---:|
| `pd_g` | 0.046911 | 20.219 | 0.031089 |
| `ctc` | 0.608160 | 58.283 | 0.401459 |
| `ctc_tuned` | 0.047712 | 9.346 | 0.017321 |
| `ctc_tuned_fast` | 0.047206 | 9.370 | 0.014496 |

*The column label “3s motion” refers to a 3-second reference motion within an **8-second total** RMSE window. Results are NOT movement-only RMSE.*

At the 3-second motion profile, `pin_ctc_tuned` gave joint RMSE 0.0472057925 rad, position RMSE 9.3697945 mm, and orientation RMSE 0.0144957499 rad, matching MuJoCo tuned CTC closely for that trial.

See [archived benchmark CSV](data/joint_controller_benchmark_20261026.csv) and [comparison chart](figures/joint_rmse_20261026.png) for all baseline/fast/slow profiles.

## Independent Cartesian planning

| Example | Worst IK waypoint position error | Worst IK waypoint orientation error | Simulated `link7` final position error | Simulated final orientation error |
|---|---:|---:|---:|---:|
| 3D line, +20 mm X | 0.0190 mm | Unconstrained | 1.777 mm | Orientation drift; RMSE = 0.006760 rad |
| 6D, +20 mm X, fixed orientation | 0.0189 mm | 2.24e-8 rad | 1.775 mm | 0.002731 rad |
| 6D, +20 mm X, +5° Z rotation | 0.0189 mm | 8.27e-5 rad | 1.780 mm | 0.002750 rad |

All three used 151 IK waypoints, PD+G tracking, an 8-second simulation and a 3-second reference motion; no actuator saturation was reported.

**Do not compare planning residuals to tracking RMSE as if they were the same metric.** Numerical IK residuals are near zero; the simulated plant follows with nonzero dynamic tracking error.

## Cartesian CTC friction research — closed topic

Changing the physical model to remove dry friction *in memory* reduced orientation RMSE from 0.373761 to 0.005769 rad, supporting a strong influence of dry friction in the tested configuration. On the **unchanged original plant**, predicted-velocity J6-only feedforward at alpha=0.8 reduced orientation RMSE to 0.111886 rad on the baseline 3 s reference, but provoked persistent hold motion. At a 20-second horizon the continuous alpha=0.8 case still showed repeated J6 direction reversals for G0/G1 and nonzero J6 RMS hold speed. Fixed-time fade stopped most motion but increased orientation residual for the harder G2/G3 goals. Further controller tuning is out of scope; report these limitations without promising general robustness.
