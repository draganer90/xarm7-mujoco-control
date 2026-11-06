# Frozen experiment protocol — release candidate

The goal of this repository is reproducible demonstration and comparison of implemented algorithms, **not universal controller optimality**.

## Joint-controller benchmark already performed

- Original MuJoCo 7-axis model with damping and `frictionloss` unchanged.
- Initial joint state `q = 0`, `dq = 0`.
- Same target `[0,-0.6,0,1,0,0.6,0]` [rad] for four methods: `pd_g`, original `ctc`, `ctc_tuned`, `ctc_tuned_fast`.
- Quintic motion durations: 1.5, 3.0, 5.0 s; total simulation duration 8.0 s each.
- Joint norm RMSE, pose RMSE, peak actuator control effort and saturation reported.
- The supplied 12-row historical summary, without Windows-specific output paths, is in `docs/data/joint_controller_benchmark_20261026.csv`.

This is a fixed, documented historical benchmark. It is not necessary to tune or rerun it repeatedly. One clean checkout comparison against the archived baseline suffices as a release check.

## 3D/6D planning functional checks already performed

- 3D: 20 mm world-X displacement; position-only IK; PD+G joint tracking; 151 planned waypoints.
- 6D: same XYZ target, with both fixed orientation (0°) and 5° world-Z orientation change; 6D IK and PD+G joint tracking; 151 planned waypoints.
- These were functional feasibility checks on one small path, **not** a globally representative planning benchmark.

## Final release acceptance (finite checks, no gain tuning)

1. Execute all unit tests once after installing both path-planning modules.
2. Verify model loading / Pinocchio mass-bias agreement, and run one PD+G baseline.
3. Verify the 3D and 6D example commands from the README on an independent checkout / second computer.
4. Confirm input model assets are available under relative paths and output files are created; no absolute username path is required in code.
5. Freeze results, document discrepancies if any, do not adjust gains to match desired outcomes.

## Reporting rules

- Distinguish planner numerical IK residual, reference-spline geometric deviation and actual simulated tracking error.
- Distinguish movement from holding errors; reference times differ across profiles.
- Cite original model and mesh sources, acknowledge third-party licenses, and do not publish historical raw diagnostics automatically.
- Use `link7` origin unless a calibrated TCP site is added and validated in a future project.
