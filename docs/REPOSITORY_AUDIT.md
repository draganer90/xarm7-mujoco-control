# Repository audit and scope freeze (27 October 2026)

## Basis

Inspected the owner's source-only inventory and source archive (four model files, active library modules, script files, README files, dependencies and tests). The archive does not contain robot mesh files, recorded numerical results, or a complete release-grade example; model-dependent simulation was **not executed** during this audit.

The original inventory categorized 7 library files (including 2 controller backups), 4 main entrypoints, 18 diagnostic-experiment scripts, 13 other scripts, 1 old experiment backup, 4 model files, 2 documentation files and 1 test file. Many diagnostics rely on `Path(__file__).resolve().parents[1]`; moving them without refactoring changes their assumed project root.

## Keep in the public-facing runtime

- `xarm_control/{robot,controllers,trajectories,pin_models}.py`, plus package init.
- `scripts/{run_experiment,compare_controllers,verify_models}.py`.
- `scripts/benchmark.py` is retained but labeled legacy; it should not be presented as the final fair benchmark.
- Both relevant robot models (MJCF and URDF) and local meshes, subject to license verification.
- `environment.yml`, `.gitignore`, automated tests, docs and an unambiguous root README.

## Preserve, but separate

- Friction investigations, exploratory tuning sweeps, diagnostics and former standalone demonstrations.
- Backups of `controllers.py` and `run_experiment.py`.
- All run-specific CSVs, plots and original reports.

These are historical research records. Keep the working original intact; optionally add a source archive (`--archive-original-experiments`) to the publication candidate. Archived scripts are **not** claimed runnable from their new archive location.

## Technical debt, prioritized

1. **Documentation drift.** Previous READMEs still refer to a starter package and a prior Windows username. They do not reflect completed tests.
2. **Task/reference design.** `Controller.ref` starts from seven zero joint angles. Cartesian references come from joint interpolation, not independently planned poses or paths.
3. **Task frame naming.** The `link7` body origin is labeled as TCP in metrics, but an actual tool site/calibration is absent.
4. **Research defaults.** The `cart_ctc_fric` defaults are velocity-based alpha 0.5, NOT the previously explored predicted-velocity alpha 0.8. Avoid equating these in reports. Earlier dry-friction heuristics had unsteady/oscillatory behavior.
5. **Benchmark comparability.** RMSE over a fixed 8 s horizon mixes motion and holding intervals; distinguish these and preserve torque/saturation data.
6. **Test coverage.** Only one test file checks cubic/quintic timing; add model loading, actuator limits and benchmark regression smoke tests before publication.
7. **Dependency report.** `pin` distribution version can be 4.1.0 while package metadata for `pinocchio` is not separately found; `import pinocchio` and real model tests are the authoritative environment check.
8. **Licensing.** Mesh and upstream source attribution are not documented sufficiently to assume public redistribution rights.

## Decision

Freeze friction tuning and historic exploratory control experiments. Focus on (a) a reproducible project skeleton, (b) independent Cartesian target/path generation and IK, and (c) one predefined final benchmark followed by documentation. Do not alter baseline controller parameters while preparing the repository.
