# Final release checklist (finite — no further gain tuning)

**Working copies:** Keep the original release candidate in the OneDrive project archive. The Git working copy lives separately outside OneDrive; do not let OneDrive synchronize its `.git` internals. Keep both in agreement when making documented releases.

## Phase A — code and verification

- [x] Preserve the original, experiment-rich `xarm7` directory separately.
- [x] Create a clean release candidate with models, meshes and source code (builder reported 8 asset paths checked).
- [x] Verify Pinocchio/MuJoCo mass-bias at two reference states (observed PASS).
- [x] Verify a 3D independent line and a 6D independent pose trajectory with the original xArm7 model (observed PASS for one +20 mm path and two pose variants).
- [x] **Full unit test suite completed after 6D installation**: 12/12 PASS (27 Oct 2026).
- [x] **Independent clean-copy check:** fresh local Git clone, separate Conda environment (Windows 11, Python 3.11.18, MuJoCo 3.17.0), 12/12 tests PASS, model verification PASS, PD+G, 3D and 6D simulations PASS. No model or controller parameters changed.

## Phase B — report and curated results

- [x] Archive a historical 12-run benchmark summary and plot in `docs/data/` and `docs/figures/` with provenance caveat.
- [x] Update README and scientific limitations to include independent 3D/6D planning.
- [x] Technical report written: `docs/xArm7_Tehnicki_izvjestaj_MuJoCo_Pinocchio_2026.docx`.
- [x] Curated report figures and benchmark data; large diagnostic outputs remain untracked.

## Phase C — publication blockers

- [ ] Verify upstream UFACTORY xArm URDF/mesh provenance and license conditions; retain applicable notices.
- [x] Select a license for original Python code **without** relicensing third-party assets: BSD-3-Clause, Copyright (c) 2026, Dragan Erceg.
- [ ] Check for credentials, user paths and large local archives. `BUILD_MANIFEST.json` records the source path; it should NOT be published without sanitization (this package's `.gitignore` excludes it by default).
- [ ] Create a **private** GitHub repository and verify a clean clone. Make public only after licensing and attribution review.

**Scope exclusions:** no hardware operation, collision checking, calibrated tool center point, general static-friction convergence proof, universal IK feasibility, or controller tuning to zero error. These are not unfinished promises for this simulation-only release.

**Exit criterion:** all unit tests pass on the final files; model and 3D/6D examples run on a clean environment; representative baseline results are reproducible within stated tolerance; report, attribution and licensing are in place. Then the research/teaching simulation project can be called complete.
