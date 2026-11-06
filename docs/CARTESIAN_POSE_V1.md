# xArm7 — Independent Cartesian 6D pose planning (v1)

This **adds** separate files; do not replace or remove `run_cartesian_line.py` or existing controllers.

## Functionality

- Independent world-frame XYZ line and rotational geodesic (shortest axis-angle arc on SO(3)); quintic 0-to-1 timing.
- Warm-started 6D damped least-squares numerical inverse kinematics at 151 sampled poses (3 s at 0.02 s).
- Interpolated 7-joint quintic spline with zero endpoint velocity and acceleration; dense checks for joint limits, position deviation and orientation deviation.
- Tracking with **existing joint-space** `pd_g` or `ctc_tuned_fast`. This is not operational-space closed-loop control.
- CSV for planned waypoints and actual simulation; JSON config and summary; PNG plots.
- The body origin of `link7` is used as the end-effector reference; this is not a calibrated tool-tip.

## Installation from the Git repository

All required Python modules, tests, robot models, and mesh files are included in the repository. No ZIP extraction or manual file copying is required. From the repository root, create the Conda environment:

```bat
conda env create -f environment.yml
conda activate robotics
```

## Commands from project root

```bat
conda activate robotics
python -m unittest discover -s tests -v
python scripts\run_cartesian_pose.py --delta-xyz 0.02,0,0 --delta-rotvec-deg 0,0,0 --controller pd_g
```

First test adds an explicit fixed-orientation constraint for a +20 mm X translation. Then optionally try a **combined** 6D change:

```bat
python scripts\run_cartesian_pose.py --delta-xyz 0.02,0,0 --delta-rotvec-deg 0,0,5 --controller pd_g
```

`--delta-rotvec-deg` is a **world-frame rotation vector, not Euler angles**. `0,0,5` means 5 deg about world Z. This rotation left-multiplies the starting body orientation.

For visualization add `--view`; to use the tuned computed-torque joint controller use `--controller ctc_tuned_fast`. If IK preflight fails, it reports which waypoint was unreachable; it does NOT run a simulation or silently ignore the orientation constraint.

## Evaluation

Separate measures for (1) planning accuracy: position and orientation IK residuals; dense trajectory deviations, and (2) dynamic tracking: move/hold/full RMS and final position/orientation errors, peak actuator command and saturation. All saved in `results/cartesian_pose/<unique_run>/`.

## Limitations

- No collision avoidance, singularity-proof guarantee, or real-robot validation. DLS numerical IK may fail in near-singular/unreachable poses. The 100 mm and 15 deg demo bounds are a conservative test scope, not collision safety certificates.
- No physical wrist tool offset is configured. You are targeting `link7` body origin.
- A 6D geometric goal leaves 1 null-space degree of freedom for a 7DOF robot; this solver uses the previous waypoint as warm start without explicit null-space optimization.
- Only tests of synthetic kinematics can run without the actual MuJoCo runtime and robot assets. The xArm7 6D preflight and dynamics **must be executed on the user's machine before declaring successful integration**.
- Reference path orientation is explicitly planned and constrained in IK, but actual physical orientation is only as well controlled as the joint tracking controller; there is no independent task-space feedback in this version.
