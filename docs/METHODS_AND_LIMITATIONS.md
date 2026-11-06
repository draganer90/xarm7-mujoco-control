# Methods and interpretation

## Controllers

- Joint-space: `pd`, `pd_g`, `pd_g_calculated`, `ctc`, `ctc_passive`, `ctc_tuned`, `ctc_tuned_fast`, `pin_ctc`, `pin_ctc_tuned`.
- Task-space feedback: `cart_pos` (position), `cart_pose` (position + orientation), `cart_ctc` (resolved-acceleration inverse dynamics with regularized Jacobian and null-space term).
- Experimental: `ctc_friction` and `cart_ctc_fric` are heuristic dry-friction compensation examples, **not a solved/validated static-friction controller**.

The default `cart_ctc_fric` settings in the inspected core code are not equivalent to every diagnostic configuration. The particular predicted-velocity, J6-only alpha=0.8 tests were exploratory and were often run in diagnostic scripts, not as the default `cart_ctc_fric` CLI configuration.

## Trajectory types (avoid conflation)

1. `run_experiment.py` creates joint cubic/quintic references; its Cartesian reference pose is obtained by forward kinematics of the joint reference.
2. `run_cartesian_line.py` independently prescribes a world-frame **XYZ line** and solves position-only numerical IK for a sampled joint trajectory; orientation is unconstrained.
3. `run_cartesian_pose.py` independently prescribes a world-frame **XYZ line plus SO(3) orientation arc** and solves 6D numerical IK; execution is still by a joint-space torque controller. It is not direct closed-loop Cartesian pose control.

The measured endpoint is the `link7` body origin, not a calibrated tool center point. Limits are checked at IK waypoints and dense spline sampling, but **collisions are not checked**. Success on a short test path is not a global reachability or safety proof.

## Model/dynamics cautions

- MuJoCo `qfrc_bias` includes gravity and velocity-dependent rigid-body bias. `qfrc_passive` covers passive forces, while `frictionloss` is handled as a constraint; one cannot assume `qfrc_passive` fully captures dry friction.
- Frictionless in-memory variants were used only for cause isolation; they are NOT controller rankings against the original plant.
- Pinocchio vs MuJoCo mass/bias agreement at two states and trajectory-level equivalence is strong consistency evidence for those cases, not a proof of agreement over all states.
- The 7-DOF arm is redundant for 6D tool pose, so joint error with respect to one chosen joint reference is not interchangeable with tool-pose error.
- Model limits/gains are for simulation, not certified hardware parameters.
- No thermal, actuator bandwidth, measurement noise, real hardware interlock, workspace obstacle, or collision safety validation is implied.

## Scientific interpretation

Compare controller methods with unchanged model, initial conditions, trajectory, friction and torque limits. Report joint norm RMSE [rad], position RMSE [mm], orientation error [rad], peak torque [Nm], saturation and separate movement/hold windows. Do not rank different trajectory durations solely from full 8-second RMSE because the holding interval differs.

The investigated friction strategies showed a trade-off: continuous compensation reduced orientation error but could cause prolonged J6 motion; a fixed-time fade reduced speed but left more residual pose error for harder goals. This is a legitimate **limitation** rather than a reason to tune indefinitely.
