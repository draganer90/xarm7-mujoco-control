"""Joint-torque and Cartesian controllers for the simulated 7-DOF xArm7.

Tuned CTC gains below come from the 2026-10-26 single-goal sweep in MuJoCo.
They are NOT hardware-safe gains and are not validated on other trajectories.
"""
import numpy as np
import mujoco
from .robot import (gravity, mass_and_bias, get_body_id,
                    body_pose_and_jacobian, orientation_error_world,
                    jacobian_dot_times_qvel)
from .trajectories import quintic, cubic

GOOD_KP = np.array([80, 80, 60, 60, 40, 30, 20], dtype=float)
GOOD_KD = np.array([12, 12, 10, 10, 6, 5, 4], dtype=float)
CALCULATED_KP = np.array([3.77, 15.51, 9.09, 7.31, 0.09, 0.19, 0.0], dtype=float)
CALCULATED_KD = np.array([1.88, 7.75, 4.55, 3.65, 0.04, 0.09, 0.0], dtype=float)

# Acceleration-space frequency gains (not joint torque-space gains).
CTC_ZETA = 1.0
CTC_OMEGA_ORIGINAL = np.full(7, 5.0, dtype=float)
CTC_OMEGA_TUNED = np.array([20, 20, 20, 20, 40, 50, 50], dtype=float)
CTC_OMEGA_TUNED_FAST = np.array([20, 20, 20, 20, 50, 50, 50], dtype=float)

# Experimental CARTESIAN CTC friction feedforward. Simulation ONLY.
# Dry friction in MuJoCo is solver-managed; this is a smooth heuristic,
# NOT the exact inverse of the friction constraint. Start with J6 alone
# because prior velocity-based tanh feedforward destabilized J7.
CART_FRIC_ALPHA_DEFAULT = 0.5
CART_FRIC_EPS_DEFAULT = 0.1  # rad/s
CART_FRIC_MASK_DEFAULT = np.array([0, 0, 0, 0, 0, 1, 0], dtype=float)
CART_FRIC_PREVIEW_DEFAULT = 0.1  # s, for diagnostic predicted_velocity option

METHODS = ('zero', 'pd', 'pd_g', 'pd_g_calculated', 'ctc', 'ctc_friction',
           'ctc_passive', 'ctc_tuned', 'ctc_tuned_fast', 'pin_ctc_tuned',
           'cart_pos', 'cart_pose', 'cart_ctc', 'cart_ctc_fric', 'pin_ctc')
CTC_METHODS = ('ctc', 'ctc_friction', 'ctc_passive', 'ctc_tuned',
               'ctc_tuned_fast', 'pin_ctc', 'pin_ctc_tuned')


def controller_settings(method):
    """Human-readable controller settings for experiment provenance."""
    if method in ('pd', 'pd_g'):
        return {'Kp_joint': GOOD_KP.tolist(), 'Kd_joint': GOOD_KD.tolist(),
                'gravity_compensation': method == 'pd_g'}
    if method == 'pd_g_calculated':
        return {'Kp_joint': CALCULATED_KP.tolist(), 'Kd_joint': CALCULATED_KD.tolist(),
                'gravity_compensation': True}
    if method in CTC_METHODS:
        omega = (CTC_OMEGA_TUNED if method == 'ctc_tuned' else
                 CTC_OMEGA_TUNED_FAST if method in ('ctc_tuned_fast', 'pin_ctc_tuned')
                 else CTC_OMEGA_ORIGINAL)
        return {'omega_n_rad_s': omega.tolist(), 'zeta': CTC_ZETA,
                'Kp_acceleration': (omega**2).tolist(),
                'Kd_acceleration': (2*CTC_ZETA*omega).tolist(),
                'compensate_passive': method in ('ctc_passive', 'ctc_tuned',
                                                 'ctc_tuned_fast', 'pin_ctc_tuned'),
                'uses_pinocchio_dynamics': method in ('pin_ctc', 'pin_ctc_tuned'),
                'tanh_friction_feedforward': method == 'ctc_friction'}
    if method == 'cart_ctc_fric':
        return {'controller': 'cart_ctc plus bounded smooth friction feedforward',
                'alpha': CART_FRIC_ALPHA_DEFAULT,
                'eps_rad_s': CART_FRIC_EPS_DEFAULT,
                'joint_mask_J1_to_J7': CART_FRIC_MASK_DEFAULT.tolist(),
                'direction': 'velocity',
                'note': 'Experimental heuristic, simulation only; no exact Coulomb inversion'}
    return {'note': 'Cartesian or zero-torque controller; see controllers.py'}


class Controller:
    def __init__(self, model, method, project_root, goal, duration,
                 ee_body='link7', trajectory='quintic', traj_duration=None,
                 fric_alpha=CART_FRIC_ALPHA_DEFAULT,
                 fric_eps=CART_FRIC_EPS_DEFAULT, fric_mask=None,
                 fric_direction='velocity',
                 fric_preview=CART_FRIC_PREVIEW_DEFAULT):
        if method not in METHODS:
            raise ValueError(f'Unknown method {method}; choices {METHODS}')
        if trajectory not in ('quintic', 'cubic'):
            raise ValueError('trajectory must be quintic or cubic')
        if duration <= 0:
            raise ValueError('duration must be positive')
        self.m = model
        self.method = method
        self.goal = np.asarray(goal, dtype=float)
        if self.goal.shape != (7,) or not np.all(np.isfinite(self.goal)):
            raise ValueError('Expected 7 finite goal joint angles')
        self.duration = float(duration)
        self.traj_duration = (min(3.0, self.duration*0.75)
                              if traj_duration is None else float(traj_duration))
        if not (0.0 < self.traj_duration <= self.duration):
            raise ValueError('traj_duration must be > 0 and <= simulation duration')
        self.trajectory = trajectory
        self.gravity_data = mujoco.MjData(model)
        self.target_data = mujoco.MjData(model)
        self.body_id = None
        self.pinmodel = None
        # Additional options are used only by the experimental cart_ctc_fric method.
        self.fric_alpha = float(fric_alpha)
        self.fric_eps = float(fric_eps)
        self.fric_mask = (CART_FRIC_MASK_DEFAULT.copy() if fric_mask is None
                          else np.asarray(fric_mask, dtype=float).copy())
        self.fric_direction = str(fric_direction)
        self.fric_preview = float(fric_preview)
        self.last_friction_ff = np.zeros(7, dtype=float)
        if not (0.0 <= self.fric_alpha <= 1.0 and np.isfinite(self.fric_alpha)):
            raise ValueError('fric_alpha must be finite and in [0, 1]')
        if not (0.0 < self.fric_eps and np.isfinite(self.fric_eps)):
            raise ValueError('fric_eps must be finite and > 0 [rad/s]')
        if (self.fric_mask.shape != (7,) or not np.all(np.isfinite(self.fric_mask))
                or np.any(self.fric_mask < 0) or np.any(self.fric_mask > 1)):
            raise ValueError('fric_mask must contain seven numbers in [0, 1]')
        if self.fric_direction not in ('velocity', 'predicted_velocity'):
            raise ValueError('fric_direction must be velocity or predicted_velocity')
        if not (np.isfinite(self.fric_preview) and self.fric_preview >= 0):
            raise ValueError('fric_preview must be finite and >= 0 [s]')
        if method.startswith('cart_'):
            self.body_id = get_body_id(model, ee_body)
        if method in ('pin_ctc', 'pin_ctc_tuned'):
            from .pin_models import PinModel
            self.pinmodel = PinModel(project_root, model)

    def ref(self, t):
        trajectory_fn = quintic if self.trajectory == 'quintic' else cubic
        return trajectory_fn(np.zeros(7), self.goal, t, self.traj_duration)

    def cart_ref(self, t, full_pose):
        qd, vd, ad = self.ref(t)
        td = self.target_data
        td.qpos[:], td.qvel[:] = qd, vd
        mujoco.mj_forward(self.m, td)
        p, R, J = body_pose_and_jacobian(self.m, td, self.body_id, full_pose)
        twist = J @ vd
        accel = J @ ad + jacobian_dot_times_qvel(self.m, td, self.body_id, full_pose)
        return p, R, twist, accel

    def command(self, data):
        q, v = data.qpos.copy(), data.qvel.copy()
        t = data.time
        qd, vd, ad = self.ref(t)
        method = self.method
        if method == 'zero':
            return np.zeros(7)
        if method in ('pd', 'pd_g', 'pd_g_calculated'):
            Kp, Kd = ((CALCULATED_KP, CALCULATED_KD)
                      if method == 'pd_g_calculated' else (GOOD_KP, GOOD_KD))
            torque = Kp*(qd-q) + Kd*(vd-v)
            if method != 'pd':
                torque += gravity(self.m, self.gravity_data, q)
            return torque
        if method in CTC_METHODS:
            if method == 'ctc_tuned':
                omega = CTC_OMEGA_TUNED
            elif method in ('ctc_tuned_fast', 'pin_ctc_tuned'):
                omega = CTC_OMEGA_TUNED_FAST
            else:
                omega = CTC_OMEGA_ORIGINAL
            a_ref = (ad + 2*CTC_ZETA*omega*(vd-v) + (omega**2)*(qd-q))
            if method in ('pin_ctc', 'pin_ctc_tuned'):
                M, b = self.pinmodel.mass_bias(q, v)
            else:
                M, b = mass_and_bias(self.m, data)
            torque = M @ a_ref + b
            if method in ('ctc_passive', 'ctc_tuned', 'ctc_tuned_fast', 'pin_ctc_tuned', 'ctc_friction'):
                # MuJoCo qfrc_passive includes e.g. joint viscous damping.
                # Dry friction (frictionloss) is a constraint, NOT in qfrc_passive.
                torque -= data.qfrc_passive.copy()
            if method == 'ctc_friction':
                # Legacy experimental option retained for reproducibility only.
                # tanh is NOT MuJoCo's actual dry-friction constraint law.
                torque += self.m.dof_frictionloss * np.tanh(v/0.02)
            return torque
        full_pose = method != 'cart_pos'
        p, R, J = body_pose_and_jacobian(self.m, data, self.body_id, full_pose)
        pdes, Rdes, twist_des, accel_des = self.cart_ref(t, full_pose)
        ep = pdes-p
        if full_pose:
            er = orientation_error_world(Rdes, R)
            err = np.hstack((ep, er))
            Kp = np.diag([350, 350, 350, 45, 45, 45])
            Kd = np.diag([35, 35, 35, 9, 9, 9])
        else:
            err = ep
            Kp = np.diag([350, 350, 350])
            Kd = np.diag([35, 35, 35])
        err_rate = twist_des - J @ v
        if method in ('cart_pos', 'cart_pose'):
            return gravity(self.m, self.gravity_data, q) + J.T @ (Kp @ err + Kd @ err_rate)
        M, b = mass_and_bias(self.m, data)
        lam = .08
        MinvJT = np.linalg.solve(M, J.T)
        Jbar = MinvJT @ np.linalg.solve(J @ MinvJT + lam**2*np.eye(J.shape[0]),
                                       np.eye(J.shape[0]))
        jd_v = jacobian_dot_times_qvel(self.m, data, self.body_id, full_pose)
        desired_cart_acc = accel_des + Kp @ err + Kd @ err_rate
        projector = np.eye(7) - Jbar @ J
        qdd_cmd = Jbar @ (desired_cart_acc - jd_v) + projector @ (5*(qd-q) - 3*v)
        torque = M @ qdd_cmd + b - data.qfrc_passive.copy()
        if method == 'cart_ctc_fric':
            # Read actual dry-friction magnitudes; do not change model physics.
            # At v=0 the velocity mode yields 0 Nm, so it cannot by itself
            # break static friction. Predicted velocity is a separate diagnostic.
            v_signal = (v if self.fric_direction == 'velocity'
                        else v + self.fric_preview*qdd_cmd)
            self.last_friction_ff[:] = (self.fric_alpha * self.fric_mask *
                                        self.m.dof_frictionloss *
                                        np.tanh(v_signal/self.fric_eps))
            torque += self.last_friction_ff
        return torque


def clip_torque(model, tau):
    """Clip torque commands to the XML actuator control ranges."""
    if not np.all(model.actuator_ctrllimited):
        raise ValueError('Please set actuator ctrllimited on all motors')
    limits = model.actuator_ctrlrange
    clipped = np.clip(tau, limits[:, 0], limits[:, 1])
    return clipped, bool(np.any(np.abs(clipped-tau) > 1e-9))
