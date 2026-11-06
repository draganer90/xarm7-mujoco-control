"""Independent Cartesian *position* line planning and numerical 7-DOF IK.

Pure NumPy/SciPy module. No MuJoCo calls here: the caller supplies an FK +
position Jacobian callback. Position is the specified body-frame origin in
world coordinates; this module does NOT regulate orientation or collisions.
"""

from dataclasses import dataclass
import numpy as np
from scipy.interpolate import make_interp_spline


@dataclass(frozen=True)
class CartesianLinePlan:
    t: np.ndarray
    xyz: np.ndarray
    q: np.ndarray
    target_start: np.ndarray
    target_end: np.ndarray
    ik_max_residual_m: float
    ik_max_iterations: int


def _validate_vec(name, x, size):
    a = np.asarray(x, dtype=float)
    if a.shape != (size,) or not np.all(np.isfinite(a)):
        raise ValueError(f'{name} must contain {size} finite numbers')
    return a.copy()


def quintic_scale(t, duration):
    """Clamped quintic time scaling 0->1, zero speed/acceleration at ends."""
    if duration <= 0 or not np.isfinite(duration):
        raise ValueError('duration must be positive and finite')
    u = np.clip(np.asarray(t, dtype=float)/duration, 0.0, 1.0)
    return 10*u**3 - 15*u**4 + 6*u**5


def line_samples(p0, p1, duration, sample_dt=0.02):
    p0, p1 = _validate_vec('p0', p0, 3), _validate_vec('p1', p1, 3)
    if not np.isfinite(sample_dt) or sample_dt <= 0:
        raise ValueError('sample_dt must be positive and finite')
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError('duration must be positive and finite')
    n = max(6, int(np.ceil(duration/sample_dt)))
    t = np.linspace(0.0, duration, n+1)
    s = quintic_scale(t, duration)
    return t, p0[None, :] + s[:, None]*(p1-p0)[None, :]


def ik_position(fk_and_jacobian, q_seed, target_xyz, lower, upper,
                tolerance_m=2e-5, damping=0.01, max_step_rad=0.15,
                max_iterations=120):
    """Damped least-squares 3D position IK with per-step clamp and joint limits.

    fk_and_jacobian(q) -> (p[3], J_pos[3, nq]); warm-start with prior waypoint.
    Raise on nonconvergence, rather than silently returning a wrong solution.
    """
    q = np.asarray(q_seed, dtype=float).copy()
    pgoal = _validate_vec('target_xyz', target_xyz, 3)
    low = np.asarray(lower, dtype=float)
    high = np.asarray(upper, dtype=float)
    if q.ndim != 1 or not np.all(np.isfinite(q)) or low.shape != q.shape or high.shape != q.shape:
        raise ValueError('Invalid seed or joint limit shapes')
    if np.any(low > high) or np.any(q < low) or np.any(q > high):
        raise ValueError('Seed is outside joint limits or limits are inverted')
    if not 0 < tolerance_m < 0.1 or damping <= 0 or max_step_rad <= 0 or max_iterations < 1:
        raise ValueError('Invalid IK solver options')
    for iteration in range(max_iterations+1):
        p, J = fk_and_jacobian(q)
        p, J = np.asarray(p, dtype=float), np.asarray(J, dtype=float)
        if p.shape != (3,) or J.shape != (3, len(q)) or not np.all(np.isfinite(J)):
            raise ValueError('FK callback must return position (3,) and Jacobian (3,nq)')
        error = pgoal-p
        residual = float(np.linalg.norm(error))
        if residual <= tolerance_m:
            return q, residual, iteration
        if iteration == max_iterations:
            break
        # Damped right pseudoinverse J.T @ inv(JJ.T + lambda^2 I).
        dq = J.T @ np.linalg.solve(J @ J.T + damping*damping*np.eye(3), error)
        scale = min(1.0, max_step_rad/max(1e-15, float(np.max(np.abs(dq)))))
        q_next = np.clip(q + scale*dq, low, high)
        if np.linalg.norm(q_next-q) < 1e-12:
            break
        q = q_next
    raise RuntimeError(f'Cartesian IK did not converge, residual={residual:.6g} m')


def plan_cartesian_line(fk_and_jacobian, q_start, target_xyz, lower, upper,
                        duration=3.0, sample_dt=0.02, tolerance_m=2e-5,
                        damping=0.01, max_step_rad=0.15):
    q0 = np.asarray(q_start, dtype=float).copy()
    p0, _ = fk_and_jacobian(q0)
    p0 = _validate_vec('initial FK position', p0, 3)
    p1 = _validate_vec('target_xyz', target_xyz, 3)
    t, xyz = line_samples(p0, p1, duration, sample_dt)
    path = [q0]
    worst_residual, worst_iters = 0., 0
    for k in range(1, len(t)):
        try:
            qk, err, iters = ik_position(
                fk_and_jacobian, path[-1], xyz[k], lower, upper,
                tolerance_m=tolerance_m, damping=damping,
                max_step_rad=max_step_rad)
        except RuntimeError as exc:
            raise RuntimeError(f'Waypoint {k}/{len(t)-1}, t={t[k]:.3f}s: {exc}') from exc
        path.append(qk)
        worst_residual = max(worst_residual, err)
        worst_iters = max(worst_iters, iters)
    return CartesianLinePlan(t=t, xyz=xyz, q=np.asarray(path),
                             target_start=p0, target_end=p1,
                             ik_max_residual_m=worst_residual,
                             ik_max_iterations=worst_iters)


class JointSpline:
    """C2 joint reference of an IK path, with zero endpoint vel and accel.

    A quintic interpolating spline may overshoot between sampled IK waypoints;
    the caller MUST check joint limits and FK residuals on the dense spline.
    """

    def __init__(self, plan):
        t, q = np.asarray(plan.t), np.asarray(plan.q)
        if t.ndim != 1 or q.shape[0] != len(t) or q.ndim != 2 or len(t) < 7:
            raise ValueError('Expected at least 7 planned time samples')
        zeros = np.zeros(q.shape[1])
        bc = ([(1, zeros), (2, zeros)], [(1, zeros), (2, zeros)])
        self.spline = make_interp_spline(t, q, k=5, bc_type=bc, axis=0)
        self.speed = self.spline.derivative(1)
        self.accel = self.spline.derivative(2)
        self.duration = float(t[-1])
        self.q_start, self.q_end = q[0].copy(), q[-1].copy()

    def evaluate(self, t):
        t = float(t)
        if t <= 0:
            return self.q_start.copy(), np.zeros_like(self.q_start), np.zeros_like(self.q_start)
        if t >= self.duration:
            return self.q_end.copy(), np.zeros_like(self.q_end), np.zeros_like(self.q_end)
        return (np.asarray(self.spline(t)), np.asarray(self.speed(t)),
                np.asarray(self.accel(t)))


def validate_spline(reference, fk_and_jacobian, initial_xyz, goal_xyz,
                    lower, upper, samples=601, max_path_error_m=0.001):
    """Safety preflight: dense-sample FK deviations and joint bounds."""
    t = np.linspace(0.0, reference.duration, int(samples))
    p0, p1 = _validate_vec('initial_xyz', initial_xyz, 3), _validate_vec('goal_xyz', goal_xyz, 3)
    worst_residual, worst_limit = 0.0, 0.0
    for ti in t:
        q, _, _ = reference.evaluate(ti)
        limit_error = max(0., float(np.max(np.asarray(lower)-q)),
                          float(np.max(q-np.asarray(upper))))
        worst_limit = max(worst_limit, limit_error)
        p, _ = fk_and_jacobian(q)
        xyz_target = p0 + quintic_scale(ti, reference.duration)*(p1-p0)
        worst_residual = max(worst_residual, float(np.linalg.norm(p-xyz_target)))
    if worst_limit > 1e-8:
        raise RuntimeError(f'IK interpolation exceeded joint limits by {worst_limit:.3g} rad')
    if worst_residual > max_path_error_m:
        raise RuntimeError(f'IK interpolation departed Cartesian line by {worst_residual*1000:.3f} mm')
    return worst_residual
