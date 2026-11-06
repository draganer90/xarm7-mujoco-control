"""Independent world-frame 6D Cartesian pose planning and damped-least-squares IK.

Position: link7 body origin in metres. Orientation: rotation matrix in SO(3).
A quintic time law drives a straight XYZ line and a geodesic SO(3) rotation.
The callback receives q and returns (p_world, R_world, J_world[6,nq]), where
J[:3] is geometric linear velocity and J[3:] is world angular velocity.

This module generates JOINT references via IK; it is NOT a direct Cartesian
feedback controller and does NOT check collisions or hardware-safe dynamics.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation
from xarm_control.cartesian_planning import JointSpline, quintic_scale


@dataclass(frozen=True)
class CartesianPosePlan:
    t: np.ndarray
    xyz: np.ndarray
    rotations: np.ndarray
    q: np.ndarray
    position_residual_max_m: float
    orientation_residual_max_rad: float
    ik_max_iterations: int


def _array(name, value, shape):
    a = np.asarray(value, dtype=float)
    if a.shape != shape or not np.all(np.isfinite(a)):
        raise ValueError(f'{name}: expected finite array of shape {shape}')
    return a.copy()


def _rotation(name, R):
    R = _array(name, R, (3, 3))
    if not (np.allclose(R.T @ R, np.eye(3), atol=1e-6)
            and np.linalg.det(R) > 0.999999):
        raise ValueError(f'{name}: not a valid right-handed rotation matrix')
    return R


def pose_error(p_goal, R_goal, p_actual, R_actual):
    """Return world-frame position and orientation error (rotvec)."""
    return (p_goal - p_actual,
            Rotation.from_matrix(R_goal @ R_actual.T).as_rotvec())


def target_pose(p0, R0, p1, R1, s):
    """XYZ line and shortest SO(3) arc, both using the same scaling s."""
    p0, p1 = _array('p0', p0, (3,)), _array('p1', p1, (3,))
    R0, R1 = _rotation('R0', R0), _rotation('R1', R1)
    if not np.isfinite(s) or s < 0 or s > 1:
        raise ValueError('s must be finite and inside [0,1]')
    rvec = Rotation.from_matrix(R1 @ R0.T).as_rotvec()
    return p0 + s*(p1-p0), Rotation.from_rotvec(s*rvec).as_matrix() @ R0


def ik_pose(fk_and_jacobian, q_seed, target_xyz, target_R, lower, upper,
            tolerance_m=2e-5, tolerance_rad=1e-4,
            characteristic_length_m=0.2, damping_m=0.003,
            max_step_rad=0.12, max_iterations=140):
    """Solve 6D IK, warm-started and joint-bounded, or raise clearly.

    Position [m] and orientation [rad] are weighted by characteristic length
    [m] so the stacked residual and Jacobian have compatible scales.
    DLS is used with a conservative backtracking check against divergence.
    """
    q = np.asarray(q_seed, dtype=float).copy()
    low, high = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
    pg, Rg = _array('target_xyz', target_xyz, (3,)), _rotation('target_R', target_R)
    if q.ndim != 1 or len(q) < 6 or not np.all(np.isfinite(q)):
        raise ValueError('q_seed must be a finite vector of at least six joints')
    if low.shape != q.shape or high.shape != q.shape or np.any(low > high):
        raise ValueError('Invalid joint limits')
    if np.any(q < low) or np.any(q > high):
        raise ValueError('Initial configuration outside joint limits')
    if not (0 < tolerance_m <= 1e-2 and 0 < tolerance_rad <= 0.1
            and characteristic_length_m > 0 and damping_m > 0
            and max_step_rad > 0 and max_iterations >= 1):
        raise ValueError('Invalid IK solver parameters')
    L = float(characteristic_length_m)
    for iteration in range(max_iterations + 1):
        p, R, J = fk_and_jacobian(q)
        p, R = _array('FK position', p, (3,)), _rotation('FK orientation', R)
        J = _array('FK Jacobian', J, (6, len(q)))
        ep, er = pose_error(pg, Rg, p, R)
        p_res, r_res = float(np.linalg.norm(ep)), float(np.linalg.norm(er))
        if p_res <= tolerance_m and r_res <= tolerance_rad:
            return q, p_res, r_res, iteration
        if iteration == max_iterations:
            break
        e = np.r_[ep, L*er]
        Jw = J.copy()
        Jw[3:] *= L
        dq = Jw.T @ np.linalg.solve(Jw @ Jw.T + damping_m**2*np.eye(6), e)
        max_dq = float(np.max(np.abs(dq)))
        if not np.isfinite(max_dq) or max_dq < 1e-13:
            break
        dq *= min(1.0, max_step_rad / max_dq)
        current = float(np.linalg.norm(e))
        accepted = False
        for factor in (1., .5, .25, .125, .0625, .03125):
            q_next = np.clip(q + factor*dq, low, high)
            if np.linalg.norm(q_next - q) < 1e-13:
                continue
            p_next, R_next, _ = fk_and_jacobian(q_next)
            ep_next, er_next = pose_error(pg, Rg, np.asarray(p_next), np.asarray(R_next))
            residual_next = np.linalg.norm(np.r_[ep_next, L*er_next])
            if residual_next < current - 1e-12:
                q = q_next
                accepted = True
                break
        if not accepted:
            break
    raise RuntimeError(f'6D IK did not converge: position={p_res*1000:.3f} mm, '
                       f'orientation={r_res:.6f} rad')


def plan_cartesian_pose(fk_and_jacobian, q_start, target_xyz, target_R,
                        lower, upper, duration=3., sample_dt=.02,
                        tolerance_m=2e-5, tolerance_rad=1e-4):
    if not np.isfinite(duration) or duration <= 0 or not np.isfinite(sample_dt) or sample_dt <= 0:
        raise ValueError('duration and sample_dt must be positive and finite')
    q0 = np.asarray(q_start, dtype=float).copy()
    start_xyz, start_R, _ = fk_and_jacobian(q0)
    start_xyz, start_R = _array('start_xyz', start_xyz, (3,)), _rotation('start_R', start_R)
    end_xyz, end_R = _array('end_xyz', target_xyz, (3,)), _rotation('end_R', target_R)
    n = max(6, int(np.ceil(duration / sample_dt)))
    t = np.linspace(0., duration, n+1)
    xyz, orientations, path = [], [], [q0]
    worst_p, worst_r, worst_iter = 0., 0., 0
    for k, ti in enumerate(t):
        pg, Rg = target_pose(start_xyz, start_R, end_xyz, end_R,
                             float(quintic_scale(ti, duration)))
        xyz.append(pg)
        orientations.append(Rg)
        if k == 0:
            continue
        try:
            q, pe, re, iters = ik_pose(
                fk_and_jacobian, path[-1], pg, Rg, lower, upper,
                tolerance_m=tolerance_m, tolerance_rad=tolerance_rad)
        except RuntimeError as exc:
            raise RuntimeError(f'6D pose waypoint {k}/{n} at t={ti:.4f} s: {exc}') from exc
        path.append(q)
        worst_p, worst_r = max(worst_p, pe), max(worst_r, re)
        worst_iter = max(worst_iter, iters)
    return CartesianPosePlan(t, np.asarray(xyz), np.asarray(orientations),
                             np.asarray(path), worst_p, worst_r, worst_iter)


def validate_pose_spline(reference, fk_and_jacobian, p0, R0, p1, R1,
                         lower, upper, samples=601,
                         max_position_error_m=.001,
                         max_orientation_error_rad=.01):
    """Check the interpolated joint spline, NOT just its IK waypoints."""
    if samples < 2:
        raise ValueError('samples >= 2 required')
    low, high = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
    maxp = maxr = maxlimit = 0.
    for t in np.linspace(0., reference.duration, samples):
        q, _, _ = reference.evaluate(float(t))
        maxlimit = max(maxlimit, float(np.max(low-q)), float(np.max(q-high)), 0.)
        pg, Rg = target_pose(p0, R0, p1, R1,
                             float(quintic_scale(t, reference.duration)))
        pa, Ra, _ = fk_and_jacobian(q)
        ep, er = pose_error(pg, Rg, np.asarray(pa), np.asarray(Ra))
        maxp = max(maxp, float(np.linalg.norm(ep)))
        maxr = max(maxr, float(np.linalg.norm(er)))
    if maxlimit > 1e-8:
        raise RuntimeError(f'Pose interpolation exceeded joint limits by {maxlimit:.5f} rad')
    if maxp > max_position_error_m:
        raise RuntimeError(f'Pose interpolation position deviation {maxp*1000:.3f} mm exceeds threshold')
    if maxr > max_orientation_error_rad:
        raise RuntimeError(f'Pose interpolation orientation deviation {maxr:.5f} rad exceeds threshold')
    return maxp, maxr
