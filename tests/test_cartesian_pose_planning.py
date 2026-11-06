"""Pure numerical tests, using synthetic FK; not full-robot MuJoCo validation."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from xarm_control.cartesian_planning import JointSpline
from xarm_control.cartesian_pose_planning import (
    pose_error, target_pose, ik_pose, plan_cartesian_pose, validate_pose_spline)


def synthetic_fk(q):
    """Six independent synthetic DOFs; rotation restricted to world Z."""
    q = np.asarray(q)
    J = np.zeros((6, 7))
    J[0, 0] = J[1, 1] = J[2, 2] = 1.0
    J[5, 5] = 1.0
    return q[:3].copy(), Rotation.from_euler('z', q[5]).as_matrix(), J


class CartesianPoseTests(unittest.TestCase):
    def setUp(self):
        self.q0 = np.zeros(7)
        self.lo, self.hi = np.full(7, -2.), np.full(7, 2.)

    def test_interpolation_endpoints(self):
        R0 = np.eye(3)
        R1 = Rotation.from_euler('z', 10., degrees=True).as_matrix()
        p0, R = target_pose([0,0,0], R0, [.02,0,0], R1, 0)
        np.testing.assert_allclose(p0, [0,0,0], atol=1e-12)
        np.testing.assert_allclose(R, R0, atol=1e-12)
        p1, R = target_pose([0,0,0], R0, [.02,0,0], R1, 1)
        np.testing.assert_allclose(p1, [.02,0,0], atol=1e-12)
        np.testing.assert_allclose(R, R1, atol=1e-12)
        p_mid, R_mid = target_pose([0,0,0], R0, [.02,0,0], R1, .5)
        np.testing.assert_allclose(p_mid, [.01,0,0], atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.norm(Rotation.from_matrix(R_mid).as_rotvec())),
                               np.deg2rad(5.), places=9)

    def test_pose_ik_and_dense_path(self):
        R1 = Rotation.from_euler('z', 5., degrees=True).as_matrix()
        plan = plan_cartesian_pose(synthetic_fk, self.q0, [.02,-.01,.015], R1,
                                   self.lo, self.hi, duration=3., sample_dt=.02)
        self.assertLess(plan.position_residual_max_m, 2e-5)
        self.assertLess(plan.orientation_residual_max_rad, 1e-4)
        spline = JointSpline(plan)
        maxp, maxr = validate_pose_spline(spline, synthetic_fk, np.zeros(3),
                                           np.eye(3), [.02,-.01,.015], R1,
                                           self.lo, self.hi)
        self.assertLess(maxp, .001)
        self.assertLess(maxr, .01)
        for t in [0., 3., 5.]:
            _, v, a = spline.evaluate(t)
            np.testing.assert_allclose(v, 0, atol=1e-12)
            np.testing.assert_allclose(a, 0, atol=1e-12)

    def test_unreachable_orientation(self):
        R1 = Rotation.from_euler('x', 5., degrees=True).as_matrix()
        with self.assertRaises(RuntimeError):
            ik_pose(synthetic_fk, self.q0, np.zeros(3), R1,
                    self.lo, self.hi, max_iterations=15)

    def test_invalid_rotation(self):
        with self.assertRaises(ValueError):
            target_pose(np.zeros(3), np.eye(3), np.zeros(3), np.zeros((3,3)), 0.5)

    def test_error_sign(self):
        target = Rotation.from_euler('z', 10., degrees=True).as_matrix()
        ep, er = pose_error(np.array([.01,0,0]), target, np.zeros(3), np.eye(3))
        np.testing.assert_allclose(ep, [.01,0,0])
        np.testing.assert_allclose(er, [0,0,np.deg2rad(10.)], atol=1e-12)


if __name__ == '__main__':
    unittest.main()
