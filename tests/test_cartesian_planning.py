"""Unit tests for Cartesian line planning using a 7-DOF synthetic FK map."""
import unittest
import numpy as np
from xarm_control.cartesian_planning import (quintic_scale, line_samples,
    ik_position, plan_cartesian_line, JointSpline, validate_spline)


def linear_fk(q):
    # Fake robot for algorithm tests, NOT evidence for the xArm7 MJCF.
    J = np.zeros((3, 7))
    J[0, 0] = J[1, 1] = J[2, 2] = 1.
    return J @ q, J


class CartesianPlanningTests(unittest.TestCase):
    def test_quintic_and_line(self):
        t, p = line_samples([0, 0, 0], [.02, -.01, .03], 3., .02)
        np.testing.assert_allclose(p[0], 0)
        np.testing.assert_allclose(p[-1], [.02, -.01, .03])
        np.testing.assert_allclose(p[len(p)//2], [.01, -.005, .015], atol=1e-12)
        self.assertAlmostEqual(float(quintic_scale(3., 3.)), 1.)

    def test_ik_and_spline(self):
        q0 = np.zeros(7)
        lo, hi = np.full(7, -2.), np.full(7, 2.)
        plan = plan_cartesian_line(linear_fk, q0, [.02, -.01, .03], lo, hi,
                                   sample_dt=.02, duration=3.)
        self.assertLess(plan.ik_max_residual_m, 2e-5)
        ref = JointSpline(plan)
        for t in (0, 3, 4):
            _, v, a = ref.evaluate(t)
            np.testing.assert_allclose(v, 0, atol=1e-12)
            np.testing.assert_allclose(a, 0, atol=1e-12)
        self.assertLess(validate_spline(ref, linear_fk, [0,0,0], [.02,-.01,.03], lo, hi), .001)

    def test_unreachable(self):
        q0 = np.zeros(7)
        lo, hi = np.full(7, -.01), np.full(7, .01)
        with self.assertRaises(RuntimeError):
            ik_position(linear_fk, q0, [1, 0, 0], lo, hi)

    def test_invalid_arguments(self):
        with self.assertRaises(ValueError):
            line_samples([0,0,0], [1,0], 3)
        with self.assertRaises(ValueError):
            line_samples([0,0,0], [.01,0,0], 0)


if __name__ == '__main__':
    unittest.main()
