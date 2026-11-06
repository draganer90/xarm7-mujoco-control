import unittest
import numpy as np
from xarm_control.trajectories import cubic, quintic


class TestTrajectories(unittest.TestCase):
    def test_quintic_endpoints(self):
        a, b, T = np.zeros(3), np.array([1., 2., -1.]), 3.
        q0,v0,a0=quintic(a,b,0,T)
        qf,vf,af=quintic(a,b,T,T)
        np.testing.assert_allclose(q0,a)
        np.testing.assert_allclose(qf,b)
        np.testing.assert_allclose(v0,0,atol=1e-12)
        np.testing.assert_allclose(vf,0,atol=1e-12)
        np.testing.assert_allclose(a0,0,atol=1e-12)
        np.testing.assert_allclose(af,0,atol=1e-12)

    def test_cubic_positions_velocities(self):
        q0,v0,_ = cubic([0], [1], 0, 2)
        qf,vf,_ = cubic([0], [1], 2, 2)
        self.assertAlmostEqual(q0[0],0)
        self.assertAlmostEqual(qf[0],1)
        self.assertAlmostEqual(v0[0],0)
        self.assertAlmostEqual(vf[0],0)

    def test_invalid_T(self):
        with self.assertRaises(ValueError): quintic([0],[1],0,0)


if __name__ == '__main__':
    unittest.main()
