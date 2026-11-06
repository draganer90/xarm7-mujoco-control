"""Optional, independently computed rigid-body model and model-agreement tests."""
from pathlib import Path
import numpy as np
import mujoco


class PinModel:
    def __init__(self, project_root, mjmodel, atol=1e-4, rtol=1e-3):
        try:
            import pinocchio as pin
        except ImportError as e:
            raise RuntimeError('Install Pinocchio from conda-forge: conda install -c conda-forge pinocchio') from e
        self.pin = pin
        path = Path(project_root)/'models'/'xarm7_mujoco.urdf'
        if not path.is_file():
            raise FileNotFoundError('Missing models/xarm7_mujoco.urdf (required for independent Pinocchio model)')
        self.model = pin.buildModelFromUrdf(str(path))
        self.data = self.model.createData()
        if self.model.nq != 7 or self.model.nv != 7:
            raise ValueError('Pinocchio model is not a 7-DOF arm')
        self.atol, self.rtol = atol, rtol
        self.check(mjmodel)

    def mass_bias(self, q, v):
        pin = self.pin
        M = pin.crba(self.model, self.data, q)
        M = np.triu(M) + np.triu(M, 1).T
        bias = pin.nonLinearEffects(self.model, self.data, q, v)
        return M.copy(), np.asarray(bias).copy()

    def check(self, mjmodel):
        """Fail closed rather than silently combining inconsistent robot models."""
        d = mujoco.MjData(mjmodel)
        candidates = [np.zeros(7), np.array([.1, -.3, .2, .7, -.1, .4, .05])]
        v = np.array([.2, -.1, .07, -.15, .2, -.08, .1])
        for q in candidates:
            d.qpos[:], d.qvel[:] = q, v
            mujoco.mj_forward(mjmodel, d)
            MM = np.empty((7,7))
            mujoco.mj_fullM(mjmodel, d, MM)
            MP, bP = self.mass_bias(q, v)
            if not np.allclose(MM, MP, atol=self.atol, rtol=self.rtol) or not np.allclose(d.qfrc_bias, bP, atol=self.atol, rtol=self.rtol):
                raise RuntimeError(
                    'Pinocchio URDF and MuJoCo MJCF disagree in mass/bias. '
                    f'M max_abs={np.max(np.abs(MM-MP)):.5g}, '
                    f'bias max_abs={np.max(np.abs(d.qfrc_bias-bP)):.5g}. '
                    'Do not use Pinocchio computed torque until the models match.'
                )
        print('PASS: Pinocchio vs MuJoCo mass and bias agree at 2 reference states.')
