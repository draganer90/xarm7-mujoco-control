"""Model validation, gravity/dynamics and task-space kinematics."""
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation

EXPECTED_JOINTS = tuple(f'joint{i}' for i in range(1, 8))
EXPECTED_MOTORS = tuple(f'motor_joint{i}' for i in range(1, 8))
DEFAULT_GOAL = np.array([0.0, -0.6, 0.0, 1.0, 0.0, 0.6, 0.0])


def load_model(project_root):
    path = Path(project_root)/'models'/'xarm7_actuated.xml'
    if not path.is_file():
        raise FileNotFoundError(f'Model not found: {path}. Keep your existing models/ folder.')
    model = mujoco.MjModel.from_xml_path(str(path))
    if (model.nq, model.nv, model.nu, model.njnt) != (7, 7, 7, 7):
        raise ValueError('Expected fixed-base 7-DOF/7-actuator xArm7 (nq=nv=nu=njnt=7).')
    joints = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(7)]
    motors = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(7)]
    if tuple(joints) != EXPECTED_JOINTS or tuple(motors) != EXPECTED_MOTORS:
        raise ValueError(f'Unexpected joint/motor ordering: {joints} / {motors}')
    if not np.allclose(model.actuator_gear[:, 0], 1.0):
        raise ValueError('Expected direct torque motor gear=1 for all actuators.')
    for i in range(7):
        if model.actuator_trnid[i, 0] != i:
            raise ValueError('Motor transmission order differs from joint order.')
    return model


def gravity(model, grav_data, q):
    """MuJoCo bias at qvel=0; equals gravity term for this fixed-base arm."""
    grav_data.qpos[:] = q
    grav_data.qvel[:] = 0
    mujoco.mj_forward(model, grav_data)
    return grav_data.qfrc_bias.copy()


def mass_and_bias(model, data):
    mass = np.empty((model.nv, model.nv))
    mujoco.mj_fullM(model, data, mass)
    return mass, data.qfrc_bias.copy()


def get_body_id(model, name='link7'):
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid < 0:
        raise ValueError(f'Missing end-effector body {name!r}; set --ee-body after inspection.')
    return bid


def body_pose_and_jacobian(model, data, bid, full_pose=True):
    """Frame origin (NOT COM), world frame, geometric 6x7 Jacobian."""
    p = data.xpos[bid].copy()
    R = data.xmat[bid].reshape(3, 3).copy()
    jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jp, jr, p, bid)
    return p, R, np.vstack((jp, jr)) if full_pose else jp


def orientation_error_world(target_R, actual_R):
    """Rotation-vector of target_R * actual_R^T, expressed in world frame."""
    return Rotation.from_matrix(target_R @ actual_R.T).as_rotvec()


def jacobian_dot_times_qvel(model, data, bid, full_pose=True, epsilon=1e-6):
    """Numerically estimate Jdot*qdot along the current qdot (7 scalar hinges)."""
    v = data.qvel.copy()
    if not np.any(v):
        return np.zeros(6 if full_pose else 3)
    _, _, j0 = body_pose_and_jacobian(model, data, bid, full_pose)
    d1 = mujoco.MjData(model)
    d1.qpos[:] = data.qpos + epsilon*v
    d1.qvel[:] = v
    mujoco.mj_forward(model, d1)
    _, _, j1 = body_pose_and_jacobian(model, d1, bid, full_pose)
    return ((j1 - j0)/epsilon) @ v


def actuator_limits(model):
    if not np.all(model.actuator_ctrllimited):
        raise ValueError('All 7 actuators must have ctrlrange for safe benchmark clipping.')
    return model.actuator_ctrlrange.copy()
