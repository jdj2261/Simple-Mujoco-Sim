import sys, os
import numpy as np
from mujoco_py import MjSim, MjViewer, load_model_from_path
pykin_path = os.path.abspath(os.path.dirname(__file__) + "/../" + "pykin")
sys.path.append(pykin_path)

from pykin.robots.single_arm import SingleArm
from pykin.kinematics.transform import Transform

def load_mujoco(path):
    mj_model = load_model_from_path(path)
    sim = MjSim(mj_model)
    viewer = MjViewer(sim)
    return sim, viewer

def load_pykin(path):
    robot = SingleArm(path, offset=Transform(rot=[1, 0, 0, 0], pos=[0, 0, 0.92]))
    return robot

def get_result_qpos(robot, init_qpos, eef_pos):
    is_limit_qpos = False
    result_qpos = robot.inverse_kin(init_qpos, eef_pos, method="LM")
    is_limit_qpos = robot.check_limit_joint(result_qpos)
    if is_limit_qpos:
        return result_qpos

    while not is_limit_qpos:
        result_qpos = robot.inverse_kin(np.random.randn(len(init_qpos)), eef_pos, method="LM")
        is_limit_qpos = robot.check_limit_joint(result_qpos)
    return result_qpos