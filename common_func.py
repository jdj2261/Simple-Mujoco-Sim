import sys, os

from mujoco_py import MjSim, MjViewer, load_model_from_path
pykin_path = os.path.abspath(os.path.dirname(__file__)+"/pykin")
sys.path.append(pykin_path)

from pykin.robots.single_arm import SingleArm
from pykin.kinematics.transform import Transform
from mj_controller.joint_pos import JointPositionController

def load_mujoco():
    mj_model = load_model_from_path("asset/franka_sim/franka_panda.xml")
    sim = MjSim(mj_model)
    viewer = MjViewer(sim)
    return sim, viewer

def load_pykin():
    panda_urdf_path = 'pykin/asset/urdf/panda/panda.urdf'
    robot = SingleArm(panda_urdf_path, offset=Transform(rot=[1, 0, 0, 0], pos=[0, 0, 0.5]))
    robot.setup_link_name(base_name="panda_link0", eef_name="panda_link7")
    return robot

def get_result_qpos(robot, init_qpos, eef_pos):
    is_limit_qpos = False
    while not is_limit_qpos:
        result_qpos = robot.inverse_kin(init_qpos, eef_pos, method="LM")
        is_limit_qpos = robot.check_limit_joint(result_qpos)
    return result_qpos