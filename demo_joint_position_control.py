from mujoco_py import MjSim, MjViewer, load_model_from_path
import numpy as np
import sys, os

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

def main():
    sim, viewer = load_mujoco()
    panda_robot = load_pykin()

    init_qpos = np.array([0 , 0, 0, -1.5708, 0, 1.8675, 0])
    desired_qpos = np.array([0, np.pi/6, 0.0, -np.pi/2, 0.0, np.pi*5/8,0.0])
    transformations = panda_robot.forward_kin(desired_qpos)
    eef_pose = transformations[panda_robot.eef_name].pose

    result_qpos = get_result_qpos(panda_robot, init_qpos, eef_pose)

    jpos_controller = JointPositionController(sim=sim, eef_name=panda_robot.eef_name)
    is_reached = False
    while True:
        
        if not is_reached:
            torque = jpos_controller.run_controller(sim, result_qpos)
            sim.data.ctrl[jpos_controller.qpos_index] = torque
            sim.data.ctrl[jpos_controller.gripper_index] = [0.4, 0.4]
            
            if jpos_controller.is_reached():
                is_reached = True
                print("is_reached")

            print(np.round(jpos_controller.err_qpos,4), np.round(sim.data.time,4))
        
        if is_reached:
            torque = jpos_controller.run_controller(sim, [0, 0, 0, -1.5708, 0, 1.8675, 0])
            sim.data.ctrl[jpos_controller.qpos_index] = torque
            sim.data.ctrl[jpos_controller.gripper_index] = [0.0, 0.0]

            print(jpos_controller.is_reached())
            print(jpos_controller.err_qpos)

            if jpos_controller.is_reached():
                is_reached = False
                print("is_reached")

        sim.step()
        viewer.render()

if __name__ == "__main__":
    main()