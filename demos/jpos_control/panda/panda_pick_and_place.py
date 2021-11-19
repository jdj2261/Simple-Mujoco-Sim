import numpy as np
import time
import sys, os

controller_path = os.path.abspath(os.path.abspath(__file__) + "/../../../../")
sys.path.append(controller_path)
demo_path = os.path.abspath(os.path.abspath(__file__) + "/../../../")
sys.path.append(demo_path)

from mj_controller.joint_pos import JointPositionController
from common import load_mujoco, load_pykin, get_result_qpos


def main():
    sim, viewer = load_mujoco("../../../asset/panda_sim/franka_panda.xml")
    panda_robot = load_pykin('../../../pykin/asset/urdf/panda/panda.urdf')
    panda_robot.setup_link_name("panda_link0", "panda_link7")

    init_qpos = np.array([0 , 0, 0, -1.5708, 0, 1.8675, 0])
    desired_qpos = np.array([0, np.pi/6, 0.0, -np.pi/2, 0.0, np.pi*5/8,0.0])
    transformations = panda_robot.forward_kin(desired_qpos)
    eef_pose = transformations[panda_robot.eef_name].pose

    result_qpos = get_result_qpos(panda_robot, init_qpos, eef_pose)

    jpos_controller = JointPositionController(sim=sim, eef_name=panda_robot.eef_name, kp=30)
    is_reached = False

    while True:
        
        if not is_reached:
            torque = jpos_controller.run_controller(sim, result_qpos)
            sim.data.ctrl[jpos_controller.qpos_index] = torque
            sim.data.ctrl[jpos_controller.gripper_index] = [0.4, -0.4]
            
            if jpos_controller.is_reached():
                is_reached = True
                print("is_reached")
        
        if is_reached:
            torque = jpos_controller.run_controller(sim, [np.pi/2, np.pi/6, 0.0, -np.pi/2, 0.0, np.pi*5/8,0.0])
            sim.data.ctrl[jpos_controller.qpos_index] = torque
            sim.data.ctrl[jpos_controller.gripper_index] = [0.0, 0.0]

            if jpos_controller.is_reached():
                is_reached = False
                print("is_reached")
        print(np.round(jpos_controller.err_qpos, 4))

        sim.step()
        viewer.render()

if __name__ == "__main__":
    main()