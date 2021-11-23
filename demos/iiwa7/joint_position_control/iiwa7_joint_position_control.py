import numpy as np
import sys, os
import time

controller_path = os.path.abspath(os.path.abspath(__file__) + "/../../../../")
sys.path.append(controller_path)
demo_path = os.path.abspath(os.path.abspath(__file__) + "/../../../")
sys.path.append(demo_path)

from mj_controller.joint_pos import JointPositionController
from common import load_mujoco, load_pykin, get_result_qpos

def main():
    sim, viewer = load_mujoco("../../../asset/iiwa7_sim/iiwa7.xml")
    iiwa7_robot = load_pykin("../../../pykin/asset/urdf/iiwa7/iiwa7.urdf")
    iiwa7_robot.setup_link_name("iiwa7_link_0", "iiwa7_link_7")

    init_qpos = np.array([0, 0, 0, -1.5708, 0, 1.8675, 0])
    desired_qpos = np.array([0, np.pi/2, 0.0, 0, 0.0, 0,0.0])
    transformations = iiwa7_robot.forward_kin(desired_qpos)
    eef_pose = transformations[iiwa7_robot.eef_name].pose

    result_qpos = get_result_qpos(iiwa7_robot, init_qpos, eef_pose)

    jpos_controller = JointPositionController(sim=sim, eef_name=iiwa7_robot.eef_name)
    jpos_controller.kp = jpos_controller.nums2array(20, 7)
    cnt = 0

    while True:
        torque = jpos_controller.run_controller(sim, result_qpos)
        sim.data.ctrl[jpos_controller.qpos_index] = torque
        sim.data.ctrl[jpos_controller.gripper_index] = [0.5, -0.5]

        print(f"Current : {jpos_controller.eef_pos}")
        print(f"Robot : {iiwa7_robot.forward_kin(jpos_controller.q_pos)[iiwa7_robot.eef_name].pos}")

        if jpos_controller.is_reached():
            cnt += 1
            print(f"End Effector Position : {np.round(jpos_controller.eef_pos,4)}")
            print("Goal reached")
            time.sleep(1)
    
            if cnt%4 == 1:
                result_qpos[0] += np.pi/2
            elif cnt%4 == 2:
                result_qpos[1] -= np.pi/2
            elif cnt%4 == 3:
                result_qpos[0] -= np.pi/2
            else:
                result_qpos[1] += np.pi/2

        sim.step()
        viewer.render()

if __name__ == "__main__":
    main()