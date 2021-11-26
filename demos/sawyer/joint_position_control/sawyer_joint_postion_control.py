import numpy as np
import time
import os, sys
sawyer_dir = os.path.dirname(os.getcwd())
parent_path = sawyer_dir + "/../../"
sys.path.append(parent_path)

from mj_controller.joint_pos import JointPositionController
from demos.common import load_mujoco, load_pykin, get_result_qpos

def main():
    sim, viewer = load_mujoco(parent_path + "asset/sawyer_sim/sawyer.xml")
    sawyer_robot = load_pykin(parent_path + 'pykin/asset/urdf/sawyer/sawyer.urdf')
    sawyer_robot.setup_link_name("sawyer_base", "sawyer_right_hand")

    print(sawyer_robot.active_joint_names)

    init_qpos = np.array([0, 0, 0, -1.5708, 0, 1.8675, 0])
    desired_qpos = np.array([0, 0, np.pi/6, 0.0, -np.pi/2, 0.0, np.pi*5/8, 0.0])
    transformations = sawyer_robot.forward_kin(desired_qpos)
    eef_pose = transformations[sawyer_robot.eef_name].pose

    result_qpos = get_result_qpos(sawyer_robot, init_qpos, eef_pose)

    jpos_controller = JointPositionController(sim=sim, eef_name=sawyer_robot.eef_name)
    jpos_controller.kp = jpos_controller.nums2array(70, 7)
    cnt = 0
    while True:
        torque = jpos_controller.run_controller(sim, result_qpos)
        sim.data.ctrl[jpos_controller.qpos_index] = torque

        # print(sim.data.geom_xpos[sim.model.geom_name2id("coke")])
        print(f"Current : {jpos_controller.eef_pos}")
        cur_qpos = np.concatenate((np.zeros(1), np.array(jpos_controller.q_pos)))
        print(f"Robot : {sawyer_robot.forward_kin(cur_qpos)[sawyer_robot.eef_name].pos}")

        if jpos_controller.is_reached():
            cnt += 1
            print(f"End Effector Position : {np.round(jpos_controller.eef_pos,4)}")
            print("Goal reached")
            sim.data.ctrl[jpos_controller.gripper_index] = [0, 0]
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