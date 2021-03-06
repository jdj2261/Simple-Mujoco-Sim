import numpy as np
import time

import os, sys
iiwa14_dir = os.path.dirname(os.getcwd())
parent_path = iiwa14_dir + "/../../"
sys.path.append(parent_path)

from controllers.joint_pos import JointPositionController
from demos.common import load_mujoco, load_pykin, get_result_qpos

def main():
    sim, viewer = load_mujoco(parent_path + "asset/iiwa14_sim/iiwa14.xml")
    iiwa14_robot = load_pykin(parent_path + 'pykin/asset/urdf/iiwa14/iiwa14.urdf')
    iiwa14_robot.setup_link_name("iiwa14_link_0", "iiwa14_right_hand")

    init_qpos = np.array([0, 0, 0, -1.5708, 0, 1.8675, 0])
    desired_qpos = np.array([0, np.pi/2, 0.0, -0, 0.0, 0,0.0])
    transformations = iiwa14_robot.forward_kin(desired_qpos)
    eef_pose = transformations[iiwa14_robot.eef_name].pose

    result_qpos = get_result_qpos(iiwa14_robot, init_qpos, eef_pose)

    jpos_controller = JointPositionController(sim=sim, eef_name=iiwa14_robot.eef_name)
    jpos_controller.kp = jpos_controller.nums2array(20, 7)
    cnt = 0

    while True:
        torque = jpos_controller.run_controller(sim, result_qpos)
        sim.data.ctrl[jpos_controller.qpos_index] = torque
    
        print(f"Current : {jpos_controller.eef_pos}")
        print(f"Robot : {iiwa14_robot.forward_kin(jpos_controller.q_pos)[iiwa14_robot.eef_name].pos}")

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