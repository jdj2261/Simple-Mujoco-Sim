import numpy as np
import time
import os, sys
iiwa7_dir = os.path.dirname(os.getcwd())
parent_path = iiwa7_dir + "/../../"
sys.path.append(parent_path)

from controllers.joint_pos import JointPositionController
from demos.common import load_mujoco, load_pykin, get_result_qpos

def main():
    sim, viewer = load_mujoco(parent_path + "asset/iiwa7_sim/iiwa7.xml")
    iiwa7_robot = load_pykin(parent_path + 'pykin/asset/urdf/iiwa7/iiwa7.urdf')
    iiwa7_robot.setup_link_name("iiwa7_link_0", "iiwa7_right_hand")

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
        
        print(f"Current : {np.round(jpos_controller.eef_pos, 7)}")
        print(f"Robot : {np.round(iiwa7_robot.forward_kin(jpos_controller.q_pos)[iiwa7_robot.eef_name].pos,7)}")
        print()
        
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