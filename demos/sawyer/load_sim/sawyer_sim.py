import numpy as np
import os, sys
sawyer_dir = os.path.dirname(os.getcwd())
parent_path = sawyer_dir + "/../../"
sys.path.append(parent_path)

from mj_controller.joint_pos import JointPositionController
from demos.common import load_mujoco, load_pykin, get_result_qpos

def main():
    sim, viewer = load_mujoco(parent_path + "asset/sawyer_sim/sawyer.xml")
    sawyer_robot = load_pykin(parent_path + 'pykin/asset/urdf/sawyer/sawyer.urdf')
    sawyer_robot.setup_link_name("sawyer_base", "sawyer_right_l6")

    init_qpos = np.array([0, 0, 0, -1.5708, 0, 1.8675, 0])
    desired_qpos = [0, 0, -1.18, 0.00, 2.18, 0.00, 0.57,0]
    transformations = sawyer_robot.forward_kin(desired_qpos)
    eef_pose = transformations[sawyer_robot.eef_name].pose
    result_qpos = get_result_qpos(sawyer_robot, init_qpos, eef_pose)

    jpos_controller = JointPositionController(sim=sim, eef_name=sawyer_robot.eef_name)
    jpos_controller.kp = 50

    while True:
        torque = jpos_controller.run_controller(sim, result_qpos)
        sim.data.ctrl[jpos_controller.qpos_index] = torque

        # print(f"Current : {np.round(jpos_controller.eef_pos, 6)}")
        # cur_qpos = np.concatenate((np.zeros(1), np.array(jpos_controller.q_pos)))
        # print(f"Robot : {np.round(sawyer_robot.forward_kin(cur_qpos)[sawyer_robot.eef_name].pos,6)}")
        # print()

        sim.step()
        viewer.render()

if __name__ == "__main__":
    main()