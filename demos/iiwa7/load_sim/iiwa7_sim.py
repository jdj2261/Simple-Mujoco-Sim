import numpy as np
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
    desired_qpos = np.array([0.000, 0.650, 0.000, -1.690, 0.000, 0.300, 0.000])
    transformations = iiwa7_robot.forward_kin(desired_qpos)
    eef_pose = transformations[iiwa7_robot.eef_name].pose
    result_qpos = get_result_qpos(iiwa7_robot, init_qpos, eef_pose)

    jpos_controller = JointPositionController(sim=sim, eef_name=iiwa7_robot.eef_name)
    jpos_controller.kp = 20

    while True:
        torque = jpos_controller.run_controller(sim, result_qpos)
        sim.data.ctrl[jpos_controller.qpos_index] = torque

        print(f"Current : {np.round(jpos_controller.eef_pos, 7)}")
        print(f"Robot : {np.round(iiwa7_robot.forward_kin(jpos_controller.q_pos)[iiwa7_robot.eef_name].pos,7)}")
        print()
        
        sim.step()
        viewer.render()

if __name__ == "__main__":
    main()