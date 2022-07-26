import numpy as np
import os, sys

panda_dir = os.path.dirname(os.getcwd())
parent_path = panda_dir + "/../../"
sys.path.append(parent_path)

from controllers.joint_pos import JointPositionController
from demos.common import load_mujoco, load_pykin, get_result_qpos

def main():
    sim, viewer = load_mujoco(parent_path + "asset/panda_sim/franka_panda.xml")
    panda_robot = load_pykin(parent_path + 'pykin/asset/urdf/panda/panda.urdf')
    panda_robot.setup_link_name("panda_link_0", "right_hand")

    init_qpos = [0, 0.1963495375, 0.00, -2.616, 0.00, 2.9415926, 0.78539815]
    desired_qpos = np.array([0, 0, 0, -1.5708, 0, 1.8675, 0])
    transformations = panda_robot.forward_kin(desired_qpos)
    eef_pose = transformations[panda_robot.eef_name].pose
    result_qpos = get_result_qpos(panda_robot, init_qpos, eef_pose)

    jpos_controller = JointPositionController(sim=sim, eef_name=panda_robot.eef_name)
    jpos_controller.kp = 20
    while True:
        torque = jpos_controller.run_controller(sim, result_qpos)
        sim.data.ctrl[jpos_controller.qpos_index] = torque

        print(f"Current : {jpos_controller.eef_pos}")
        print(f"Robot : {panda_robot.forward_kin(jpos_controller.q_pos)[panda_robot.eef_name].pos}")
        print()
        sim.step()
        viewer.render()

if __name__ == "__main__":
    main()

    