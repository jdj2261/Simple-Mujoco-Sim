import numpy as np
import os, sys
sawyer_dir = os.path.dirname(os.getcwd())
parent_path = sawyer_dir + "/../../"
sys.path.append(parent_path)

from mj_controller.joint_pos import JointPositionController
from mj_controller.plot import plot_joints
from demos.common import load_mujoco, load_pykin, get_result_qpos


def main():
    t = 0
    n_timesteps = int(10/0.002)
    qlog = np.zeros((n_timesteps, 7))

    sim, viewer = load_mujoco(parent_path + "asset/sawyer_sim/sawyer.xml")
    sawyer_robot = load_pykin(parent_path + 'pykin/asset/urdf/sawyer/sawyer.urdf')
    sawyer_robot.setup_link_name("sawyer_base", "sawyer_right_hand")

    init_qpos = np.array([0, 0, 0, -1.5708, 0, 1.8675, 0])
    desired_qpos = np.array([0, 0, np.pi/6, 0.0, -np.pi/2, 0.0, np.pi*5/8,0.0])
    transformations = sawyer_robot.forward_kin(desired_qpos)
    eef_pose = transformations[sawyer_robot.eef_name].pose

    result_qpos = get_result_qpos(sawyer_robot, init_qpos, eef_pose)

    jpos_controller = JointPositionController(sim=sim, eef_name=sawyer_robot.eef_name)
    jpos_controller.kp = jpos_controller.nums2array(50, 7)
    jpos_controller.ki = jpos_controller.nums2array(0.1, 7)
    jpos_controller.kd = jpos_controller.nums2array(10, 7)

    while t < n_timesteps:
        torque = jpos_controller.run_controller(sim, result_qpos)
        sim.data.ctrl[jpos_controller.qpos_index] = torque
        qlog[t] = sim.data.qpos[jpos_controller.qpos_index]
        print(np.round(sim.data.qpos[jpos_controller.qpos_index], 4))

        t += 1
        sim.step()
        # viewer.render()

    plot_joints(desired_qpos, qlog, jpos_controller, True)

if __name__ == "__main__":
    main()