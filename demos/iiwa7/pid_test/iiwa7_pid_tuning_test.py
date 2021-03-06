import numpy as np
import os, sys
iiwa7_dir = os.path.dirname(os.getcwd())
parent_path = iiwa7_dir + "/../../"
sys.path.append(parent_path)

from controllers.joint_pos import JointPositionController
from controllers.plot import plot_joints
from demos.common import load_mujoco, load_pykin, get_result_qpos

def main():
    t = 0
    n_timesteps = int(10/0.002)
    qlog = np.zeros((n_timesteps, 7))

    sim, viewer = load_mujoco("../../../asset/iiwa7_sim/iiwa7.xml")
    iiwa7_robot = load_pykin("../../../pykin/asset/urdf/iiwa7/iiwa7.urdf")
    iiwa7_robot.setup_link_name("iiwa7_link_0", "iiwa7_right_hand")

    init_qpos = np.array([0, 0, 0, -1.5708, 0, 1.8675, 0])
    desired_qpos = np.array([0, np.pi/6, 0.0, -0, 0.0, 0,0.0])
    transformations = iiwa7_robot.forward_kin(desired_qpos)
    eef_pose = transformations[iiwa7_robot.eef_name].pose

    result_qpos = get_result_qpos(iiwa7_robot, init_qpos, eef_pose)

    jpos_controller = JointPositionController(sim=sim, eef_name=iiwa7_robot.eef_name)
    jpos_controller.kp = jpos_controller.nums2array(20, 7)
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