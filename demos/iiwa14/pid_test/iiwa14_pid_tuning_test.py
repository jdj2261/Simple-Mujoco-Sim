import numpy as np
import os, sys
iiwa14_dir = os.path.dirname(os.getcwd())
parent_path = iiwa14_dir + "/../../"
sys.path.append(parent_path)

from mj_controller.joint_pos import JointPositionController
from mj_controller.plot import plot_joints
from demos.common import load_mujoco, load_pykin, get_result_qpos

def main():
    t = 0
    n_timesteps = int(10/0.002)
    qlog = np.zeros((n_timesteps, 7))

    sim, viewer = load_mujoco("../../../asset/iiwa14_sim/iiwa14.xml")
    iiwa14_robot = load_pykin("../../../pykin/asset/urdf/iiwa14/iiwa14.urdf")
    iiwa14_robot.setup_link_name("iiwa14_link_0", "iiwa14_link_7")

    init_qpos = np.array([0, 0, 0, -1.5708, 0, 1.8675, 0])
    desired_qpos = np.array([0, np.pi/6, 0.0, -0, 0.0, 0,0.0])
    transformations = iiwa14_robot.forward_kin(desired_qpos)
    eef_pose = transformations[iiwa14_robot.eef_name].pose

    result_qpos = get_result_qpos(iiwa14_robot, init_qpos, eef_pose)

    jpos_controller = JointPositionController(sim=sim, eef_name=iiwa14_robot.eef_name)
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