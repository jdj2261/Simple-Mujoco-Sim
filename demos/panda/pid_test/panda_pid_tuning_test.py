import numpy as np
import time
import sys, os
import matplotlib.pyplot as plt

controller_path = os.path.abspath(os.path.abspath(__file__) + "/../../../../")
sys.path.append(controller_path)
demo_path = os.path.abspath(os.path.abspath(__file__) + "/../../../")
sys.path.append(demo_path)

from mj_controller.joint_pos import JointPositionController
from common import load_mujoco, load_pykin, get_result_qpos
from mj_controller.plot import plot_joints

def main():
    t = 0
    n_timesteps = int(5/0.002)
    qlog = np.zeros((n_timesteps, 7))

    sim, viewer = load_mujoco("../../../asset/panda_sim/franka_panda.xml")
    panda_robot = load_pykin('../../../pykin/asset/urdf/panda/panda.urdf')
    panda_robot.setup_link_name("panda_link0", "panda_link7")

    init_qpos = np.array([0 , 0, 0, -1.5708, 0, 1.8675, 0])
    desired_qpos = np.array([0, np.pi/6, 0.0, -np.pi/2, 0.0, np.pi*5/8,0.0])
    transformations = panda_robot.forward_kin(desired_qpos)
    eef_pose = transformations[panda_robot.eef_name].pose

    result_qpos = get_result_qpos(panda_robot, init_qpos, eef_pose)

    jpos_controller = JointPositionController(sim=sim, eef_name=panda_robot.eef_name)
    jpos_controller.kp = np.array([50, 50, 50, 50, 50, 50, 50])
    jpos_controller.ki = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    jpos_controller.kd = np.array([22.2, 22.2, 22.2, 22.2, 22.2, 22.2, 22.2])

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