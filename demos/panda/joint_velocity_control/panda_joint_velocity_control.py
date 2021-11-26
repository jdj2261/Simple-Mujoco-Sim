import numpy as np
import time
import os, sys

panda_dir = os.path.dirname(os.getcwd())
parent_path = panda_dir + "/../../"
sys.path.append(parent_path)

from mj_controller.joint_vel import JointVelocityController
from mj_controller.joint_pos import JointPositionController
from demos.common import load_mujoco, load_pykin, get_result_qpos
from pykin.planners.cartesian_planner import CartesianPlanner

def main():
    sim, viewer = load_mujoco(parent_path + "asset/panda_sim/franka_panda.xml")
    panda_robot = load_pykin(parent_path + 'pykin/asset/urdf/panda/panda.urdf')
    panda_robot.setup_link_name("panda_link0", "panda_right_hand")

    # init_qpos = np.array([0, 0.1963495375, 0.00, -2.616, 0.00, 2.9415926, 0.78539815])
    # eef_pose = [0.65, 0.3464,  1.17,  -0.025,  0.778,  0.597, -0.193,]
    # result_qpos = get_result_qpos(panda_robot, init_qpos, eef_pose)
    # target_eef_pose = [0.65, 0.3464,  1.47,  -0.025,  0.778,  0.597, -0.193]

    jvel_controller = JointVelocityController(sim=sim, eef_name=panda_robot.eef_name)
    
    while True:
        torque = jvel_controller.run_controller(sim, [0.1, 0, 0, 0, 0, 0, 0])
        sim.data.ctrl[jvel_controller.qpos_index] = torque
        sim.step()
        viewer.render()

if __name__ == "__main__":
    main()