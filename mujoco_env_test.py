from mujoco_py import MjSim, MjViewer, load_model_from_path
import mujoco_py
import numpy as np
import matplotlib.pyplot as plt
import sys, os

pykin_path = os.path.abspath(os.path.dirname(__file__)+"/pykin/" )
sys.path.append(pykin_path)
print(pykin_path)

from pykin.robots.single_arm import SingleArm
from pykin.kinematics.transform import Transform


def controller(sim, q, q_des,e_prev,e_int,time_step):
    K_p = 0.01
    K_d = 0.01
    K_i = 0.6
    e = np.zeros(7)

    for i in range(np.size(q_des,axis=0)):
        e[i] = q_des[i]-q[i]		
        e_d = (e[i]-e_prev[i])/time_step
        e_i = e_int[i] + e[i]*time_step
        sim.data.ctrl[i] = K_p*e[i] + K_d * e_d + K_i * e_i
    return e


model = load_model_from_path("asset/franka_sim/franka_panda.xml")
sim = MjSim(model)
viewer = MjViewer(sim)

urdf_path = './pykin/asset/urdf/panda/panda.urdf'
robot = SingleArm(urdf_path, offset=Transform(rot=[1, 0, 0, 0], pos=[0, 0, 0]))
robot.setup_link_name("panda_link0", "panda_hand")

print([sim.model.joint_name2id(joint) for joint in robot.get_revolute_joint_names()])

print(sim.get_state())
q_init = np.array([0 , 0, 0, -1.5708, 0, 1.8675, 0])


q_t = np.array([0.0, np.pi/6, 0.0, -np.pi*12/24, 0.0, np.pi*5/8,0.0])
fk = robot.forward_kin(q_t)
eef_pose = robot.get_eef_pose(fk)

is_limit = False
while not is_limit:
    q_desired = robot.inverse_kin(np.random.randn(7), eef_pose)
    is_limit = robot.check_limit_joint(q_desired)

print(q_desired)
# q_desired = [5.60215674e-17, 5.23598764e-01, 0.00000000e+00, -1.57079633e+00, 0.00000000e+00, 1.96349541e+00, 7.48387434e-17]
# q_desired = np.array([-0.46925752, 0.60292218, 0.58800103, -1.56093861, -0.33429735,  1.92466966, 0.1955557 ])
# q_desired = np.zeros(7)



e_prev = np.zeros(7)
e_int = np.zeros(7)

time_step = 0.002
sim_state = sim.get_state()

while True:
    sim_state = sim.get_state()
    q = sim_state.qpos[0:7]
    e_prev = controller(sim,q, q_desired,e_prev,e_int,time_step)
    e_int = e_int + e_prev*time_step

    sim.step()
    viewer.render()


# print(model.njnt)
# n_timesteps = int(10/0.002)

# qpos_d = np.array([0, 0.461, 0, -0.817, 0, 0.69, 0])
# qvel_d = np.zeros(7)

# qlog = np.zeros((3, sim.model.nu))
# print(qlog, qlog.shape)


# print(robot.get_revolute_joint_names())
# # print(sim.data.body_xpos[model.body_name2id('panda_link7')])


# # for i in range(10000):
# #     sim.data.ctrl[:] = np.random.randn(7)
# #     sim.step()
# #     viewer.render()
