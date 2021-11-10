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
from pykin.utils.transform_utils import compute_pose_error


def controller(sim, q, q_des,e_prev,e_int,time_step):
    K_p = 0.001
    K_d = 0.1
    K_i = 1
    e = np.zeros(7)

    for i in range(np.size(q_des,axis=0)):
        e[i] = q_des[i]-q[i]		
        e_d = (e[i]-e_prev[i])/time_step
        e_i = e_int[i] + e[i]*time_step
        sim.data.ctrl[i] = K_p*e[i] + K_d * e_d + K_i * e_i
    return e

def get_mujoco_robot_info(sim):
    #sim.model._body_name2id
    #sim.model._joint_name2id
    #sim.model.nu
    pass


model = load_model_from_path("asset/franka_sim/franka_panda.xml")
sim = MjSim(model)
viewer = MjViewer(sim)

urdf_path = './pykin/asset/urdf/panda/panda.urdf'
robot = SingleArm(urdf_path, offset=Transform(rot=[1, 0, 0, 0], pos=[0, 0, 0.5]))
robot.setup_link_name("panda_link0", "panda_link7")

print([sim.model.joint_name2id(joint) for joint in robot.get_revolute_joint_names()])
print(robot.get_all_active_joint_names())
print(sim.model.body_pos[9])
print(sim.model._body_name2id)
q_init = np.array([0 , 0, 0, -1.5708, 0, 1.8675, 0])


q_t = np.array([0.0, np.pi/6, 0.0, -np.pi/2, 0.0, np.pi*5/8 ,0.0])
fk = robot.forward_kin(q_t)
eef_pose = robot.get_eef_pose(fk)

is_limit = False
while not is_limit:
    q_desired = robot.inverse_kin(q_init, eef_pose, method="LM")
    is_limit = robot.check_limit_joint(q_desired)

e_prev = np.zeros(7)
e_int = np.zeros(7)

time_step = 0.002
sim_state = sim.get_state()

while True:
    sim_state = sim.get_state()
    q = sim_state.qpos[0:7]
    
    e_prev = controller(sim,q, q_desired,e_prev,e_int,time_step)
    print(np.round(sim.data.body_xpos[model.body_name2id(robot.eef_name)],6), np.round(eef_pose[:3],6))
    e_int = e_int + e_prev*time_step

    sim.step()
    viewer.render()
