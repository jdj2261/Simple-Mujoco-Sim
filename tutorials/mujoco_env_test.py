from mujoco_py import MjSim, MjViewer, load_model_from_path
import numpy as np
import sys, os

pykin_path = os.path.abspath(os.path.dirname(__file__)+"/../pykin/" )
sys.path.append(pykin_path)
print(pykin_path)
from pykin.robots.single_arm import SingleArm
from pykin.kinematics.transform import Transform


# def controller(sim, q, q_des,e_prev,e_int,time_step):
#     K_p = 0.001
#     K_d = 0.1
#     K_i = 1
#     e = np.zeros(7)
#     e_ds = np.zeros(7)
#     e_is = np.zeros(7)
#     for i in range(np.size(q_des,axis=0)):
#         e[i] = q_des[i]-q[i]		
#         e_d = (e[i]-e_prev[i])/time_step
#         e_i = e_int[i] + e[i]*time_step
#         e_ds[i] = e_d
#         e_is[i] = e_i
#         sim.data.ctrl[i] = K_p*e[i] + K_d * e_d + K_i * e_i
#     print(e)
#     # print(e_ds)
#     # print(e_is)
#     return e

def controller(sim, q, q_des, time_step):
    K_p = 0.001
    K_d = 0.1
    K_i = 1
    e = np.zeros(7)
    e_ds = np.zeros(7)
    e_is = np.zeros(7)

    for i in range(np.size(q_des,axis=0)):
        e[i] = q_des[i]-q[i]		
        e_d[i] = (e[i]-e_prev[i])/time_step
        e_i = e_int[i] + e[i]*time_step
        e_ds[i] = e_d
        e_is[i] = e_i
        sim.data.ctrl[i] = K_p*e[i] + K_d * e_d + K_i * e_i
    e_prev = e
    print(e)
    # print(e_ds)
    # print(e_is)
    return e


def finger_open(sim):
	left = 7
	right = 8
	sim.data.ctrl[left] = 0.04
	sim.data.ctrl[right] = 0.04

def finger_close(sim):
	left = 7
	right = 8
	sim.data.ctrl[left] = 0.0
	sim.data.ctrl[right] = 0.0


def check_pose_reached(q, q_desir):
	EPS = 1e-2
	reached_joints=np.zeros(7)
	for i in range(np.size(q_desir,axis=0)):
		if abs(q[i]-q_desir[i])<EPS:
			reached_joints[i] = 1
		else:
			reached_joints[i] = 0

	reached_pose = True
	for i in range(np.size(q_desir,axis=0)):
		reached_pose = reached_pose and reached_joints[i]

	return reached_pose


def get_mujoco_robot_info(sim):
    #sim.model._body_name2id
    #sim.model._joint_name2id
    #sim.model.nu
    pass


model = load_model_from_path("../asset/franka_sim/franka_panda.xml")
sim = MjSim(model)
viewer = MjViewer(sim)

urdf_path = '../pykin/asset/urdf/panda/panda.urdf'
robot = SingleArm(urdf_path, offset=Transform(rot=[1, 0, 0, 0], pos=[0, 0, 0.5]))
robot.setup_link_name("panda_link0", "panda_link7")

print([sim.model.joint_name2id(joint) for joint in robot.get_revolute_joint_names()])
print(robot.get_all_active_joint_names())
print(sim.model.body_pos[9])
print(sim.model._body_name2id)
print(sim.model._actuator_name2id)
q_init = np.array([0 , 0, 0, -1.5708, 0, 1.8675, 0])


q_t = np.array([0.0, np.pi/6, 0.0, -np.pi/2, 0.0, np.pi*5/8 ,0.0])
q_t = np.array([0.0, np.pi/6, 0.0, np.pi, 0.0, np.pi*5/8 ,0.0])
fk = robot.forward_kin(q_t)
eef_pose = robot.get_eef_pose(fk)

is_limit = False
while not is_limit:
    q_desired = robot.inverse_kin(q_init, eef_pose, method="LM")
    is_limit = robot.check_limit_joint(q_desired)

err_pose= robot.get_pose_error(target=robot.get_eef_h_mat(fk), result=robot.get_eef_h_mat(robot.forward_kin(q_desired)))
print(err_pose)

e_prev = np.zeros(7)
e_int = np.zeros(7)

time_step = 0.002
# sim_state = sim.get_state()
# sim.model.opt.timestep = 0.02
pre_time = 0

def joint_control(q_desired, q_in, time_step, e_prev, e_int):
    K_p = 0.001
    K_d = 0.1
    K_i = 1
    e = np.zeros(7)
    e_d = np.zeros(7)
    e_i = np.zeros(7)
    torque = np.zeros(7)

    for i in range(np.size(q_desired, axis=0)):
        e[i] = q_desired[i]-q_in[i]		
        e_d[i] = (e[i]-e_prev[i]) / time_step
        e_i[i] = e_int[i] + e[i] * time_step
        torque[i] = K_p*e[i] + K_d * e_d[i] + K_i * e_i[i]

    e_prev = e
    e_int = e_i

    return torque, e_prev, e_int

while sim.data.time < 10:
    current_time = sim.data.time
    sim_state = sim.get_state()
    q = sim_state.qpos[0:7]

    torque, e_prev, e_int = joint_control(q_desired, q, time_step, e_prev, e_int)
    sim.data.ctrl[:7] = torque
    # K_p = 0.001
    # K_d = 0.1
    # K_i = 1
    # e = np.zeros(7)
    # e_d = np.zeros(7)
    # e_i = np.zeros(7)

    # for i in range(np.size(q_desired, axis=0)):
    #     e[i] = q_desired[i]-q[i]		
    #     e_d[i] = (e[i]-e_prev[i]) / time_step
    #     e_i[i] = e_int[i] + e[i] * time_step
    #     sim.data.ctrl[i] = K_p*e[i] + K_d * e_d[i] + K_i * e_i[i]



    # e_prev = controller(sim, q, q_desired, e_prev, e_int, time_step)
    # print(np.round(sim.data.body_xpos[model.body_name2id(robot.eef_name)],6), np.round(eef_pose[:3],6))
    # e_int = e_int + e_prev*time_step

    finger_open(sim)

    # error_pose = robot.get_pose_error()
    # print(sim.model.opt.timestep)
    pre_time = current_time

    sim.step()
    viewer.render()
