import robosuite as suite
import numpy as np

from PIL import Image
from IPython.display import display
from robosuite import robots

from robosuite.utils.transform_utils import *
from transform import *


# env = suite.make(
#     env_name="Lift",
#     robots="Panda",
#     gripper_types=None,
#     has_renderer=True,
#     has_offscreen_renderer=False,
#     use_camera_obs=False,
#     use_object_obs=False,
#     control_freq=50,
#     render_camera=None,
#     horizon=2000, 
# )

env = suite.make(
    env_name="Lift",
    robots="Panda",
    has_renderer=True,
    ignore_done=True,
    has_offscreen_renderer=False,
    use_camera_obs=False,
    camera_names="frontview",
)

def getBodyMat(body_name):
    body_id = env.sim.model.body_name2id(body_name)
    R = np.array(env.sim.data.body_xmat[body_id]).reshape(3, 3)
    p = np.array(env.sim.data.body_xpos[body_id])
    M = pose2mat(R, p)
    return M

jnt_idx = env.robots[0]._ref_joint_indexes
n = len(jnt_idx) # Franka is a 7-dof arm

for i in range(n):
    k = env.sim.model.jnt_bodyid[i]
    print(env.sim.model.body_id2name(k))

# reset the robot to its zero pose
env.reset()
env.robots[0].set_robot_joint_positions(np.zeros(n))

# birdview = env.sim.render(height=256, width=256, camera_name="birdview")[::-1]
# frontview = env.sim.render(height=256, width=256, camera_name="frontview")[::-1]
# display(Image.fromarray(birdview), Image.fromarray(frontview))
# Image.fromarray(birdview).show()
# Image.fromarray(frontview).show()


# the configuration of the end-effector frame relative to 
# the fixed base frame in its zero pose
w0 = np.array(env.sim.data.xaxis)[jnt_idx]
p0 = np.array(env.sim.data.xanchor)[jnt_idx]
v0 = -np.cross(w0, p0)

theta0 = env.sim.data.qpos[jnt_idx]

M0 = dict()
for i in range(1, 8):
    joint_name = "robot0_link{}".format(i)
    M0[joint_name] = getBodyMat(joint_name)

body_name = "robot0_right_hand"
M0[body_name] = getBodyMat(body_name)

# Joints are also bodies. These are two equivalent ways
# of retrieving the pose
body_idx = [env.sim.model.jnt_bodyid[x] for x in jnt_idx]
body_p0 = np.array(env.sim.data.body_xpos[body_idx])
assert np.allclose(body_p0, p0)

# sanity check (no motion no change)
# All Ts should be identity matrix
M = M0[body_name]
for i in reversed(range(n)):
    T = exp2mat(w0[i], v0[i], theta0[i])
    M = np.matmul(T, M)
assert np.allclose(M, M0["robot0_right_hand"])

env.reset()
T_desired = getBodyMat(body_name)
theta_desired = env.sim.data.qpos[jnt_idx]

low, high = env.action_spec # get action limits
for i in range(100):
    action = np.random.uniform(low, high) # sample random action
    obs, reward, done, _ = env.step(action)
    env.render()

# Product of Exponentials formula
body_name = "robot0_right_hand"
theta = env.sim.data.qpos[jnt_idx]

def forward_kinematics(w_init, v_init, theta, M_init):
    M = M_init
    for i in reversed(range(n)):
        T = exp2mat(w_init[i], v_init[i], theta[i])
        M = np.matmul(T, M)
    return M    

M_final = forward_kinematics(w0, v0, theta, M0[body_name])

R, p = mat2pose(M_final)
R_gt, p_gt = mat2pose(getBodyMat(body_name))

# compute rotation error as angle of the two rotations
R_err = np.arccos((np.trace(np.matmul(R, R_gt.T)) - 1)/2.0)
p_err = np.linalg.norm(p - p_gt)

print("rotation err:", R_err)
print("position err:", p_err)

# def space_jacobian(w_init, v_init, theta):
#     # screw axis at rest place
#     S = np.hstack([w_init, v_init]) 
#     J = []
#     Ts = np.eye(4)

#     # compute each column of the Jacobian
#     for i in range(n):
#         row = adjoint(Ts).dot(S[i])
#         J.append(row)
#         T = exp2mat(w0[i], v0[i], theta[i])
#         Ts = np.matmul(Ts, T)

#     return np.array(J).T

# def velocity_kinematics(J, theta_dot):
#     V = J.dot(theta_dot) # spatial twist
#     return V

# Js = space_jacobian(w0, v0, theta)
# theta_dot = env.sim.data.qvel[jnt_idx]

# V = velocity_kinematics(Js, theta_dot)

# # rotational velocity of end-effector
# w_pred = V[:3]

# # linear velocity of end-effector
# v = V[3:]
# Vm = np.vstack([
#     np.hstack([skew_sym(w_pred), v[:,np.newaxis]]),
#     np.zeros([1, 4])
# ])

# T_dot = np.matmul(Vm, M_final)
# _, v_pred = mat2pose(T_dot)

# v_gt = env.sim.data.get_body_xvelp(body_name)
# v_err = np.linalg.norm(v_pred - v_gt)
# print('linear velocity err', v_err)

# w_gt = env.sim.data.get_body_xvelr(body_name)
# w_err = np.linalg.norm(w_pred - w_gt)
# print('angular velocity err', w_err)

# # set the arm to its zero position (all joint position = 0)
# birdview = env.sim.render(height=256, width=256, camera_name="birdview")[::-1]
# frontview = env.sim.render(height=256, width=256, camera_name="frontview")[::-1]
# display(Image.fromarray(birdview), Image.fromarray(frontview))

# print(Js.T)

# # get ground-truth rotation jacobian
# nv = env.sim.model.nv
# env.sim.data.get_body_jacp(body_name).reshape(3, nv)[:, jnt_idx].dot(theta_dot)
# print(env.sim.data.get_body_jacr(body_name).reshape(3, nv)[:, jnt_idx].T)

# print(env.sim.data.get_body_jacp(body_name).reshape(3, nv)[:, jnt_idx].T)

# R, p = mat2pose(M_final)

# print(np.matmul(R, Js[3:, :]).T)