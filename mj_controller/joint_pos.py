import numpy as np
import sys, os

mj_controller_path = os.path.abspath(os.path.dirname(__file__)+"/.." )
sys.path.append(mj_controller_path)

from mj_controller.base_controller import Controller

# kp 500
# ki 22
# kd 0.01

#self.kp = 150
#self.ki = 15
#self.kd = 2.2 * np.sqrt(self.kp) * damping_ratio

class JointPositionController(Controller):
    def __init__(
        self,
        sim,
        eef_name=None,
        arm_dof=7,
        actuator_range=None,
        kp=5000,
        qpos_limits=None,
    ):
        super().__init__(sim, eef_name, arm_dof, actuator_range)

        self.control_dim = arm_dof
        self.position_limits = np.array(qpos_limits) if qpos_limits is not None else self.joint_limits
        self.kp = self.nums2array(kp, self.control_dim)
        self.ki = self.nums2array(20, self.control_dim)

        self.kd = self.nums2array(25, self.control_dim)
        self.kp[5] = 6000
        self.ki[5] = 80
        self.kd[5] = 50
        # self.kd[5] = 100
        # self.kd = 2. * np.sqrt(self.kp) * 3
        # self.ki = self.nums2array(15, self.control_dim)
        # self.kp[5] = 300
        # self.kd[5] = 200
        # self.ki[5] = 100
        # self.ki = self.nums2array(0, self.control_dim)
        # self.kd = self.nums2array(0, self.control_dim)

        # self.kp[0] = 50
        # self.kp[1] = 70
        # self.kp[2] = 50
        # self.kp[3] = 50
        # self.kp[4] = 50
        # self.kp[5] = 50
        # self.kp[6] = 50

        # self.ki[0] = 10
        # self.ki[1] = 50
        # self.ki[2] = 1
        # self.ki[3] = 30
        # self.ki[4] = 50
        # self.ki[5] = 100
        # self.ki[6] = 1

        # self.kd[0] = 10
        # self.kd[1] = 10
        # self.kd[2] = 10
        # self.kd[3] = 10
        # self.kd[4] = 10
        # self.kd[5] = 10
        # self.kd[6] = 10

        # self.ki = 25
        # self.ki = self.kp * 10
        # self.kd = self.kp * 1
        self.summed_err = 0

    def move_to_qpos(self, sim, q_desired):
        self.goal_qpos = q_desired
        self.update(sim)
        current_q = self.q_pos
        position_error = np.array(q_desired) - current_q
        vel_pos_error = -self.q_vel
        self.summed_err = self.summed_err + position_error * self.time_step

        desired_torque = position_error  * self.kp + \
                            self.summed_err * self.ki + \
                            vel_pos_error   * self.kd
        
        # print(self.q_pos)
        # print(self.goal_qpos)
        # print()
        
        self.torques = np.dot(self.mass_matrix, desired_torque) + self.torque_compensation
        # self.torques = self.clip_torques(self.torques)

        return self.torques


    def reset_goal(self):
        self.goal_qpos = self.joint_pos

    def is_reached(self):
        EPS = 1e-2
        _is_reached = False

        err_qpos = np.array([abs(self.joint_pos[i] - self.goal_qpos[i]) for i in self.qpos_index])
        print(np.round(err_qpos, 3))
        if np.all(err_qpos < EPS):
            _is_reached = True

        return _is_reached


if __name__ == "__main__":
    from mujoco_py import MjSim, MjViewer, load_model_from_path
    import numpy as np
    import sys, os

    pykin_path = os.path.abspath(os.path.dirname(__file__)+"/../pykin/" )
    sys.path.append(pykin_path)
    print(pykin_path)

    from pykin.robots.single_arm import SingleArm
    from pykin.kinematics.transform import Transform
    from pykin.utils.transform_utils import get_h_mat

    model = load_model_from_path("../asset/franka_sim/franka_panda.xml")
    sim = MjSim(model)
    viewer = MjViewer(sim)

    urdf_path = '../pykin/asset/urdf/panda/panda.urdf'
    robot = SingleArm(urdf_path, offset=Transform(rot=[1, 0, 0, 0], pos=[0, 0, 0.5]))
    robot.setup_link_name("panda_link0", "panda_link7")

    joint_controller = JointPositionController(sim=sim, eef_name=robot.eef_name)

    q_init = np.array([0 , 0, 0, -1.5708, 0, 1.8675, 0])

    q_desired = np.array([0, np.pi/7, 0.0, -np.pi/3, 0.0, np.pi*5/8 ,0.0])
    # q_desired = np.array([0, np.pi/6, 0, 0, 0.0, 0 ,0.0])
    fk = robot.forward_kin(q_desired)
    eef_pose = robot.get_eef_pose(fk)

    is_limit = False

    while not is_limit:
        goal_qpos = robot.inverse_kin(q_init, eef_pose, method="LM")
        is_limit = robot.check_limit_joint(goal_qpos)

    while True:
        torque = joint_controller.move_to_qpos(sim, q_desired)
        sim.data.ctrl[joint_controller.qpos_index] = torque
        
        if joint_controller.is_reached():
            print("is reached")

        sim.step()
        viewer.render()


    # print(sim.model.joint_names)
    # print(sim.model.geom_names)
    # print(sim.model.actuator_names)
    

    # print(joint_controller.control_dim)
    # print(sim.model.opt.timestep)
    # joint_controller.time_step = 0.02
    # print(joint_controller.time_step)
    # print(sim.model.opt.timestep)
    # joint_controller.set_time_step(0.001)
    # print(joint_controller.get_time_step())