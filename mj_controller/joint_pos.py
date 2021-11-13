import numpy as np
import sys, os

from mj_controller.base_controller import Controller


class JointPositionController(Controller):
    def __init__(
        self,
        sim,
        eef_name=None,
        arm_dof=7,
        actuator_range=None,
        kp=700, # 5000
        qpos_limits=None,
    ):
        super().__init__(sim, eef_name, arm_dof, actuator_range)

        self.control_dim = arm_dof

        self.kp = self.nums2array(kp, self.control_dim)
        self.ki = self.nums2array(0, self.control_dim) # 20
        self.kd = self.nums2array(0, self.control_dim) # 25
        

        # self.kp[3] = 60
        # self.ki[3] = 15    # 80
        # self.kd[3] = 10     # 50

        # self.kp[4] = 870
        # self.ki[4] = 15    # 80
        # self.kd[4] = 10     # 50

        # self.kp[5] = 120  # 6000
        # self.ki[5] = 50    # 80
        # self.kd[5] = 10    # 50

        self.kp = self.nums2array(0.001, self.control_dim)
        self.ki = self.nums2array(1, self.control_dim) # 20
        self.kd = self.nums2array(0.1, self.control_dim) # 25

        self.position_limits = np.array(qpos_limits) if qpos_limits is not None else self.joint_limits
        self.summed_err = 0
        

    def run_controller(self, sim, q_desired):
        self.goal_qpos = np.array(q_desired)
        assert self.goal_qpos.shape == (len(self.qpos_index),)
        self.update(sim)

        current_q = self.q_pos
        position_error = self.goal_qpos - current_q
        vel_pos_error = -self.q_vel
        self.summed_err = self.summed_err + position_error * self.time_step

        desired_torque = position_error  * self.kp + \
                         self.summed_err * self.ki + \
                         vel_pos_error   * self.kd

        self.torques = desired_torque     
        # self.torques = np.dot(self.mass_matrix, desired_torque) + self.torque_compensation
        
        self.torques = self.clip_torques(self.torques)

        return self.torques

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

    q_desired = [0.0, np.pi/6, 0.0, -np.pi/2, 0.0, np.pi*5/8,0.0]
    # q_desired = [np.pi/3, np.pi/3, 0, 0, np.pi/3, 0, np.pi/2]
    fk = robot.forward_kin(q_desired)
    eef_pose = robot.get_eef_pose(fk)

    is_limit = False

    while not is_limit:
        goal_qpos = robot.inverse_kin(q_init, eef_pose, method="LM")
        is_limit = robot.check_limit_joint(goal_qpos)

    while True:
        torque = joint_controller.run_controller(sim, goal_qpos)
        sim.data.ctrl[joint_controller.qpos_index] = torque
        sim.data.ctrl[joint_controller.gripper_index] = [10, 10]
        
        if joint_controller.is_reached():
            print("contact")
            sim.data.ctrl[joint_controller.gripper_index] = [0.0, 0.0]

        sim.step()
        viewer.render()

