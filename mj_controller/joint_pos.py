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
        kp=0.001, # 5000
        qpos_limits=None,
    ):
        super().__init__(sim, eef_name, arm_dof, actuator_range)

        self.control_dim = arm_dof

        self.kp = self.nums2array(kp, self.control_dim)
        self.ki = self.nums2array(1, self.control_dim) 
        self.kd = self.nums2array(0.1, self.control_dim)
        
        # self.kp[3] = 60
        # self.ki[3] = 15    # 80
        # self.kd[3] = 10    # 50

        # self.kp[4] = 870
        # self.ki[4] = 15    # 80
        # self.kd[4] = 10    # 50

        # self.kp[5] = 120   # 6000
        # self.ki[5] = 50    # 80
        # self.kd[5] = 10    # 50

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