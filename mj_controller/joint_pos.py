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
        kp=50,
        ki=0,
        kd=22,
        qpos_limits=None,
    ):
        super().__init__(sim, eef_name, arm_dof, actuator_range)

        self.control_dim = arm_dof
        
        self.windup = self.nums2array(1, self.control_dim)

        self._kp = self.nums2array(kp, self.control_dim)
        self._ki = self.nums2array(ki, self.control_dim) 
        self._kd = self.nums2array(kd, self.control_dim)
        
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
        self.summed_err = self.clip_anti_wiseup(self.summed_err)
        
        desired_torque = position_error  * self._kp + \
                         self.summed_err * self._ki + \
                         vel_pos_error   * self._kd

        self.err_qpos = np.array([abs(self.joint_pos[i] - self.goal_qpos[i]) for i in self.qpos_index])
                
        self.torques = desired_torque     
        self.torques = np.dot(self.mass_matrix, desired_torque) + self.torque_compensation
        
        self.torques = self.clip_torques(self.torques)

        return self.torques

    def clip_anti_wiseup(self, torques):
        return np.clip(torques, -self.windup, self.windup)