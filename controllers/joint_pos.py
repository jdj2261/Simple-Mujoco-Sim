import numpy as np
from controllers.base_controller import Controller


class JointPositionController(Controller):
    def __init__(
        self,
        sim,
        eef_name=None,
        arm_dof=7,
        actuator_range=None,
        kp=50,
        ki=0.1,
        kd=22,
        qpos_limits=None,
    ):
        super().__init__(
            sim, 
            eef_name, 
            arm_dof, 
            actuator_range,
        )

        self.control_dim = arm_dof
        self._kp = self.nums2array(kp, self.control_dim)
        self._ki = self.nums2array(ki, self.control_dim) 
        self._kd = self.nums2array(kd, self.control_dim)
        
        self.position_limits = np.array(qpos_limits) if qpos_limits is not None else self.joint_limits
        self.summed_err = np.zeros(self.control_dim)
        self.err_qpos = None
        self._is_reached_result = False
        self.windup = self.nums2array(self.torque_limits[1]/2, self.control_dim)
        
    def run_controller(self, sim, qpos_desired):
        self.goal_qpos = np.array(qpos_desired)
        assert self.goal_qpos.shape == (len(self.qpos_index),)
        
        self.update(sim)

        current_q = self.q_pos
        position_error = self.goal_qpos - current_q
        vel_pos_error = -self.q_vel
        self.summed_err = self.summed_err + position_error * self.time_step
        self.summed_err = self.clip_anti_windup(self.summed_err)
        
        self.err_qpos = np.array([abs(position_error[i]) for i in self.qpos_index])

        desired_torque = position_error  * self._kp + \
                         self.summed_err * self._ki + \
                         vel_pos_error   * self._kd

        desired_torque = np.dot(self.mass_matrix, desired_torque) + self.torque_compensation
        self.torques = self.clip_torques(desired_torque)

        super().run_controller()
        return self.torques

    def clip_anti_windup(self, torques):
        return np.clip(torques, -self.windup, self.windup)

    def is_reached(self, eps=0.01):
        if self.err_qpos is None:
            return self._is_reached_result

        if np.all(self.err_qpos < eps) and not self._is_reached_result:
            self._is_reached_result = True
            return self._is_reached_result

        self._is_reached_result = False
        return self._is_reached_result