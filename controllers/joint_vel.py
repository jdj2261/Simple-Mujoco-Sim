import numpy as np
from controllers.base_controller import Controller

class JointVelocityController(Controller):
    def __init__(
        self,
        sim,
        eef_name=None,
        arm_dof=7,
        actuator_range=None,
        kp=0.25,
        qvel_limits=[-1, 1],
    ):
        super().__init__(
            sim, 
            eef_name, 
            arm_dof, 
            actuator_range,
        )

        self.control_dim = arm_dof
        self._kp = self.nums2array(kp, self.control_dim)
        self._ki = self.kp * 0.005
        self._kd = self.kp * 0.001
        self.last_err = np.zeros(self.control_dim)
        
        self.velocity_limits = np.array(qvel_limits)
        self.summed_err = 0                
        self.windup = self.nums2array(self.torque_limits[1]/2, self.control_dim)
        
    def run_controller(self, sim, qvel_desired):
        self.goal_qvel = np.array(qvel_desired)
        assert self.goal_qvel.shape == (len(self.qvel_index),)

        self.update(sim)

        current_vel = np.clip(self.q_vel, self.actuator_limits[0], self.actuator_limits[1])
        
        velocity_error = self.goal_qvel - current_vel
        d_vel_error =  velocity_error - self.last_err
        self.last_err = velocity_error

        self.summed_err = self.summed_err + velocity_error * self.time_step
        self.summed_err = self.clip_anti_windup(self.summed_err)
        
        self.err_qvel = np.array([abs(velocity_error[i]) for i in self.qpos_index])

        desired_torque = velocity_error  * self._kp + \
                         self.summed_err * self._ki + \
                         d_vel_error     * self._kd

        desired_torque = np.dot(self.mass_matrix, desired_torque) + self.torque_compensation
        self.torques = self.clip_torques(desired_torque)

        return self.torques

    def clip_anti_windup(self, torques):
        return np.clip(torques, -self.windup, self.windup)

    def is_reached(self):
        eps = 1e-2
        return True if np.all(self.err_qvel < eps) else False

