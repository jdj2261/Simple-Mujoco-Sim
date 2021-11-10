import abc
import numpy as np


class Controller(metaclass=abc.ABCMeta):
    def __init__(self,
                 sim,
                 eef_name,
                 joint_indexes,
                 actuator_range
    ):
    
        # Actuator range
        self.actuator_min = actuator_range[0]
        self.actuator_max = actuator_range[1]

        

class PIDControl:

    def __init__(self, kp=0.2, ki=0.0, kd=0.0):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._clear()

    def _clear(self):
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.last_error = 0

        # Windup Guard
        self._windup = 20.0

    def update(self, target, current, delta_time):
        error = target - current
        delta_error = error - self.last_error

        # print(error, delta_error)
        self.p_term = self._kp * error
        self.i_term += self._ki * error * delta_time 

        if (self.i_term < -self._windup):
            self.i_term = -self._windup
        elif (self.i_term > self._windup):
            self.i_term = self._windup

        self.d_term = self._kd * (delta_error / delta_time)

        # Remember last time and last error for next calculation
        self.last_error = error
        output = self.p_term + self.i_term + self.d_term        
        return output

    @property
    def kp(self):
        return self._kp

    @kp.setter
    def kp(self, kp):
        self._kp = kp

    @property
    def kd(self):
        return self._kd

    @kd.setter
    def kd(self, kd):
        self._kd = kd

    @property
    def ki(self):
        return self._ki

    @ki.setter
    def ki(self, ki):
        self._ki = ki

    @property
    def windup(self):
        return self._windup

    @windup.setter
    def windup(self, windup):
        self._windup = windup
