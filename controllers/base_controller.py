import abc
import numpy as np
import mujoco_py

from collections.abc import Iterable
from utils.transform_utils import pose2mat

class Controller(metaclass=abc.ABCMeta):
    def __init__(
        self,
        sim,
        eef_name,
        arm_dof,
        actuator_range=None
    ):

        self.sim = sim
        self.eef_name = eef_name
        self.arm_dof = arm_dof
        self.actuator_range = actuator_range
        self.goal_qpos = None
        
        if actuator_range is not None:
            self.input_actuator_min = actuator_range[0]
            self.input_actuator_max = actuator_range[1]

        self._setup_robot_state()

        self.new_update = True
        self.sim.forward()
        self.time_step = self.sim.model.opt.timestep
        self.sim_state = self.sim.get_state()
        self.update(self.sim)

    @abc.abstractmethod
    def run_controller(self):
        self.new_update = True

    @abc.abstractmethod
    def is_reached(self):
        pass
    
    def _setup_robot_state(self):
        self.arm_joints = [self.sim.model.joint_id2name(x) for x in range(self.arm_dof)]
        self.qpos_index = [self.sim.model.get_joint_qpos_addr(x) for x in self.arm_joints]
        self.qvel_index = [self.sim.model.get_joint_qvel_addr(x) for x in self.arm_joints]
 
        # indices for joint indexes
        self.joint_index = [
            self.sim.model.joint_name2id(joint)
            for joint in self.sim.model._joint_name2id.keys()
        ]

        self.actuator_index =  [
            self.sim.model.actuator_name2id(actuator) 
            for actuator in self.sim.model._actuator_name2id.keys()
        ]

        self.gripper_index = list(set(self.actuator_index).difference(self.qpos_index))
        self.eef_index = self.sim.model.body_name2id(self.eef_name)
        self.eef_pos = None
        self.eef_ori_mat = None
        self.eef_pose = None
        self.eef_pos_vel = None
        self.eef_ori_vel = None
        self.err_qpos = None

    def update(self, sim):

        if self.new_update:
            self.sim = sim
            # self.sim.forward()
            self.sim_state = self.sim.get_state()
            self.q_pos = self.sim_state.qpos[self.qpos_index]
            self.q_vel = self.sim_state.qvel[self.qvel_index]

            self.eef_pos = np.array(self.sim.data.body_xpos[self.sim.model.body_name2id(self.eef_name)])
            self.eef_ori_mat = np.array(self.sim.data.body_xmat[self.sim.model.body_name2id(self.eef_name)].reshape([3, 3]))
            self.eef_pose = pose2mat(R=self.eef_ori_mat, p=self.eef_pos)

            self.eef_pos_vel = np.array(self.sim.data.body_xvelp[self.sim.model.body_name2id(self.eef_name)])
            self.eef_ori_vel = np.array(self.sim.data.body_xvelr[self.sim.model.body_name2id(self.eef_name)])

            self.J_pos = np.array(self.sim.data.get_body_jacp(self.eef_name).reshape((3, -1))[:, self.qvel_index])
            self.J_ori = np.array(self.sim.data.get_body_jacr(self.eef_name).reshape((3, -1))[:, self.qvel_index])
            self.J_full = np.array(np.vstack([self.J_pos, self.J_ori]))

            mass_matrix = np.ndarray(shape=(len(self.sim.data.qvel) ** 2,), dtype=np.float64, order='C')
            mujoco_py.cymj._mj_fullM(self.sim.model, mass_matrix, self.sim.data.qM)
            mass_matrix = np.reshape(mass_matrix, (len(self.sim.data.qvel), len(self.sim.data.qvel)))
            self.mass_matrix = mass_matrix[self.qvel_index, :][:, self.qvel_index]

        self.new_update = False

    def clip_torques(self, torques):
        return np.clip(torques, self.torque_limits[0], self.torque_limits[1])

    #TODO
    def is_contact(self):
        contact_l = self.sim.data.contact[self.gripper_index[0]]
        contact_r = self.sim.data.contact[self.gripper_index[1]]
        is_left_contact = False
        is_right_contact = False

        if self.sim.model.geom_id2name(contact_l.geom1) == "object" or self.sim.model.geom_id2name(contact_l.geom2) == "object":
            is_left_contact = True
        if self.sim.model.geom_id2name(contact_r.geom1) == "object" or self.sim.model.geom_id2name(contact_r.geom2) == "object":
            is_right_contact = True

        if (is_left_contact and contact_l.dist < 1e-3) and \
           (is_right_contact and contact_r.dist < 1e-3):
            return True
        return False

    @staticmethod
    def nums2array(nums, dim):
        if isinstance(nums, str):
            raise TypeError("Error: Only numeric inputs are supported for this function, nums2array!")
        return np.array(nums) if isinstance(nums, Iterable) else np.ones(dim) * nums

    @property
    def torque_compensation(self):
        return self.sim.data.qfrc_bias[self.qvel_index]

    @property
    def joint_limits(self):
        low = self.sim.model.jnt_range[self.joint_index, 0]
        high = self.sim.model.jnt_range[self.joint_index, 1]
        return low, high

    @property
    def torque_limits(self):
        low = self.sim.model.actuator_ctrlrange[self.qpos_index, 0]
        high = self.sim.model.actuator_ctrlrange[self.qpos_index, 1]
        return low, high

    @property
    def actuator_limits(self):
        if self.actuator_range is None:
            return self.torque_limits
        return self.input_actuator_min, self.input_actuator_max

    @property
    def time_step(self):
        return self.sim.model.opt.timestep

    @time_step.setter
    def time_step(self, time):
        self.sim.model.opt.timestep = time

    @property
    def kp(self):
        return self._kp

    @kp.setter
    def kp(self, kp):
        self._kp = kp

    @property
    def ki(self):
        return self._ki

    @ki.setter
    def ki(self, ki):
        self._ki = ki

    @property
    def kd(self):
        return self._kd

    @kd.setter
    def kd(self, kd):
        self._kd = kd