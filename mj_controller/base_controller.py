import abc
import numpy as np

from collections.abc import Iterable

class Controller(metaclass=abc.ABCMeta):
    def __init__(
        self,
        sim,
        robot,
        actuator_range=None
    ):

        self.sim = sim
        self.robot = robot
        self.actuator_range = actuator_range

        if actuator_range is not None:
            self.input_actuator_min = actuator_range[0]
            self.input_actuator_max = actuator_range[1]

        self._setup_mujoco_state()
        self._setup_robot_state()

        self.new_update = True

        self.sim.forward()
        self.update()

    def _setup_mujoco_state(self):
        self.robot_joints = self.robot.get_revolute_joint_names()
        self.qpos_index = [
            self.sim.model.get_joint_qpos_addr(x) 
            for x in self.robot_joints
        ]
        
        self.qvel_index = [
            self.sim.model.get_joint_qvel_addr(x) 
            for x in self.robot_joints
        ]

        # indices for joint indexes
        self.joint_index = [
            self.sim.model.joint_name2id(joint)
            for joint in self.sim.model._joint_name2id.keys()
        ]

        self.actuator_index =  [
            self.sim.model.actuator_name2id(actuator) 
            for actuator in self.sim.model._actuator_name2id.keys()
        ]

        self.gripper_index = list(
            set(self.actuator_index).difference(self.qpos_index)
        )

    def _setup_robot_state(self):
        self.eef_name = self.robot.eef_name
        self.eef_index = self.sim.model.body_name2id(self.eef_name)
        self.ee_pos = None
        self.ee_ori_mat = None
        self.ee_pos_vel = None
        self.ee_ori_vel = None
        self.joint_pos = None
        self.joint_vel = None

    # @abc.abstractmethod
    def run_controller(self):
        """
        Abstract method that should be implemented in all subclass controllers, and should convert a given action
        into torques (pre gravity compensation) to be executed on the robot.
        Additionally, resets the self.new_update flag so that the next self.update call will occur
        """
        self.new_update = True

    def update(self):
        if self.new_update:
            self.sim.forward()

            self.ee_pos = np.array()
            self.new_update = False

    def clip_torques(self, torques):
        return np.clip(torques, self.torque_limits[0], self.torque_limits[1])

    def open_gripper(self):
        pass

    def close_gripper(self):
        pass

    def is_contact(self):
        pass

    def is_reached(self):
        pass

    # @abc.abstractclassmethod
    # def set_goal(self):
    #     pass

    # @abc.abstractclassmethod
    # def run_controller(self):
    #     pass

    # @abc.abstractclassmethod
    # def reset_goal(self):
    #     pass

    @staticmethod
    def nums2array(nums, dim):
        """
        Convert input @nums into numpy array of length @dim. If @nums is a single number, broadcasts it to the
        corresponding dimension size @dim before converting into a numpy array

        Args:
            nums (numeric or Iterable): Either single value or array of numbers
            dim (int): Size of array to broadcast input to env.sim.data.actuator_force

        Returns:
            np.array: Array filled with values specified in @nums
        """
        # First run sanity check to make sure no strings are being inputted
        if isinstance(nums, str):
            raise TypeError("Error: Only numeric inputs are supported for this function, nums2array!")

        # Check if input is an Iterable, if so, we simply convert the input to np.array and return
        # Else, input is a single value, so we map to a numpy array of correct size and return
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
        low = self.sim.model.actuator_ctrlrange[self.actuator_index, 0]
        high = self.sim.model.actuator_ctrlrange[self.actuator_index, 1]
        return low, high

    @property
    def actuator_limits(self):
        if self.actuator_range is None:
            return self.torque_limits
        return self.input_actuator_min, self.input_actuator_max


if __name__ == "__main__":
    from mujoco_py import MjSim, MjViewer, load_model_from_path
    import numpy as np
    import sys, os

    pykin_path = os.path.abspath(os.path.dirname(__file__)+"/../pykin/" )
    sys.path.append(pykin_path)
    print(pykin_path)

    from pykin.robots.single_arm import SingleArm
    from pykin.kinematics.transform import Transform

    model = load_model_from_path("../asset/franka_sim/franka_panda.xml")
    sim = MjSim(model)
    viewer = MjViewer(sim)

    urdf_path = '../pykin/asset/urdf/panda/panda.urdf'
    robot = SingleArm(urdf_path, offset=Transform(rot=[1, 0, 0, 0], pos=[0, 0, 0.5]))
    robot.setup_link_name("panda_link0", "panda_link7")

    controller = Controller(sim=sim, robot=robot)
    print(controller.actuator_index)
    print(controller.joint_index)


    print(sim.data.body_xpos(model.body_name2id(robot.eef_name))
    # print(controller.ref_joint_vel_indexes)
    # print(controller.ref_joint_pos_indexes)
    # print(controller.eef_name)
    # print(controller.ref_gripper_indexes)
    # print(controller.joint_limits)
    # print(controller.torque_compensation)
    # print(controller.torque_limits)


# def joint_position_control(sim, goal_qpos):
#     # desired torque: right arm
#     joint_pos_right = np.array(sim.data.qpos[qpos_idx_right])
#     joint_vel_right = np.array(sim.data.qvel[qvel_idx_right])

#     position_error_right = goal_qpos[:7] - joint_pos_right
#     vel_pos_error_right = -joint_vel_right
#     desired_torque_right = (np.multiply(np.array(position_error_right), kp*np.ones(len(qpos_idx_right)))
#                             + np.multiply(vel_pos_error_right, kd*np.ones(len(qvel_idx_right))))

#     # desired torque: left arm
#     joint_pos_left = np.array(sim.data.qpos[qpos_idx_left])
#     joint_vel_left = np.array(sim.data.qvel[qvel_idx_left])

#     position_error_left = goal_qpos[7:] - joint_pos_left
#     vel_pos_error_left = -joint_vel_left
#     desired_torque_left = (np.multiply(np.array(position_error_left), kp*np.ones(len(qpos_idx_left)))\
#                            + np.multiply(vel_pos_error_left, kd*np.ones(len(qvel_idx_left))))

#     # calculate mass-matrix
#     mass_matrix = np.ndarray(shape=(len(sim.data.qvel) ** 2,), dtype=np.float64, order='C')
#     cymj._mj_fullM(sim.model, mass_matrix, sim.data.qM)
#     mass_matrix = np.reshape(mass_matrix, (len(sim.data.qvel), len(sim.data.qvel)))
#     mass_matrix_right = mass_matrix[qvel_idx_right, :][:, qvel_idx_right]
#     mass_matrix_left = mass_matrix[qvel_idx_left, :][:, qvel_idx_left]

#     # calculate torque-compensation
#     torque_compensation_right = sim.data.qfrc_bias[qvel_idx_right]
#     torque_compensation_left = sim.data.qfrc_bias[qvel_idx_left]

#     # calculate torque values
#     torques_right = np.dot(mass_matrix_right, desired_torque_right) + torque_compensation_right
#     torques_left = np.dot(mass_matrix_left, desired_torque_left) + torque_compensation_left

#     torques = np.concatenate((torques_right, torques_left))
#     torques = np.clip(torques, torques_low, torques_high)

#     goal_reach = True
#     for i in range(len(position_error_right)):
#         if np.abs(position_error_right[i]) > 0.01:
#             goal_reach = False
#             break
#     for i in range(len(position_error_left)):
#         if np.abs(position_error_left[i]) > 0.01:
#             goal_reach = False
#             break

#     return torques, goal_reach
