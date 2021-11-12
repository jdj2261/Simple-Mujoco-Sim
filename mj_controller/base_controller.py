import abc
import numpy as np
import mujoco_py

from collections.abc import Iterable


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

        if actuator_range is not None:
            self.input_actuator_min = actuator_range[0]
            self.input_actuator_max = actuator_range[1]

        self._setup_mujoco_state()
        self._setup_robot_state()

        self.new_update = True
        self.time_step = self.sim.model.opt.timestep

        self.sim_state = self.sim.get_state()
        print(self.sim_state)
        self.sim.forward()
        self.update(self.sim)

    def _setup_mujoco_state(self):
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
       
    def _setup_robot_state(self):
        self.eef_index = self.sim.model.body_name2id(self.eef_name)
        self.eef_pos = None
        self.eef_ori_mat = None
        self.eef_pos_vel = None
        self.eef_ori_vel = None
        self.joint_pos = None
        self.joint_vel = None

    # @abc.abstractmethod
    def run_controller(self):
        """
        Abstract method that should be implemented in all subclass controllers, and should convert a given action
        into torques (pre gravity compensation) to be executed on the robot.
        Additionally, resets the self.new_update flag so that the next self.update call will occur
        """
        pass

    def update(self, sim):
        self.sim = sim
        self.sim.forward()
        self.sim_state = self.sim.get_state()
        self.q_pos = self.sim_state.qpos[self.qpos_index]
        self.q_vel = self.sim_state.qvel[self.qvel_index]
        self.eef_pos = np.array(self.sim.data.body_xpos[self.sim.model.body_name2id(self.eef_name)])
        self.eef_ori_mat = np.array(self.sim.data.body_xmat[self.sim.model.body_name2id(self.eef_name)].reshape([3, 3]))
        self.eef_pos_vel = np.array(self.sim.data.body_xvelp[self.sim.model.body_name2id(self.eef_name)])
        self.eef_ori_vel = np.array(self.sim.data.body_xvelr[self.sim.model.body_name2id(self.eef_name)])

        self.torque_compensation =  self.sim.data.qfrc_bias[self.qvel_index]
        
        self.joint_pos = np.array(self.sim.data.qpos[self.qpos_index])
        self.joint_vel = np.array(self.sim.data.qvel[self.qvel_index])

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

    def open_gripper(self):
        pass

    def close_gripper(self):
        pass

    def is_contact(self):
        pass

    @abc.abstractmethod
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

    # @property
    # def torque_compensation(self):
    #     return self.sim.data.qfrc_bias[self.qvel_index]

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

    print(sim.model.joint_names)
    controller = Controller(sim=sim, eef_name=robot.eef_name, arm_dof=robot.arm_dof)

    # print(controller.eef_pos)
    # print(controller.ee_ori_mat)
    # print(controller.J_full)
    # print(controller.mass_matrix)
    print(controller.torque_limits)

    # print(controller.qpos_index)
    # print(controller.qvel_index)
    # print(controller.actuator_index)
    # print(controller.joint_index)


    # print(sim.data.body_xpos(model.body_name2id(robot.eef_name)))

