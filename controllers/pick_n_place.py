import numpy as np


class PicknPlace:
    def __init__(
        self, 
        sim,
        viewer,
        mujoco_robot,
        pykin_robot,
        gripper,
        controller,
        
    ):
        self._sim = sim
        self._viewer = viewer
        self._robot = mujoco_robot
        self._pykin = pykin_robot
        self._gripper = gripper
        self._controller = controller

    def move_to_init_pose(self):
        pass

    def select_grasp_pose(self):
        pass

    def compute_grasp_pose(self, init_qpos, target_pose):
        pass

    def _check_ik_solution(self, init_qpos, target_pose):
        is_limit_qpos = False

        result_qpos = self._pykin.inverse_kin(init_qpos, target_pose, method="LM")
        is_limit_qpos = self._pykin.check_limit_joint(result_qpos)
        if is_limit_qpos:
            return result_qpos

        while not is_limit_qpos:
            result_qpos = self._pykin.inverse_kin(np.random.randn(len(init_qpos)), target_pose, method="LM")
            is_limit_qpos = self._pykin.check_limit_joint(result_qpos)
        return result_qpos

    def approach_pose_using_rrt(self, pose):
        pass

    def approach_pose_using_diff_inverse(self, pos):
        pass

    def jmove_in_cspace(self, pose):
        init_qpos = self._robot.init_qpos
        result_qpos = self._check_ik_solution(init_qpos, pose)
        is_reached = False

        while not is_reached:
            torque = self._controller.run_controller(self._sim, result_qpos)
            self._sim.data.ctrl[self._controller.qpos_index] = torque
            if self._controller.is_reached():
                is_reached = True
            self._sim.step()
            self._viewer.render()

    def jmove_in_jspace(self, joints):
        is_reached = False
        while not is_reached:
            torque = self._controller.run_controller(self._sim, joints)
            self._sim.data.ctrl[self._controller.qpos_index] = torque
            if self._controller.is_reached():
                is_reached = True
            self._sim.step()
            self._viewer.render()
    def cmove(self, sim, pose):
        pass

    def stop(self):
        pass

    def open_gripper(self):
        pass

    def close_gripper(self):
        pass

    def _check_grasp(self):
        pass

    def pick(self, pose):
        pass

    def place(self, pose):
        pass
