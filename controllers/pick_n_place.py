import numpy as np


class PicknPlace:
    def __init__(
        self, 
        sim,
        viewer,
        mujoco_robot,
        pykin_robot,
        controller,
        gripper,
        cartesisan_planner,
        rrt_planner,
    ):
        self._sim = sim
        self._viewer = viewer
        self._robot = mujoco_robot
        self._pykin = pykin_robot
        self._controller = controller
        self._gripper = gripper
        self._c_planner = cartesisan_planner
        self._rrt_planner = rrt_planner

    def move_to_init_pose(self):
        self.jmove_in_jspace(self._robot.init_qpos)

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

    def approach_pose_using_rrt(self, pose, is_interpolated=True):
        interpolated_path, joint_path = self._rrt_planner.get_path_in_joinst_space(
            cur_q=self._controller.q_pos, 
            goal_pose=pose,
            resolution=1)
        
        if is_interpolated:
            for _, qpos in enumerate(interpolated_path):
                self.jmove_in_jspace(qpos)
        else:
            for _, qpos in enumerate(joint_path):
                self.jmove_in_jspace(qpos)
        print("reach")

    def jmove_in_cspace(self, pose):
        init_qpos = self._robot.init_qpos
        result_qpos = self._check_ik_solution(init_qpos, pose)
        self.jmove_in_jspace(result_qpos)

    def jmove_in_jspace(self, joints):
        while not self._controller.is_reached():
            self.torque = self._controller.run_controller(self._sim, joints)
            self._sim.data.ctrl[self._controller.qpos_index] = self.torque
            self._sim.step()
            self._viewer.render()

    def cmove(self, pose, resolution=0.01, damping=0.03, pos_sensitivity=0.03, is_slerp=False):
        joint_path, _ = self._c_planner.get_path_in_joinst_space(            
            current_q=self._controller.q_pos,
            goal_pose=pose,
            resolution=resolution, 
            damping=damping,
            pos_sensitivity=pos_sensitivity,
            is_slerp=is_slerp)

        for _, qpos in enumerate(joint_path):
            self.jmove_in_jspace(qpos)
        print("reach")

    def stop(self):
        pass

    def open_gripper(self):
        for _ in range(1000):
            torqe = self._controller.run_controller(self._sim, self._controller.q_pos)
            self._sim.data.ctrl[self._controller.qpos_index] = torqe
            self._sim.data.ctrl[self._controller.gripper_index] = [0.4, -0.4]
            # self._sim.data.ctrl[self._controller.gripper_index] = [-0.020833, 0.020833] // Rethink
            # self._sim.data.ctrl[self._controller.gripper_index] = [0.0, 0.0] // Robotiq140
            self._sim.step()
            self._viewer.render()

    def close_gripper(self):
        for _ in range(1000):
            torqe = self._controller.run_controller(self._sim, self._controller.q_pos)
            self._sim.data.ctrl[self._controller.qpos_index] = torqe
            self._sim.data.ctrl[self._controller.gripper_index] = [0.0, 0.0]
            # self._sim.data.ctrl[self._controller.gripper_index] = [0.7, -0.7] // Robotiq140
            # self._sim.data.ctrl[self._controller.gripper_index] = [0.0115, -0.0115] // Rethink
            self._sim.step()
            self._viewer.render()

    def _check_grasp(self):
        pass

    def pick(self, pose):
        self.jmove_in_cspace(pose)
        self.open_gripper()
        self.cmove([ 0.7, -0.3, 0.97])
        self.close_gripper()
            
    def place(self, pose):
        self.jmove_in_cspace(pose)
        self.cmove([0.6, 0.4, 1.05])
        self.open_gripper()
