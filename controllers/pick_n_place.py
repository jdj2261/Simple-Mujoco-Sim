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
            for i, qpos in enumerate(interpolated_path):
                self.jmove_in_jspace(qpos)
                print(f"{i+1}/{len(interpolated_path)}")
        else:
            for i, qpos in enumerate(joint_path):
                self.jmove_in_jspace(qpos)
                print(f"{i+1}/{len(joint_path)}")
        print("reach")

    def jmove_in_cspace(self, pose):
        init_qpos = self._robot.init_qpos
        result_qpos = self._check_ik_solution(init_qpos, pose)
        self.jmove_in_jspace(result_qpos)
        
    def jmove_in_jspace(self, joints):
        while not self._controller.is_reached():
            torque = self._controller.run_controller(self._sim, joints)
            self._sim.data.ctrl[self._controller.qpos_index] = torque
            self._sim.step()
            self._viewer.render()

    def cmove(self, pose, resolution=1, damping=0.03, pos_sensitivity=0.03, is_slerp=False):
        joint_path, _ = self._c_planner.get_path_in_joinst_space(            
            current_q=self._controller.q_pos,
            goal_pose=pose,
            resolution=resolution, 
            damping=damping,
            pos_sensitivity=pos_sensitivity,
            is_slerp=is_slerp)

        for i, qpos in enumerate(joint_path):
            self.jmove_in_jspace(qpos)
            print(f"{i+1}/{len(joint_path)}")
        print("reach")

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
