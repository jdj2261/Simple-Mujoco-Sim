import numpy as np


class PicknPlace:
    def __init__(
        self, 
        controller_info="JOINT_POSITION",
    ):
        self.controller = self._get_controller(controller_info)

    def _get_controller(self, info):
        if info == "JOINT_POSITION":
            from controllers.joint_pos import JointPositionController
            controller = JointPositionController()
        if info == "JOINT_VELOCITY":
            from controllers.joint_vel import JointVelocityController
            controller = JointVelocityController() 
        return controller

    def jmove(self, joints):
        pass

    def cmove(self, pose):
        pass

    def stop(self):
        pass

    def open_gripper(self):
        pass

    def close_gripper(self):
        pass

    def pick(self, pose):
        pass

    def place(self, pose):
        pass
