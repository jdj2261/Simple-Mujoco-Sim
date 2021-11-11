import numpy as np
import sys, os

mj_controller_path = os.path.abspath(os.path.dirname(__file__)+"/.." )
sys.path.append(mj_controller_path)

from mj_controller.base_controller import Controller


class JointPositionController(Controller):
    def __init__(
        self,
        sim,
        robot=None,
        actuator_range=None,
        qpos_limits=None,
    ):
        super().__init__(sim, robot, actuator_range)

        self.control_dim = len(self.ref_joint_pos_indexes)
        self.position_limits = np.array(qpos_limits) if qpos_limits is not None else self.joint_limits
        self.goal_qpos = None

    def run_controller(self, goal_qpos):
        self.update()
        
    def reset_goal(self):
        pass


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

    joint_controller = JointPositionController(sim=sim, robot=robot)
    print(joint_controller.control_dim)