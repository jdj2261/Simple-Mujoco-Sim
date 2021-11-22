import numpy as np
import time
import sys, os

controller_path = os.path.abspath(os.path.abspath(__file__) + "/../../../../")
sys.path.append(controller_path)
demo_path = os.path.abspath(os.path.abspath(__file__) + "/../../../")
sys.path.append(demo_path)

pykin_path = os.path.abspath(os.path.dirname(__file__) + "/../../../../" + "pykin")
sys.path.append(pykin_path)

from mj_controller.joint_pos import JointPositionController
import mjcf_parser as mp
from common import load_mujoco, load_pykin, get_result_qpos
from pykin.planners.cartesian_planner import CartesianPlanner
from pykin.collision.collision_manager import CollisionManager
from pykin.utils.transform_utils import change_to_pose
from pykin.utils.collision_utils import apply_robot_to_collision_manager

def main():
    sim, viewer = load_mujoco("../../../asset/panda_sim/franka_panda.xml")
    panda_robot = load_pykin('../../../pykin/asset/urdf/panda/panda.urdf')
    panda_robot.setup_link_name("panda_link0", "panda_link7")

    ####
    # model = mp.from_file(controller_path+"/asset/panda_sim/assets/panda_objects.xml", model_dir=controller_path+"/asset/common_objects/")
    # # model = mp.from_file('test/humanoid.xml')
    # print(model)
    # body = model.worldbody.body[0]
    # print(body.body[0].pos)
    
    # for geom in body.body[0].geom:
    #     print(geom.name)
    #     print(geom.pos)
    #     print(geom.mesh.name)
    ####

    init_qpos = np.array([0, 0.1963495375, 0.00, -2.616, 0.00, 2.9415926, 0.78539815])
    eef_pose = [0.65, 0.3464,  1.17,  -0.025,  0.778,  0.597, -0.193,]
    result_qpos = get_result_qpos(panda_robot, init_qpos, eef_pose)
    target_eef_pose = [0.65, 0.3464,  1.47,  -0.025,  0.778,  0.597, -0.193]
    target_eef_pose1 = [0.65, -0.3464,  1.47,  -0.025,  0.778,  0.597, -0.193]
    target_eef_pose2 = [0.65, -0.3464,  1.17,  -0.025,  0.778,  0.597, -0.193]

    jpos_controller = JointPositionController(sim=sim, eef_name=panda_robot.eef_name)
    jpos_controller.kp = 20
    # # print(sim.model.geom_names)

    task_plan = CartesianPlanner(
        panda_robot, 
        self_collision_manager=None,
        obstacle_collision_manager=None,
        n_step=1000,
        dimension=7)

    is_get_path = [False, True, True]

    if not is_get_path[0]:
        while True:
            torque = jpos_controller.run_controller(sim, result_qpos)
            sim.data.ctrl[jpos_controller.qpos_index] = torque
            sim.step()
            viewer.render()

            if jpos_controller.is_reached():
                print("reached")
                break

    if jpos_controller.is_reached() and not is_get_path[0]:
        jpos_controller.kp = 50
        joint_path, _ = task_plan.get_path_in_joinst_space(
            current_q=jpos_controller.joint_pos,
            goal_pose=target_eef_pose,
            resolution=0.001, 
            damping=0.03,
            pos_sensitivity=0.04,
            is_slerp=True)
        
        if joint_path is not None:
            is_get_path[0] = True
            is_get_path[1] = False

    if is_get_path[0]:        
        for i, joint in enumerate(joint_path):
            while True:
                torque = jpos_controller.run_controller(sim, joint)
                sim.data.ctrl[jpos_controller.qpos_index] = torque
                sim.step()
                viewer.render()

                if jpos_controller.is_reached():
                    print("reach")
                    print(f"{i+1}/{len(joint_path)}")
                    break

    if jpos_controller.is_reached() and not is_get_path[1]:
        joint_path, _ = task_plan.get_path_in_joinst_space(
            current_q=jpos_controller.joint_pos,
            goal_pose=target_eef_pose1,
            resolution=0.002, 
            damping=0.03,
            pos_sensitivity=0.04,
            is_slerp=True)
        
        if joint_path is not None:
            is_get_path[1] = True
            is_get_path[2] = False

    if is_get_path[1]:        
        for i, joint in enumerate(joint_path):
            while True:
                torque = jpos_controller.run_controller(sim, joint)
                sim.data.ctrl[jpos_controller.qpos_index] = torque
                sim.step()
                viewer.render()

                if jpos_controller.is_reached():
                    print("reach")
                    print(f"{i+1}/{len(joint_path)}")
                    break

    if jpos_controller.is_reached() and not is_get_path[2]:
        joint_path, _ = task_plan.get_path_in_joinst_space(
            current_q=jpos_controller.joint_pos,
            goal_pose=target_eef_pose2,
            resolution=0.001, 
            damping=0.03,
            pos_sensitivity=0.04,
            is_slerp=False)
        
        if joint_path is not None:
            is_get_path[2] = True

    if is_get_path[2]:        
        for i, joint in enumerate(joint_path):
            while True:
                torque = jpos_controller.run_controller(sim, joint)
                sim.data.ctrl[jpos_controller.qpos_index] = torque
                sim.step()
                viewer.render()

                if jpos_controller.is_reached():
                    print("reach")
                    print(f"{i+1}/{len(joint_path)}")
                    break

    last_qpos = joint_path[-1]
    while True:
        if jpos_controller.is_reached():
            torque = jpos_controller.run_controller(sim, last_qpos)
            sim.data.ctrl[jpos_controller.qpos_index] = torque

            sim.step()
            viewer.render()

if __name__ == "__main__":
    main()