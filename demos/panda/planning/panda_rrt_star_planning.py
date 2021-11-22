import numpy as np
import time
import sys, os

controller_path = os.path.abspath(os.path.abspath(__file__) + "/../../../../")
sys.path.append(controller_path)
demo_path = os.path.abspath(os.path.abspath(__file__) + "/../../../")
sys.path.append(demo_path)
pykin_path = os.path.abspath(os.path.dirname(__file__) + "/../../../" + "pykin")
sys.path.append(pykin_path)

from mj_controller.joint_pos import JointPositionController
from common import load_mujoco, load_pykin, get_result_qpos
from pykin.planners.rrt_star_planner import RRTStarPlanner
from pykin.collision.collision_manager import CollisionManager
from pykin.utils.transform_utils import change_to_pose
from pykin.utils.collision_utils import apply_robot_to_collision_manager
from pykin.utils.obstacle_utils import Obstacle
import trimesh


"""
mjGEOM_PLANE        = 0,        # plane
mjGEOM_HFIELD,                  # height field
mjGEOM_SPHERE,                  # sphere
mjGEOM_CAPSULE,                 # capsule
mjGEOM_ELLIPSOID,               # ellipsoid
mjGEOM_CYLINDER,                # cylinder
mjGEOM_BOX,                     # box
mjGEOM_MESH,                    # mesh
"""

def main():
    sim, viewer = load_mujoco("../../../asset/panda_sim/franka_panda.xml")
    panda_robot = load_pykin('../../../pykin/asset/urdf/panda/panda.urdf')
    panda_robot.setup_link_name("panda_link0", "panda_link7")

    init_qpos = np.array([0 , 0, 0, -1.5708, 0, 1.8675, 0])
    desired_qpos = np.array([np.pi/6, np.pi/6, 0.0, -np.pi*12/24, 0.0, np.pi*5/8,0.0])
    transformations = panda_robot.forward_kin(desired_qpos)
    eef_pose = transformations[panda_robot.eef_name].pose
    target_eef_pose = [0.5979,  -0.345249, 1.35337482, -0.00853688,  0.79165415,  0.60745875, -0.06484359]

    result_qpos = get_result_qpos(panda_robot, init_qpos, eef_pose)

    jpos_controller = JointPositionController(sim=sim, eef_name=panda_robot.eef_name)
    jpos_controller.kp = 50
    # jpos_controller.kd = 2

    mesh_path = pykin_path+"/asset/urdf/panda/"
    c_manager = CollisionManager(mesh_path)
    c_manager.filter_contact_names(panda_robot, panda_robot.forward_kin(jpos_controller.joint_pos))
    c_manager = apply_robot_to_collision_manager(c_manager, panda_robot, panda_robot.forward_kin(jpos_controller.joint_pos))

    o_manager = CollisionManager()

    # o_manager.add_object(
    #     "milk",
    #     gtype="mesh", 
    #     gparam=trimesh.load_mesh(controller_path+"/asset/common_objects/meshes/milk.stl"), 
    #     transform=sim.data.geom_xpos[sim.model.geom_name2id("milk")])

    obstacle_safety = np.array([0.05, 0.12, 0])
    o_manager.add_object(
        "1_frame",
        gtype="cylinder", 
        gparam=sim.model.geom_size[sim.model.geom_name2id("1_frame")] + obstacle_safety, 
        transform=sim.data.geom_xpos[sim.model.geom_name2id("1_frame")])

    print(sim.model.geom_size[sim.model.geom_name2id("1_frame")] + obstacle_safety)

    o_manager.add_object(
        "2_frame",
        gtype="cylinder", 
        gparam=sim.model.geom_size[sim.model.geom_name2id("2_frame")] + obstacle_safety, 
        transform=sim.data.geom_xpos[sim.model.geom_name2id("2_frame")])

    o_manager.add_object(
        "3_frame",
        gtype="cylinder", 
        gparam=sim.model.geom_size[sim.model.geom_name2id("3_frame")] + obstacle_safety, 
        transform=sim.data.geom_xpos[sim.model.geom_name2id("3_frame")])

    o_manager.add_object(
        "4_frame",
        gtype="cylinder", 
        gparam=sim.model.geom_size[sim.model.geom_name2id("4_frame")] + obstacle_safety, 
        transform=sim.data.geom_xpos[sim.model.geom_name2id("4_frame")])

    o_manager.add_object(
        "5_frame",
        gtype="cylinder", 
        gparam=sim.model.geom_size[sim.model.geom_name2id("5_frame")] + obstacle_safety, 
        transform=sim.data.geom_xpos[sim.model.geom_name2id("5_frame")])

    # result = c_manager.get_distances_other(o_manager)
    # print(result)


    # test, name, data = c_manager.in_collision_other(o_manager, True, True)
    # print(test, name)

    rrt_planner = RRTStarPlanner(
        robot=panda_robot,
        self_collision_manager=c_manager,
        obstacle_collision_manager=o_manager,
        delta_distance=0.3,
        epsilon=0.2, 
        max_iter=1000,
        gamma_RRT_star=10,
        dimension=7
    )
    # print(sim.data.geom_xpos[sim.model.geom_name2id("r_frame")])
    # print(sim.data.geom_xpos[sim.model.geom_name2id("c_frame")])
    # print(sim.data.geom_xpos[sim.model.geom_name2id("l_frame")])
    # print(sim.model.geom_type[sim.model.geom_name2id("r_frame")])
    # print(sim.model.geom_names[sim.model.geom_name2id("r_frame")])
    # print(sim.model.geom_size[sim.model.geom_name2id("r_frame")])

    is_get_path = False
    if not is_get_path:
        for _ in range(20000):
            # print(result_qpos)
            torque = jpos_controller.run_controller(sim, result_qpos)
            sim.data.ctrl[jpos_controller.qpos_index] = torque
            # print(jpos_controller.time_step)
            sim.step()
            viewer.render()

            if jpos_controller.is_reached():
                print("reached")
                break

    if not is_get_path:
        joint_path, is_get_path = rrt_planner.get_path_in_joinst_space(
            cur_q=jpos_controller.joint_pos, 
            goal_pose=target_eef_pose,
            resolution=1)
        
    if is_get_path:
        for i, joint in enumerate(joint_path):
            while True:
                # print(jpos_controller.time_step)
                print(f"{i}/{len(joint_path)}")
                jpos_controller.kp = 40
                # jpos_controller.ki = 0.2
                # jpos_controller.kd = 10.22
                torque = jpos_controller.run_controller(sim, joint)
                print(sim.data.body_xpos[sim.model.body_name2id("panda_leftfinger")], panda_robot.forward_kin(jpos_controller.q_pos)["panda_leftfinger"].pos)
                
                if jpos_controller.is_reached():
                    print("reach")
                    print(f"{i+1}/{len(joint_path)}")
                    break
                
                # print(jpos_controller.err_qpos)
                sim.data.ctrl[jpos_controller.qpos_index] = torque
                sim.step()
                viewer.render()
    last_qpos = joint_path[-1]
    while True:
        if jpos_controller.is_reached():
            torque = jpos_controller.run_controller(sim, last_qpos)
            sim.data.ctrl[jpos_controller.qpos_index] = torque

            sim.step()
            viewer.render()

if __name__ == "__main__":
    main()