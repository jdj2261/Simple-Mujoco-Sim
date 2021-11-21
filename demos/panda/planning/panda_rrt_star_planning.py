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
    eef_pose = change_to_pose([0.426, -0.3, 1.215,  3.14079280e+00, -6.52698077e-01,  2.20425796e-06])
    target_eef_pose = change_to_pose([0.726, 0.5, 1.215,  3.14079280e+00, -6.52698077e-01,  -2.20425796e-06])

    result_qpos = get_result_qpos(panda_robot, init_qpos, eef_pose)

    jpos_controller = JointPositionController(sim=sim, eef_name=panda_robot.eef_name)
    jpos_controller.kp = 30
    jpos_controller.kd = 10


    mesh_path = pykin_path+"/asset/urdf/panda/"
    c_manager = CollisionManager(mesh_path)
    c_manager.filter_contact_names(panda_robot, panda_robot.forward_kin(jpos_controller.joint_pos))
    c_manager = apply_robot_to_collision_manager(c_manager, panda_robot, panda_robot.forward_kin(jpos_controller.joint_pos))

    result, objs_in_collision, contact_data = c_manager.in_collision_internal(return_names=True, return_data=True)

    o_manager = CollisionManager()

    # o_manager.add_object(
    #     "milk",
    #     gtype="mesh", 
    #     gparam=trimesh.load_mesh(controller_path+"/asset/common_objects/meshes/milk.stl"), 
    #     transform=sim.data.geom_xpos[sim.model.geom_name2id("milk")])

    # o_manager.add_object(
    #     "milk1",
    #     gtype="mesh", 
    #     gparam=trimesh.load_mesh(controller_path+"/asset/common_objects/meshes/milk.stl"), 
    #     transform=sim.data.geom_xpos[sim.model.geom_name2id("milk1")])

    # o_manager.add_object(
    #     "milk2",
    #     gtype="mesh", 
    #     gparam=trimesh.load_mesh(controller_path+"/asset/common_objects/meshes/milk.stl"), 
    #     transform=sim.data.geom_xpos[sim.model.geom_name2id("milk")])

    # o_manager.add_object(
    #     "milk",
    #     gtype="mesh", 
    #     gparam=trimesh.load_mesh(controller_path+"/asset/common_objects/meshes/milk.stl"), 
    #     transform=sim.data.geom_xpos[sim.model.geom_name2id("milk")])

    # o_manager.add_object(
    #     "milk",
    #     gtype="mesh", 
    #     gparam=trimesh.load_mesh(controller_path+"/asset/common_objects/meshes/milk.stl"), 
    #     transform=sim.data.geom_xpos[sim.model.geom_name2id("milk")])


    o_manager.add_object(
        "l_frame",
        gtype="cylinder", 
        gparam=sim.model.geom_size[sim.model.geom_name2id("l_frame")], 
        transform=sim.data.geom_xpos[sim.model.geom_name2id("l_frame")])

    o_manager.add_object(
        "c_frame",
        gtype="cylinder", 
        gparam=sim.model.geom_size[sim.model.geom_name2id("c_frame")], 
        transform=sim.data.geom_xpos[sim.model.geom_name2id("c_frame")])

    o_manager.add_object(
        "r_frame",
        gtype="cylinder", 
        gparam=sim.model.geom_size[sim.model.geom_name2id("r_frame")], 
        transform=sim.data.geom_xpos[sim.model.geom_name2id("r_frame")])


    # result = c_manager.get_distances_other(o_manager)
    # print(result)


    # test, name, data = c_manager.in_collision_other(o_manager, True, True)
    # print(test, name)

    rrt_planner = RRTStarPlanner(
        robot=panda_robot,
        self_collision_manager=c_manager,
        obstacle_collision_manager=o_manager,
        delta_distance=0.2,
        epsilon=0.2, 
        max_iter=2000,
        gamma_RRT_star=1,
        dimension=7
    )
    # print(sim.data.geom_xpos[sim.model.geom_name2id("r_frame")])
    # print(sim.data.geom_xpos[sim.model.geom_name2id("c_frame")])
    # print(sim.data.geom_xpos[sim.model.geom_name2id("l_frame")])
    # print(sim.model.geom_type[sim.model.geom_name2id("r_frame")])
    # print(sim.model.geom_names[sim.model.geom_name2id("r_frame")])
    # print(sim.model.geom_size[sim.model.geom_name2id("r_frame")])

    # joint_path, is_get_path = rrt_planner.get_path_in_joinst_space(
    #     cur_q=jpos_controller.joint_pos, 
    #     goal_pose=target_eef_pose)

    # print(joint_path)

    # for i in range(len(joint_path)):
    #     print(joint_path[i])
    #     for _ in range(60000):
    #         torque = jpos_controller.run_controller(sim, joint_path[i])
    #         if jpos_controller.is_reached():
    #             break
    #         sim.data.ctrl[jpos_controller.qpos_index] = torque
    #         sim.step()
    #         viewer.render()
    is_get_path = False
    if not is_get_path:
        for _ in range(20000):
            print(result_qpos)
            torque = jpos_controller.run_controller(sim, result_qpos)
            sim.data.ctrl[jpos_controller.qpos_index] = torque
            sim.step()
            viewer.render()

            if jpos_controller.is_reached():
                break

    if not is_get_path:
        joint_path, is_get_path = rrt_planner.get_path_in_joinst_space(
            cur_q=jpos_controller.joint_pos, 
            goal_pose=target_eef_pose)
        
    if is_get_path:        
        for i, joint in enumerate(joint_path):
            while True:
                print(f"{i}/{len(joint_path)}")
                torque = jpos_controller.run_controller(sim, joint)

                if jpos_controller.is_reached():
                    print("reach")
                    print()
                    print()
                    break
                
                # print(jpos_controller.err_qpos)
                sim.data.ctrl[jpos_controller.qpos_index] = torque
                sim.step()
                viewer.render()
        last_qpos = joint_path[-1]
    
    # # if is_get_path:
    #     torque = jpos_controller.run_controller(sim, last_qpos)
    #     sim.data.ctrl[jpos_controller.qpos_index] = torque

    #     sim.step()
    #     viewer.render()

if __name__ == "__main__":
    main()