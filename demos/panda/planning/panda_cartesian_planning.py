import numpy as np
import os, sys
import trimesh
# import mjcf_parser as mp

panda_dir = os.path.dirname(os.getcwd())
parent_path = panda_dir + "/../../"
sys.path.append(parent_path)
from controllers.joint_pos import JointPositionController

from demos.common import load_mujoco, load_pykin, get_result_qpos
from pykin.planners.cartesian_planner import CartesianPlanner
from pykin.collision.collision_manager import CollisionManager
from pykin.utils.collision_utils import apply_robot_to_collision_manager
from pykin.kinematics.transform import Transform

def main():
    sim, viewer = load_mujoco(parent_path + "asset/panda_sim/franka_panda.xml")
    panda_robot = load_pykin(parent_path + 'pykin/asset/urdf/panda/panda.urdf')
    panda_robot.setup_link_name("panda_link_0", "panda_right_hand")

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
    target_eef_pose = [0.65, 0.3464,  1.37,  -0.025,  0.778,  0.597, -0.193]
    target_eef_pose1 = [0.65, -0.3464,  1.37,  -0.025,  0.778,  0.597, -0.193]
    target_eef_pose2 = [0.65, -0.3464,  1.07,  -0.025,  0.778,  0.597, -0.193]

    jpos_controller = JointPositionController(sim=sim, eef_name=panda_robot.eef_name)
    jpos_controller.kp = 20
    # # print(sim.model.geom_names)


    mesh_path = parent_path+"pykin/asset/urdf/panda/"
    milk_path = parent_path+"pykin/asset/objects/meshes/milk.stl"
    milk_mesh = trimesh.load_mesh(milk_path)
    c_manager = CollisionManager(mesh_path)
    c_manager.filter_contact_names(panda_robot, panda_robot.forward_kin(jpos_controller.q_pos))
    c_manager = apply_robot_to_collision_manager(c_manager, panda_robot, panda_robot.forward_kin(jpos_controller.q_pos))

    o_manager = CollisionManager()
    o_manager.add_object("milk", gtype="mesh", gparam=milk_mesh, transform=sim.data.geom_xpos[sim.model.geom_name2id("milk")])
    print(sim.data.geom_xpos[sim.model.geom_name2id("milk")])
    
    # obstacle_safety = 1.0

    # for i in range(1,6):
    #     name = str(i)+"_frame"
    #     o_manager.add_object(        
    #     name=name,
    #     gtype="cylinder", 
    #     gparam=sim.model.geom_size[sim.model.geom_name2id(name)] * obstacle_safety, 
    #     transform=sim.data.geom_xpos[sim.model.geom_name2id(name)])
    #     print(sim.model.geom_size[sim.model.geom_name2id(name)]*obstacle_safety)

    task_plan = CartesianPlanner(
        panda_robot, 
        self_collision_manager=c_manager,
        obstacle_collision_manager=o_manager,
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
            current_q=jpos_controller.q_pos,
            goal_pose=target_eef_pose,
            resolution=0.01, 
            damping=0.03,
            pos_sensitivity=0.04,
            is_slerp=False)
        
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
            current_q=jpos_controller.q_pos,
            goal_pose=target_eef_pose1,
            resolution=0.01, 
            damping=0.03,
            pos_sensitivity=0.04,
            is_slerp=False)
        
        if joint_path is not None:
            is_get_path[1] = True
            is_get_path[2] = False

    if is_get_path[1]:        
        for i, joint in enumerate(joint_path):
            while True:
                torque = jpos_controller.run_controller(sim, joint)
                sim.data.ctrl[jpos_controller.qpos_index] = torque
                cur_right_finger = sim.data.body_xpos[sim.model.body_name2id("panda_finger_joint2_tip")]
                robot_right_finger = panda_robot.forward_kin(jpos_controller.q_pos)["panda_finger_joint2_tip"].pos
                print(f"current : {cur_right_finger}")
                print(f"robot : {robot_right_finger}")
                # print(sim.model.geom_size[sim.model.geom_name2id(name)] * obstacle_safety
                #      +sim.data.geom_xpos[sim.model.geom_name2id(name)][2])
                sim.step()
                viewer.render()

                if jpos_controller.is_reached():
                    print("reach")
                    print(f"{i+1}/{len(joint_path)}")
                    break

    if jpos_controller.is_reached() and not is_get_path[2]:
        joint_path, _ = task_plan.get_path_in_joinst_space(
            current_q=jpos_controller.q_pos,
            goal_pose=target_eef_pose2,
            resolution=0.01, 
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