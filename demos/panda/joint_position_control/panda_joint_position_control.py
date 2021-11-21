import numpy as np
import time
import sys, os

controller_path = os.path.abspath(os.path.abspath(__file__) + "/../../../../")
sys.path.append(controller_path)
demo_path = os.path.abspath(os.path.abspath(__file__) + "/../../../")
sys.path.append(demo_path)

from mj_controller.joint_pos import JointPositionController
from common import load_mujoco, load_pykin, get_result_qpos

def main():
    # sim, viewer = load_mujoco("../../asset/franka_sim/assets/franka_chain_test.xml")
    sim, viewer = load_mujoco("../../../asset/panda_sim/franka_panda.xml")
    panda_robot = load_pykin('../../../pykin/asset/urdf/panda/panda.urdf')
    panda_robot.setup_link_name("panda_link0", "panda_link7")

    init_qpos = np.array([0 , 0, 0, -1.5708, 0, 1.8675, 0])
    desired_qpos = np.array([0.0, np.pi/6, 0.0, -np.pi*12/24, 0.0, np.pi*5/8,0.0])
    transformations = panda_robot.forward_kin(desired_qpos)
    eef_pose = transformations[panda_robot.eef_name].pose
    print(eef_pose)
    result_qpos = get_result_qpos(panda_robot, init_qpos, eef_pose)

    jpos_controller = JointPositionController(sim=sim, eef_name=panda_robot.eef_name)
    jpos_controller.kp = 20
    # print(sim.model.geom_names)

    cnt = 0
    is_grasp = False

    while True:
        torque = jpos_controller.run_controller(sim, result_qpos)
        sim.data.ctrl[jpos_controller.qpos_index] = torque
        sim.data.ctrl[jpos_controller.gripper_index] = [0.4, 0.4]

        # for contact in sim.data.contact:
        #     # geom_name1 = sim.model.geom_id2name(contact.geom1)
        #     # geom_name2 = sim.model.geom_id2name(contact.geom2)
            
        #     # if geom_name1 == "ground" and geom_name2=="ground":
        #     #     continue
        #     # print(geom_name1, geom_name2)

        # print(sim.data.geom_xpos[sim.model.geom_name2id("coke")])
        print(f"Current : {jpos_controller.eef_pos}")
        print(f"Robot : {panda_robot.forward_kin(jpos_controller.q_pos)[panda_robot.eef_name].pos}")
        # print(np.round(jpos_controller.err_qpos, 4))

        if not is_grasp:
            sim.data.ctrl[jpos_controller.gripper_index] = [0.4, 0.4]
        
        if jpos_controller.is_reached():
            cnt += 1
            print(f"End Effector Position : {np.round(jpos_controller.eef_pos,4)}")
            print("Goal reached")
            time.sleep(1)
            sim.data.ctrl[jpos_controller.gripper_index] = [0.0, 0.0]
            is_grasp = True
            if cnt%4 == 1:
                result_qpos[0] += np.pi/2
            elif cnt%4 == 2:
                result_qpos[1] -= np.pi/2
            elif cnt%4 == 3:
                result_qpos[0] -= np.pi/2
            else:
                result_qpos[1] += np.pi/2

        sim.step()
        viewer.render()

if __name__ == "__main__":
    main()