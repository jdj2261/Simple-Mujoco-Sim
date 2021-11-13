import numpy as np
import time

from mj_controller.joint_pos import JointPositionController
from common_func import load_mujoco, load_pykin, get_result_qpos


def main():
    sim, viewer = load_mujoco()
    panda_robot = load_pykin()

    init_qpos = np.array([0 , 0, 0, -1.5708, 0, 1.8675, 0])
    desired_qpos = np.array([0, np.pi/6, 0.0, -np.pi/2, 0.0, np.pi*5/8,0.0])
    transformations = panda_robot.forward_kin(desired_qpos)
    eef_pose = transformations[panda_robot.eef_name].pose

    result_qpos = get_result_qpos(panda_robot, init_qpos, eef_pose)

    jpos_controller = JointPositionController(sim=sim, eef_name=panda_robot.eef_name)
    
    cnt = 0
    while True:
        torque = jpos_controller.run_controller(sim, result_qpos)
        sim.data.ctrl[jpos_controller.qpos_index] = torque

        if jpos_controller.is_reached():
            cnt += 1
            print(f"End Effector Position : {np.round(jpos_controller.eef_pos,4)}")
            print("Goal reached")
            time.sleep(1)
            if cnt%4 == 1:
                result_qpos[0] += np.pi/2
            elif cnt%4 == 2:
                result_qpos[1] -= np.pi/2
            elif cnt%4 == 3:
                result_qpos[0] -= np.pi/2
            else:
                result_qpos[1] += np.pi/2
        # if not is_reached:
        #     torque = jpos_controller.run_controller(sim, result_qpos)
        #     sim.data.ctrl[jpos_controller.qpos_index] = torque
        #     sim.data.ctrl[jpos_controller.gripper_index] = [0.4, 0.4]
            
        #     if jpos_controller.is_reached():
        #         is_reached = True
        #         print("is_reached")

        #     print(np.round(jpos_controller.err_qpos,4), np.round(sim.data.time,4))
        
        # if is_reached:
        #     torque = jpos_controller.run_controller(sim, [0, 0, 0, -1.5708, 0, 1.8675, 0])
        #     sim.data.ctrl[jpos_controller.qpos_index] = torque
        #     sim.data.ctrl[jpos_controller.gripper_index] = [0.0, 0.0]

        #     print(jpos_controller.is_reached())
        #     print(jpos_controller.err_qpos)

        #     if jpos_controller.is_reached():
        #         is_reached = False
        #         print("is_reached")

        sim.step()
        viewer.render()

if __name__ == "__main__":
    main()