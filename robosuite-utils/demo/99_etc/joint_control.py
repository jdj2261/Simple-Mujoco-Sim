import numpy as np
import robosuite as suite


def relative2absolute_joint_pos_commands(goal_joint_pos, robot, kp, kd):
    assert len(goal_joint_pos) == robot.dof

    action = [0 for _ in range(robot.dof)]
    curr_joint_pos = robot._joint_positions
    curr_joint_vel = robot._joint_velocities

    for i in range(robot.dof):
        action[i] = (goal_joint_pos[i] - curr_joint_pos[i]) * kp - curr_joint_vel[
            i
        ] * kd

    return action


def robosuite_simulation_controller_test(env, sim_time):
    # Reset the env
    env.reset()

    robot = env.robots[0]

    kp = 2
    kd = 1.2

    for t in range(sim_time):
        if env.done:
            break
        env.render()

        action = relative2absolute_joint_pos_commands(
            [np.pi / 2, 0, 0, 0, 0, 0], robot, kp, kd
        )

        if t > 1200:
            action = relative2absolute_joint_pos_commands(
                [0, -np.pi / 4, 0, 0, 0, 0], robot, kp, kd
            )
        elif t > 800:
            action = relative2absolute_joint_pos_commands(
                [0, 0, 0, 0, 0, 0], robot, kp, kd
            )
        elif t > 400:
            action = relative2absolute_joint_pos_commands(
                [3 * np.pi / 2, 0, 0, 0, 0, 0], robot, kp, kd
            )

        observation, reward, done, info = env.step(action)

    # close window
    env.close()


env = suite.make(
    "Lift",
    robots="UR5e",
    gripper_types=None,
    has_renderer=True,
    has_offscreen_renderer=False,
    use_camera_obs=False,
    use_object_obs=False,
    control_freq=50,
    render_camera=None,
    horizon=2000,
)

robosuite_simulation_controller_test(env, env.horizon)
