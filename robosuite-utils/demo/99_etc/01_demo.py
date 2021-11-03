import numpy as np
import robosuite as suite

# create environment instance
env = suite.make(
    env_name="Door",  # try with other tasks like "Stack" and "Door"
    robots="Jaco",  # try with other robots like "Sawyer" and "Jaco"
    has_renderer=True,
    has_offscreen_renderer=False,
    use_camera_obs=False,
    horizon=1000,
)

# reset the environment
env.reset()

step = 0.1

for i in range(env.horizon):

    if i < 200:
        action = np.array([0, 0, step, 0, 0, 0, 0,0])
    elif 200 < i < 600:
        action = np.array([0, step, 0, 0, 0, 0, 0,0])
    else:
        action = np.array([0, 0, step, 0, 0, 0, 0,0])

    # print(i, env.robots[0]._joint_positions)
    # action = [step, 0, 0, 0, 0, 0, 0]  # sample random action
    # take action in the environment
    obs, reward, done, info = env.step(action)
    print(obs, reward, done, info)
    env.render()  # render on display
