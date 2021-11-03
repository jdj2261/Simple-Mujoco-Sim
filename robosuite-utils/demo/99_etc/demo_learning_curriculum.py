import os

import robosuite
from robosuite import make
from robosuite.wrappers import DemoSamplerWrapper

# TODO: Demonstrations path is now depreceated. Need to update and/or get new demonstrations!!



env = make(
    "PickPlace",
    robots="Sawyer",
    has_renderer=True,
    has_offscreen_renderer=False,
    ignore_done=True,
    use_camera_obs=False,
    reward_shaping=True,
    gripper_visualizations=True,
)

env = DemoSamplerWrapper(
    env,
    demo_path=os.path.join(
        robosuite.models.assets_root, "demonstrations/SawyerPickPlace"
    ),
    need_xml=True,
    num_traj=-1,
    sampling_schemes=["uniform", "random"],
    scheme_ratios=[0.9, 0.1],
)

for _ in range(100):
    env.reset()
    env.viewer.set_camera(0)
    env.render()
    for i in range(100):
        if i == 0:
            reward = env.reward()
            print("reward", reward)
        env.render()