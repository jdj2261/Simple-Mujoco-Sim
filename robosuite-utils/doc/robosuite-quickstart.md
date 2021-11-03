# Quick Start

**간단한 예제 실행해보기**

[Quick Start](https://robosuite.ai/docs/quickstart.html), [Demo Showcases](https://robosuite.ai/docs/demos.html) 참조

- 기본 환경 실행

  ~~~
  $ vi demo_quick_start.py
  ~~~

  아래 내용 복사

  ~~~python
  import numpy as np
  import robosuite as suite
  
  # create environment instance
  env = suite.make(
      env_name="Lift", # try with other tasks like "Stack" and "Door"
      robots="Panda",  # try with other robots like "Sawyer" and "Jaco"
      has_renderer=True,
      has_offscreen_renderer=False,
      use_camera_obs=False,
  )
  
  # reset the environment
  env.reset()
  
  for i in range(1000):
      action = np.random.randn(env.robots[0].dof) # sample random action
      obs, reward, done, info = env.step(action)  # take action in the environment
      env.render()  # render on display
  ~~~

  - 설명
    1. make 함수로 시뮬레이션 환경을 만듭니다.
    2. 환경을 리셋시킵니다.
    3. numpy.array 타입의 action을 랜덤하게 생성 후 동작 시킵니다.
    4. 이 작업을 1000번 반복합니다.


- 커스텀 환경 만들기

  ~~~
  $ vi demo_custom_env.py
  ~~~

  아래 내용 복사

  ~~~python
  from robosuite.models import MujocoWorldBase
  from robosuite.models.robots import Panda
  from robosuite.models.grippers import gripper_factory
  from robosuite.models.arenas import TableArena
  from robosuite.models.objects import BallObject, BoxObject
  from mujoco_py import MjSim, MjViewer
  
  # Creating the world
  world = MujocoWorldBase()
  
  # Creating the robot.
  mujoco_robot = Panda()
  
  # add a gripper
  gripper = gripper_factory('PandaGripper')
  mujoco_robot.add_gripper(gripper)
  mujoco_robot.set_base_xpos([0, 0, 0])
  world.merge(mujoco_robot)
  
  # add arena
  mujoco_arena = TableArena()
  mujoco_arena.set_origin([0.8, 0, 0])
  world.merge(mujoco_arena)
  
  # add ball object
  ball_obj = BallObject(
      name="ball",
      size=[0.04],
      rgba=[0, 0.5, 0.5, 1]).get_obj()
  ball_obj.set('pos', '1.0 0 1.0')
  
  # add box object
  box_obj = BoxObject(
      name="box",
      size_min=[0.015, 0.015, 0.015],  # [0.015, 0.015, 0.015],
      size_max=[0.022, 0.022, 0.022],  # [0.018, 0.018, 0.018])
      rgba=[0, 0.5, 0.5, 1]).get_obj()
  box_obj.set('pos', '0.5 0 1.0')
  
  world.worldbody.append(ball_obj)
  world.worldbody.append(box_obj)
  
  # running simulation 
  model = world.get_model(mode="mujoco_py")
  
  sim = MjSim(model)
  viewer = MjViewer(sim)
  viewer.vopt.geomgroup[0] = 0  # disable visualization of collision mesh
  
  for i in range(10000):
      sim.data.ctrl[:] = 0
      sim.step()
      viewer.render()
  ~~~


---

<div style="text-align: right">next 👉  <a href="robotics-transformation.md"> transformation</a> </div>

---
