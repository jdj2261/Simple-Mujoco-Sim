from robosuite.models import MujocoWorldBase
from robosuite.models.robots import Panda
from robosuite.models.grippers import gripper_factory, RethinkGripper
from robosuite.models.arenas import BinsArena, TableArena, WipeArena
from robosuite.models.objects import BallObject, BoxObject
from mujoco_py import MjSim, MjViewer

world = MujocoWorldBase()

mujoco_robot = Panda()

gripper = gripper_factory('PandaGripper')
mujoco_robot.add_gripper(gripper)
mujoco_robot.set_base_xpos([0, 0, 0])
world.merge(mujoco_robot)

mujoco_arena = BinsArena()
mujoco_arena.set_origin([0.8, 0, 0])
world.merge(mujoco_arena)

ball_obj = BallObject(
    name="ball",
    size=[0.04],
    rgba=[0, 0.5, 0.5, 1]).get_obj()

box_obj = BoxObject(
    name="box",
    size_min=[0.015, 0.015, 0.015],  # [0.015, 0.015, 0.015],
    size_max=[0.022, 0.022, 0.022],  # [0.018, 0.018, 0.018])
    rgba=[0, 0.5, 0.5, 1]).get_obj()

ball_obj.set('pos', '1.0 0 1.0')
world.worldbody.append(ball_obj)

box_obj.set('pos', '0.8 0 1.0')
world.worldbody.append(box_obj)

model = world.get_model(mode="mujoco_py")

sim = MjSim(model)
viewer = MjViewer(sim)
viewer.vopt.geomgroup[0] = 0  # disable visualization of collision mesh

for i in range(10000):
    sim.data.ctrl[:] = 0
    sim.step()
    viewer.render()


# import xml.etree.ElementTree as ET
# from robosuite.utils.mjcf_utils import new_joint, new_actuator

# # add a gripper
# gripper = RethinkGripper()
# # Create another body with a slider joint to which we'll add this gripper
# gripper_body = ET.Element("body", name="gripper_base")
# gripper_body.set("pos", "0 0 0.3")
# gripper_body.set("quat", "0 0 1 0")  # flip z
# gripper_body.append(
#     new_joint(name="gripper_z_joint", type="slide",
#               axis="0 0 1", damping="50")
# )

# # print(ET.tostring(gripper_body).decode())
# # Add the dummy body with the joint to the global worldbody
# world.worldbody.append(gripper_body)

# # Merge the actual gripper as a child of the dummy body
# world.merge(gripper, merge_body="gripper_base")
# # Create a new actuator to control our slider joint
# world.actuator.append(
#     new_actuator(
#         joint="gripper_z_joint", act_type="position", name="gripper_z", kp="500"
#     )
# )


# mujoco_arena = PegsArena(table_full_size=(0.8, 0.8, 0.05),
#                          table_friction=(1, 0.005, 0.0001),
#                          table_offset=np.array((0, 0, 0.82)))
