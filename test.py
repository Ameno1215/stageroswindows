from denso_http_client import DensoRobotClient

robot = DensoRobotClient("http://localhost:8000")

print(robot.health())

# Init
print(robot.init_robot(model="vs060", planning_group="arm", velocity_scale=0.1, accel_scale=0.1))

# Modify vitesse/accel
print(robot.set_scaling(1, 1))


# print(robot.goto_joint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], execute=True))


# print(robot.get_joint_state())

# Move joint
# print(robot.goto_joint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], execute=True))
# print(robot.goto_joint([1.57, 0.0, 1.57, 0.0, 1.57, 0.0], execute=False))
# print(robot.get_joint_state())

# Move pose
print(robot.goto_pose(
    frame_id="base_link",
    x=-0.10, y=0.40, z=0.3,
    qx=0.7071068, qy=-0.7071068, qz=0.0, qw=0.0,
    execute=True
))

print(robot.get_current_pose(output_format="both"))


print(robot.goto_euler_world(x=-0.10, y=0.40, z=0.3, rx=-3.1408355429921593, ry=-0.0008065712482139904, rz=-1.5698598943251731))

print(robot.get_current_pose(output_format="both"))

# print(robot.move_relative_world(dx=0, dy=0.3, dz=0.0, drx=0, dry=0, drz=0, execute=True))


# print(robot.goto_euler_world(x=0.5, y=0.0, z=0.4, rx=3.14/2, ry=0, rz=0.0))
# print(robot.goto_euler_world(x=0.5, y=0.0, z=0.4, rx=3.14/2, ry=0, rz=3.14/4))

# print(robot.move_relative_tool(dx=0, dy=0, dz=0.1, drx=0, dry=0, drz=0))

# print(robot.move_relative_tool(dx=0, dy=0, dz=0, drx=0, dry=0, drz=1.57))

# print(robot.get_current_pose(output_format="quaternion")) 
# print(robot.get_current_pose(child_frame_id="J6", output_format="quaternion"))
# print(robot.get_current_pose(frame_id="J3", child_frame_id="J5", output_format="quaternion"))



# print(robot.goto_joint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], execute=True))



