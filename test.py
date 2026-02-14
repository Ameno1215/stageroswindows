from denso_http_client import DensoRobotClient

robot = DensoRobotClient("http://localhost:8000")

print(robot.health())

# 1) Init
print(robot.init_robot(model="vs060", planning_group="arm", velocity_scale=0.1, accel_scale=0.1))

# 2) Modifier vitesse/accel
print(robot.set_scaling(0.5, 0.5))

# 3) Move joint
print(robot.goto_joint([1.57, 0.0, 1.57, 0.0, 1.57, 0.0], execute=True))

# 4) Move pose
print(robot.goto_pose(
    frame_id="base_link",
    x=-0.10, y=0.40, z=0.40,
    qx=0.7071068, qy=-0.7071068, qz=0.0, qw=0.0,
    execute=True
))
