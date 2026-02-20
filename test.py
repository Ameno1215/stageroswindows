import time
from denso_http_client import DensoRobotClient

# Initialisation du client
robot = DensoRobotClient("http://localhost:8000")

print("Health:", robot.health())
print("Init:", robot.init_robot(model="vs060", planning_group="arm", velocity_scale=0.2, accel_scale=0.2))
print("Scaling:", robot.set_scaling(velocity_scale=1, accel_scale=1))



print(robot.move_joints([0.0, 0.0, 1.57, 0.0, 1.57, 0.0], is_relative=False))



print(robot.move_to_pose(
    x=0.435, y=0.0488, z=0.0800, 
    r1=-3.14, r2=0.83, r3=-2.5,
    rotation_format="RPY", 
    reference_frame="WORLD",
    cartesian_path=True
))

carrer_points = [
    {"x": 0.0, "y": 0.0,  "z": -0.1, "r1": 0.0, "r2": 0.0, "r3": 0.0},
    {"x": 0.0,  "y": 0.0, "z": 0.1, "r1": 0.0, "r2": 0.0, "r3": 0.0}, 
]

print(robot.move_waypoints(
    waypoints=carrer_points,
    rotation_format="RPY",
    reference_frame="TOOL", 
    is_relative=True, 
    cartesian_path=True
))


# print(robot.move_to_pose(
#     x=0.3, y=0.3, z=0.10, 
#     r1=0.0, r2=0.0, r3=0.0, 
#     rotation_format="RPY", 
#     reference_frame="WORLD", 
#     is_relative=True, 
#     cartesian_path=True 
# ))

# print(robot.move_to_pose(
#     x=0, y=0, z=0.3, 
#     r1=0.0, r2=0.0, r3=0.0, 
#     rotation_format="RPY", 
#     reference_frame="TOOL", 
#     is_relative=True, 
#     cartesian_path=True,
#     execute=True
# ))
# time.sleep(1)

"""
carrer_points = [
    {"x": 0.10, "y": 0.0,  "z": 0.0, "r1": 0.0, "r2": 0.0, "r3": 0.0}, # Avance X
    {"x": 0.0,  "y": 0.10, "z": 0.0, "r1": 0.0, "r2": 0.0, "r3": 0.0}, # Décale Y
    {"x": -0.10,"y": 0.0,  "z": 0.0, "r1": 0.0, "r2": 0.0, "r3": 0.0}, # Recule X
    {"x": 0.0,  "y": -0.10,"z": 0.0, "r1": 0.0, "r2": 0.0, "r3": 0.0}, # Retour Y
]

print(robot.move_waypoints(
    waypoints=carrer_points,
    rotation_format="RPY",
    reference_frame="TOOL", 
    is_relative=True, 
    cartesian_path=True  
))
"""


print("Joints :", robot.get_joint_state())
print("Pose (Euler) :", robot.get_current_pose(output_format="euler"))