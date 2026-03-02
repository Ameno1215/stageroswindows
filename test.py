import math
import time
from motion_http_client import MotionRobotClient
from math import pi
from plate import load_plate_from_file

from pathlib import Path
from plate import load_plate_from_file







def run():
    robot = MotionRobotClient("http://localhost:8000")

    print("Health:", robot.health())
    robot.init_robot(model="vs060", planning_group="arm", velocity_scale=0.2, accel_scale=0.2)
    robot.set_scaling(velocity_scale=1, accel_scale=1)

    robot.move_joints(joints=[0, -pi/4, 0, -pi/2, 0], is_relative=False)

    robot.move_joints(joints=[0, 0, pi/2, pi/2, 0], is_relative=False)

    robot.move_to_pose(
        x=0.4, y=0.0, z=0.4,
        r1=0.0, r2=pi/2, r3=0.0,
        rotation_format="RPY",
        reference_frame="WORLD",
        execute=True,
        cartesian_path=True
        )





if __name__ == "__main__":
    run()











# print(robot.move_to_pose(
#     x=0.435, y=0.0488, z=0.0800, 
#     r1=-3.14, r2=0.83, r3=-2.5,
#     rotation_format="RPY", 
#     reference_frame="WORLD",
#     cartesian_path=False
# ))

# carrer_points = [
#     # {"x": 0.4, "y": 0.3,  "z": 0.2, "r1": 3.14, "r2": 0.0, "r3": -1.57},
#     # {"x": 0.2,  "y": 0.3, "z": 0.1, "r1": 3.14, "r2": 0.0, "r3": -1.57}, 
#     # {"x": 0.2,  "y": -0.3, "z": 0.3, "r1": 3.14, "r2": 0.0, "r3": -1.57}, 
#     # {"x": 0.4, "y": 0.3,  "z": 0.2, "r1": 3.14, "r2": 0.0, "r3": -1.57},
#     { "x": 0.0, "y": 0.0, "z": -0.2, "r1": 0, "r2": 0, "r3": 0, "is_relative": True, "reference_frame": "TOOL" },
#     { "x": 0.0, "y": 0, "z": 0.2, "r1": 0, "r2": 0, "r3": 0, "is_relative": True, "reference_frame": "WORLD" },
# ]

# robot.move_waypoints(
#     waypoints=carrer_points,
#     rotation_format="RPY",
#     reference_frame="TOOL", 
#     is_relative=False, 
#     cartesian_path=True
# )


# print(robot.move_to_pose(
#     x=0.4, y=0.3, z=0.2, 
#     r1=3.14, r2=0.0, r3=-1.57, 
#     rotation_format="RPY", 
#     reference_frame="WORLD", 
#     is_relative=False, 
#     cartesian_path=True 
# ))
# print(robot.move_to_pose(
#     x=0.2, y=0.3, z=0.1, 
#     r1=3.14, r2=0.0, r3=-1.57, 
#     rotation_format="RPY", 
#     reference_frame="WORLD", 
#     is_relative=False, 
#     cartesian_path=True 
# ))
# print(robot.move_to_pose(
#     x=0.2, y=-0.3, z=0.3, 
#     r1=3.14, r2=0.0, r3=-1.57, 
#     rotation_format="RPY", 
#     reference_frame="WORLD", 
#     is_relative=False, 
#     cartesian_path=True 
# ))
# print(robot.move_to_pose(
#     x=0.4, y=0.3, z=0.2, 
#     r1=3.14, r2=0.0, r3=-1.57, 
#     rotation_format="RPY", 
#     reference_frame="WORLD", 
#     is_relative=False, 
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



# print("Joints :", robot.get_joint_state())
# print("Pose (Euler) :", robot.get_current_pose(output_format="euler"))

# print(robot.set_virtual_cage(
#     enable=True, 
#     front=0.5, back=0.5, 
#     left=0.5, right=0.5, 
#     top=0.8, bottom=0.0
# ))
# time.sleep(2) 

# print(robot.move_to_pose(
#     x=0.30, y=0.0, z=0.50, 
#     r1=0.0, r2=1.57, r3=0.0, 
#     rotation_format="RPY", 
#     reference_frame="WORLD",
#     execute=True,
#     cartesian_path=True
# ))

# print(robot.move_to_pose(
#     x=0.30, y=0.0, z=0.80, 
#     r1=0.0, r2=1.57, r3=0.0, 
#     rotation_format="RPY", 
#     reference_frame="WORLD",
#     execute=True,
#     cartesian_path=True
# ))
# time.sleep(2)

# print(robot.set_virtual_cage(enable=False))


# robot.get_joint_state()
# robot.get_current_pose(output_format="both")