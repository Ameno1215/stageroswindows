import math
import time
from motion_http_client import MotionRobotClient
from math import pi
from plate import load_plate_from_file
from pathlib import Path
import urllib.parse
from logger_worker import tail_linux_logs, win_logger
import threading
import argparse


parser = argparse.ArgumentParser(description="Test file to command real robot")
parser.add_argument("--real-robot", action="store_false",
                    help="Connect to the real robot (default: simulation)")
parser.add_argument("--model", choices=["vs060", "vp5243", "tx40"], default="vs060",
                    help="Robot model to use (default: vs060)")

args = parser.parse_args()

SIM = args.real_robot
MODEL = args.model


STAUBLI_PLATE_OFFSET = 0.11
OFFSET = 0.015
# OFFSET = 0.07
CARTESIAN_PATH = False
CARTESIAN_ALL = True
STAUBLI_SPEED = 0.1
SPEED = 0.5
ACCEL = 0.5

NUMBER_OF_TEST = 5

box_source = 1
box1 = {
    "box_number": 1,
    "position": {
        "x": 0.2581,
        "y": 0.1360,
        "z": 0.0,
        "rx": 180*pi/180,
        "ry": 0,
        "rz": -90*pi/180
    }
}
box2 = {
    "box_number": 2,
    "position": {
        "x": 0.2618,
        "y": -0.1192,
        "z": 0.0,
        "rx": 180*pi/180,
        "ry": 0,
        "rz": -90*pi/180
    }
}
box1_staubli = {
    "box_number": 1,
    "position": {
        "x": 0.15,
        "y": 0.15,
        "z": 0.0,
        "rx": 180*pi/180,
        "ry": 0,
        "rz": -90*pi/180
    }
}
box2_staubli = {
    "box_number": 2,
    "position": {
        "x": 0.15,
        "y": -0.15,
        "z": 0.0,
        "rx": 180*pi/180,
        "ry": 0,
        "rz": -90*pi/180
    }
}


base_path = Path.cwd()
rel_path_to_stl = "card_storage/boitier.STL"

def to_path_real(input):
    abs_path_to_stl = base_path / input

    posix_path = abs_path_to_stl.as_posix()

    if posix_path.lower().startswith("c:/"):
        wsl_path = "/mnt/c/" + posix_path[3:]
    else:
        wsl_path = posix_path

    return f"file://{wsl_path}"




def run():
    linux_log_path = r"\\wsl.localhost\Ubuntu-22.04\home\antonin\workspace\robot_system.log"
    
    # Start the log reader in a background daemon thread
    log_thread = threading.Thread(
        target=tail_linux_logs, 
        args=(linux_log_path,), 
        daemon=True
    )
    log_thread.start()

    robot = MotionRobotClient("http://localhost:8000", SIM)

    win_logger.info(f"Health: {robot.health()}")
    win_logger.info(f"Initialising robot")
    robot.init_robot(model=MODEL, 
                           velocity_scale=0.1, 
                           accel_scale=0.1, 
                           planning_time=10, 
                           planning_attempts=20, 
                           allow_replanning=True, 
                           planner_id="RRTConnect")

    if MODEL == "tx40":
        robot.set_scaling(velocity_scale=STAUBLI_SPEED, accel_scale=ACCEL, cartesian_speed=SPEED)
    else:
        robot.set_scaling(velocity_scale=SPEED, accel_scale=ACCEL)

    robot.clear_environment()

    if MODEL == "tx40":
        robot.manage_mesh(
            mesh_id="box1",
            mesh_path=to_path_real(rel_path_to_stl),
            x=box1_staubli["position"]["x"], y=box1_staubli["position"]["y"], z=box1_staubli["position"]["z"],
            r1=0.0, r2=0.0, r3=0.0,
            scale_x=0.001, scale_y=0.001, scale_z=0.001,
            rotation_format="RPY",
            a=1, r=0.5, g=0.5, b=0.5,
            action="ADD"
        )
        robot.manage_mesh(
            mesh_id="box2",
            mesh_path=to_path_real(rel_path_to_stl),
            x=box2_staubli["position"]["x"], y=box2_staubli["position"]["y"], z=box2_staubli["position"]["z"],
            r1=0.0, r2=0.0, r3=0.0,
            scale_x=0.001, scale_y=0.001, scale_z=0.001,
            rotation_format="RPY",
            a=1, r=0.5, g=0.5, b=0.5,
            action="ADD"
        )
    else:
        robot.manage_mesh(
            mesh_id="box1",
            mesh_path=to_path_real(rel_path_to_stl),
            x=box1["position"]["x"], y=box1["position"]["y"], z=box1["position"]["z"],
            r1=0.0, r2=0.0, r3=0.0,
            scale_x=0.001, scale_y=0.001, scale_z=0.001,
            rotation_format="RPY",
            a=1, r=0.5, g=0.5, b=0.5,
            action="ADD"
        )
        robot.manage_mesh(
            mesh_id="box2",
            mesh_path=to_path_real(rel_path_to_stl),
            x=box2["position"]["x"], y=box2["position"]["y"], z=box2["position"]["z"],
            r1=0.0, r2=0.0, r3=0.0,
            scale_x=0.001, scale_y=0.001, scale_z=0.001,
            rotation_format="RPY",
            a=1, r=0.5, g=0.5, b=0.5,
            action="ADD"
        )

    robot.set_virtual_fence(
        enable=True, 
        front=0.66, back=0.35, 
        left=0.325, right=0.325, 
        top=0.9, bottom=0.0
    )
    time.sleep(2)

    print(robot.set_servo_on(True))

    robot.move_to_home()
    robot.clear_trace()
    robot.start_trace()


    side = 0.15
    a = 1
    square = [
        {"x":  side, "y":  0.0,  "z": 0, "r1": 0, "r2": 0, "r3": 0,
        "is_relative": True, "reference_frame": "WORLD"},
        {"x":  0.0,  "y":  -a*side, "z": 0, "r1": 0, "r2": 0, "r3": 0,
        "is_relative": True, "reference_frame": "WORLD"},
        {"x": -side, "y":  0.0,  "z": 0, "r1": 0, "r2": 0, "r3": 0,
        "is_relative": True, "reference_frame": "WORLD"},
        {"x":  0.0,  "y": a*side, "z": 0, "r1": 0, "r2": 0, "r3": 0,
        "is_relative": True, "reference_frame": "WORLD"},
    ]

    robot.move_waypoints(square, cartesian_path=False, blend_radius=0.05, path_tolerance=0.05)

    return



    
    # x=0.3
    # y=0.1
    # z1=0.39
    # z2=0.2
    # robot.move_to_home()
    # robot.move_to_pose(x = x, y=y, z=z1, r1=math.pi, r2=0, r3=math.pi, is_relative=False, cartesian_path=True)

    # T = 8

    # robot.clear_trace()
    # robot.start_trace()

    # print("\nnon cartesian")
    # x_=0
    # y_=0
    # z1_=-0.15
    # z2_=0.15
    # for i in range(T):
    #     robot.move_to_pose(x = x_, y=-y_, z=z1_, r1=0, r2=0, r3=0, is_relative=True, cartesian_path=False)
    #     # time.sleep(0.1)
    #     pos = robot.get_current_pose()["position"]
    #     print(f"x={round(pos["x"], 5)}, y={round(pos["y"], 5)}, z={round(pos["z"], 5)}")
    
    #     robot.move_to_pose(x =x_, y=-y_, z=z2_, r1=0, r2=0, r3=0, is_relative=True, cartesian_path=False)
    #     # time.sleep(0.1)
    #     pos = robot.get_current_pose()["position"]
    #     print(f"x={round(pos["x"], 5)}, y={round(pos["y"], 5)}, z={round(pos["z"], 5)}")
        
    # print("\n")
    # robot.stop_trace()
    # robot.move_to_pose(x = x, y=-y, z=z1, r1=math.pi, r2=0, r3=math.pi, is_relative=False, cartesian_path=True)
    # robot.start_trace()
    
    # x_=0
    # y_=0
    # z1_=-0.15
    # z2_=0.15
    # for i in range(T):
    #     robot.move_to_pose(x = x_, y=-y_, z=z1_, r1=0, r2=0, r3=0, is_relative=True, cartesian_path=False)
    #     time.sleep(0.3)
    #     pos = robot.get_current_pose()["position"]
    #     print(f"x={round(pos["x"], 5)}, y={round(pos["y"], 5)}, z={round(pos["z"], 5)}")
    
    #     robot.move_to_pose(x =x_, y=-y_, z=z2_, r1=0, r2=0, r3=0, is_relative=True, cartesian_path=False)
    #     time.sleep(0.3)
    #     pos = robot.get_current_pose()["position"]
    #     print(f"x={round(pos["x"], 5)}, y={round(pos["y"], 5)}, z={round(pos["z"], 5)}")


    # robot.start_trace()
    # print("\nCartesian")
    # for i in range(T):
    #     robot.move_to_pose(x = x, y=y, z=z1, r1=math.pi, r2=0, r3=math.pi, is_relative=False, cartesian_path=True)
    #     # time.sleep(0.5)
    #     pos = robot.get_current_pose()["position"]
    #     print(f"x={round(pos["x"], 5)}, y={round(pos["y"], 5)}, z={round(pos["z"], 5)}")
    #     print(f"DELTA x={round((x-pos["x"])*1000, 5)}, y={round((y-pos["y"])*1000, 5)}, z={round((z1-pos["z"])*1000, 5)}")
    
    #     robot.move_to_pose(x =x, y=y, z=z2, r1=math.pi, r2=0, r3=math.pi, is_relative=False, cartesian_path=True)
    #     # time.sleep(0.5)
    #     pos = robot.get_current_pose()["position"]
    #     print(f"x={round(pos["x"], 5)}, y={round(pos["y"], 5)}, z={round(pos["z"], 5)}")
    #     print(f"DELTA x={round((x-pos["x"])*1000, 5)}, y={round((y-pos["y"])*1000, 5)}, z={round((z2-pos["z"])*1000, 5)}")
    
    # robot.stop_trace()



    side = 0.15
    a = 1
    # robot.move_to_home()
    # robot.move_to_pose(x=0, y=0, z=-0.1, r1=0, r2=0, r3=0, is_relative=True, cartesian_path=False, execute=True)

    robot.clear_trace()
    robot.start_trace()


    square = [
        {"x":  side, "y":  0.0,  "z": -side, "r1": 0, "r2": 0, "r3": 0,
        "is_relative": True, "reference_frame": "WORLD"},
        {"x":  0.0,  "y":  -a*side, "z": side, "r1": 0, "r2": 0, "r3": 0,
        "is_relative": True, "reference_frame": "WORLD"},
        {"x": -side, "y":  0.0,  "z": -side, "r1": 0, "r2": 0, "r3": 0,
        "is_relative": True, "reference_frame": "WORLD"},
        {"x":  0.0,  "y": a*side, "z": side, "r1": 0, "r2": 0, "r3": 0,
        "is_relative": True, "reference_frame": "WORLD"},
    ]

    TIMES = 10

    robot.stop_trace()
    robot.move_to_home()
    home_pos=robot.get_current_pose()["position"]
    robot.move_to_pose(x=0, y=0, z=-0.12, r1=0, r2=0, r3=0, is_relative=True, cartesian_path=False, execute=True)
    
    theorical_pos = robot.get_current_pose()["position"]
    for j in range(TIMES):
        print(j)
        robot.start_trace()
        for int, i in enumerate(square):
            robot.move_to_pose(x=i["x"], y=i["y"], z=i["z"], r1=0, r2=0, r3=0, cartesian_path=False, is_relative=True)
            theorical_pos = {
                "x": theorical_pos["x"] + i["x"],
                "y": theorical_pos["y"] + i["y"],
                "z": theorical_pos["z"] + i["z"],
            }
            # if ((j*4+int)-2)%4==0:
            pos = robot.get_current_pose()["position"]
            print(f"x={round(pos["x"], 5)}, y={round(pos["y"], 5)}, z={round(pos["z"], 5)}")
            print(f"\t\t\t\t\tDELTA x={round((theorical_pos["x"]-pos["x"])*1000, 5)}, y={round((theorical_pos["y"]-pos["y"])*1000, 5)}, z={round((theorical_pos["z"]-pos["z"])*1000, 5)}")
        robot.stop_trace()

    a=-1
    square = [
        {"x":  side, "y":  0.0,  "z": 0.0, "r1": 0, "r2": 0, "r3": 0,
        "is_relative": True, "reference_frame": "WORLD"},
        {"x":  0.0,  "y":  -a*side, "z": 0.0, "r1": 0, "r2": 0, "r3": 0,
        "is_relative": True, "reference_frame": "WORLD"},
        {"x": -side, "y":  0.0,  "z": 0.0, "r1": 0, "r2": 0, "r3": 0,
        "is_relative": True, "reference_frame": "WORLD"},
        {"x":  0.0,  "y": a*side, "z": 0.0, "r1": 0, "r2": 0, "r3": 0,
        "is_relative": True, "reference_frame": "WORLD"},
    ]

    robot.stop_trace()
    robot.move_to_home()
    robot.move_to_pose(x=0, y=0, z=-0.1, r1=0, r2=0, r3=0, is_relative=True, cartesian_path=False, execute=True)

    theorical_pos = robot.get_current_pose()["position"]
    for i in range(TIMES):
        print(i)
        robot.start_trace()
        for int, i in enumerate(square):
            robot.move_to_pose(x=i["x"], y=i["y"], z=i["z"], r1=0, r2=0, r3=0, cartesian_path=True, is_relative=True)
            theorical_pos = {
                "x": theorical_pos["x"] + i["x"],
                "y": theorical_pos["y"] + i["y"],
                "z": theorical_pos["z"] + i["z"],
            }
            # if ((j*4+int)-2)%4==0:
            pos = robot.get_current_pose()["position"]
            print(f"x={round(pos["x"], 5)}, y={round(pos["y"], 5)}, z={round(pos["z"], 5)}")
            print(f"\t\t\t\t\tDELTA x={round((theorical_pos["x"]-pos["x"])*1000, 5)}, y={round((theorical_pos["y"]-pos["y"])*1000, 5)}, z={round((theorical_pos["z"]-pos["z"])*1000, 5)}")
        robot.stop_trace()


    robot.move_to_home()    
    robot.move_to_pose(x=0.04, y=0.05, z=0, r1=0, r2=0, r3=0, is_relative=True, cartesian_path=False, execute=True)
    
    theorical_pos = robot.get_current_pose()["position"]
    for i in range(TIMES):
        print(f"Down cartesian {i}")
        robot.start_trace()
        robot.move_to_pose(x=0, y=0, z=-0.2, r1=0, r2=0, r3=0, cartesian_path=True, is_relative=True)
        pos = robot.get_current_pose()["position"]
        print(f"x={round(pos["x"], 5)}, y={round(pos["y"], 5)}, z={round(pos["z"], 5)}")
        print(f"\t\t\t\t\tDELTA x={round((theorical_pos["x"]-pos["x"])*1000, 5)}, y={round((theorical_pos["y"]-pos["y"])*1000, 5)}, z={round((0.19-pos["z"])*1000, 5)}")
        robot.move_to_pose(x=0, y=0, z=0.2, r1=0, r2=0, r3=0, cartesian_path=True, is_relative=True)
        pos = robot.get_current_pose()["position"]
        print(f"x={round(pos["x"], 5)}, y={round(pos["y"], 5)}, z={round(pos["z"], 5)}")
        print(f"\t\t\t\t\tDELTA x={round((theorical_pos["x"]-pos["x"])*1000, 5)}, y={round((theorical_pos["y"]-pos["y"])*1000, 5)}, z={round((0.39-pos["z"])*1000, 5)}")
        robot.stop_trace()

    
    robot.move_to_home()    
    robot.move_to_pose(x=0.06, y=0.05, z=0, r1=0, r2=0, r3=0, is_relative=True, cartesian_path=False, execute=True)
    
    theorical_pos = robot.get_current_pose()["position"]
    for i in range(TIMES):
        print(f"Down non-cartesian {i}")
        robot.start_trace()
        robot.move_to_pose(x=0, y=0, z=-0.2, r1=0, r2=0, r3=0, cartesian_path=False, is_relative=True)
        pos = robot.get_current_pose()["position"]
        print(f"x={round(pos["x"], 5)}, y={round(pos["y"], 5)}, z={round(pos["z"], 5)}")
        print(f"\t\t\t\t\tDELTA x={round((theorical_pos["x"]-pos["x"])*1000, 5)}, y={round((theorical_pos["y"]-pos["y"])*1000, 5)}, z={round((0.19-pos["z"])*1000, 5)}")
        robot.move_to_pose(x=0, y=0, z=0.2, r1=0, r2=0, r3=0, cartesian_path=False, is_relative=True)
        pos = robot.get_current_pose()["position"]
        print(f"x={round(pos["x"], 5)}, y={round(pos["y"], 5)}, z={round(pos["z"], 5)}")
        print(f"\t\t\t\t\tDELTA x={round((theorical_pos["x"]-pos["x"])*1000, 5)}, y={round((theorical_pos["y"]-pos["y"])*1000, 5)}, z={round((0.39-pos["z"])*1000, 5)}")
        robot.stop_trace()

    robot.get_current_pose()
    return



if __name__ == "__main__":
    try:
        run()
    except RuntimeError as e:
        win_logger.error(f"Program stopped due to motion error: {e}")
    except KeyboardInterrupt:
        win_logger.info("Program interrupted by user.")
    except Exception as e:
        win_logger.error(f"Unexpected error: {e}")