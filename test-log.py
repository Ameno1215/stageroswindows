import math
import time
from motion_http_client import MotionRobotClient
from math import pi
from plate import load_plate_from_file
from pathlib import Path
import urllib.parse
from logger_worker import tail_linux_logs, win_logger
import threading


box_source = 1
box1 = {
    "box_number": 1,
    "position": {
        "x": 0.2581,
        "y": 0.1360,
        "z": 0.01,
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
        "z": 0.01,
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



    robot = MotionRobotClient("http://localhost:8000")

    win_logger.info(f"Health: {robot.health()}")
    win_logger.info(f"Initialising robot")
    print(robot.init_robot(model="vs060", 
                           planning_group="arm", 
                           velocity_scale=0.2, 
                           accel_scale=0.2, 
                           planning_time=10, 
                           planning_attempts=20, 
                           allow_replanning=True, 
                           planner_id="RRTConnect"))

    robot.set_scaling(velocity_scale=1, accel_scale=1)

    print(robot.manage_mesh(
        mesh_id="box1",
        mesh_path=to_path_real(rel_path_to_stl),
        x=box1["position"]["x"], y=box1["position"]["y"], z=0.0,
        r1=0.0, r2=0.0, r3=0.0,
        scale_x=0.001, scale_y=0.001, scale_z=0.001,
        rotation_format="RPY",
        a=1, r=1, g=0, b=0,
        action="ADD"
    ))

    print(robot.manage_mesh(
        mesh_id="box2",
        mesh_path=to_path_real(rel_path_to_stl),
        x=box2["position"]["x"], y=box2["position"]["y"], z=0.0,
        r1=0.0, r2=0.0, r3=0.0,
        scale_x=0.001, scale_y=0.001, scale_z=0.001,
        rotation_format="RPY",
        a=0.5, r=0, g=0, b=1,
        action="ADD"
    ))


    robot.set_virtual_cage(
        enable=True, 
        front=0.66, back=0.35, 
        left=0.325, right=0.325, 
        top=0.9, bottom=0.0
    )
    time.sleep(2)

    robot.move_to_home()

    # robot.move_joints([0.0, 0.0, 1.57, 0.0, 1.57, 5], is_relative=False)
    
    robot.move_to_pose(
        x=0.0,
        y=0,
        z=0.0,
        r1=0,
        r2=0,
        r3=0,
        rotation_format="RPY",
        reference_frame="WORLD",
        angle_format="RAD",
        is_relative=False,
        cartesian_path=False,
        execute=True
    )

    print(robot.get_current_pose())


    
    time.sleep(10)


    
    print(robot.set_virtual_cage(enable=False))  

    robot.manage_mesh(
        mesh_id="box1",
        action="REMOVE"
    )
    robot.manage_mesh(
        mesh_id="box2",
        action="REMOVE"
    )

    
         



if __name__ == "__main__":
    try:
        run()
    except RuntimeError as e:
        win_logger.error(f"Program stopped due to motion error: {e}")
    except KeyboardInterrupt:
        win_logger.info("Program interrupted by user.")
    except Exception as e:
        win_logger.error(f"Unexpected error: {e}")
