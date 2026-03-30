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

args = parser.parse_args()

SIM = args.real_robot

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
    print(robot.init_robot(model="vs060", 
                           planning_group="arm", 
                           velocity_scale=0.2, 
                           accel_scale=0.2, 
                           planning_time=10, 
                           planning_attempts=20, 
                           allow_replanning=True, 
                           planner_id="RRTConnect"))
    

    robot.set_virtual_cage(
        enable=True,
        front=0.66, back=0.35,
        left=0.325, right=0.325,
        top=0.9, bottom=0.0
    )

    

    robot.set_scaling(velocity_scale=0.05, accel_scale=0.05)

    # robot.move_joints([0.0, 0.0, 1.57, 0.0, 1.57, 0], is_relative=False)

    # robot.move_joints([0.0, 0.2, 1.4, 0.0, 1.4, 0], is_relative=False)

    print(robot.set_servo_on(False))
    time.sleep(5)

    
    print(robot.set_servo_on(True))
    time.sleep(2)

    # print(robot.init_robot(model="vs060", 
    #                     planning_group="arm", 
    #                     velocity_scale=0.2, 
    #                     accel_scale=0.2, 
    #                     planning_time=10, 
    #                     planning_attempts=20, 
    #                     allow_replanning=True, 
    #                     planner_id="RRTConnect"))

    return 

    for i in range(10):
        print(i)
        robot.move_to_pose(
            x=0.05,
            y=0,
            z=0,
            r1=0,
            r2=0,
            r3=0,
            rotation_format="RPY",
            is_relative=True,
            cartesian_path=True,
            execute=True
        )

        robot.move_to_pose(
            x=-0.05,
            y=0,
            z=0,
            r1=0,
            r2=0,
            r3=0,
            rotation_format="RPY",
            is_relative=True,
            cartesian_path=True,
            execute=True
        )



    # print(robot.set_servo_on(True))
    # time.sleep(2)

    # robot.move_to_pose(
    #     x=0.05,
    #     y=0,
    #     z=0,
    #     r1=0,
    #     r2=0,
    #     r3=0,
    #     rotation_format="RPY",
    #     is_relative=True,
    #     cartesian_path=True,
    #     execute=True
    # )

    
    # robot.move_joints([0.0, 0.0, 1.57, 0.0, 1.57, 0], is_relative=False)

    robot.set_virtual_cage(
        enable=False
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
