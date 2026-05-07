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
parser.add_argument("--model", choices=["vs060", "vp5243", "tx2_60l", "tx40"], default="vs060",
                    help="Robot model to use (default: vs060)")

args = parser.parse_args()

SIM = args.real_robot
MODEL = args.model

def run():
    linux_log_path = r"\\wsl.localhost\Ubuntu-22.04\home\antonin\workspace\robot_system.log"
    
    log_thread = threading.Thread(
        target=tail_linux_logs, 
        args=(linux_log_path,), 
        daemon=True
    )
    log_thread.start()

    robot = MotionRobotClient("http://localhost:8000", SIM)

    win_logger.info(f"Health: {robot.health()}")
    win_logger.info(f"Initialising robot")
    print(robot.init_robot(model=MODEL,
                           velocity_scale=0.3, 
                           accel_scale=0.1, 
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
    print(robot.get_current_pose())

    robot.set_servo_on(True)
    print(robot.get_current_pose())

    time.sleep(2)

    # robot.pump_release()
    # return

    
    # print(robot.pump_grab())

    # t = time.time()

    # while time.time() - t < 5:
    #     print(f'at t={time.time() - t}: {robot.pump_is_grabbed()}')
    #     if robot.pump_is_grabbed()["grabbed"]:
    #         win_logger.info("Card is grabbed")
    #         break
    
    # print(robot.pump_release())

    # return
    
    # robot.move_to_home()


    robot.move_to_home()
    print(robot.get_current_pose())

    for i in range(10):
        robot.move_to_pose(
            0.15, 0.0, 0,
            0, 0.0, 0.0,
            rotation_format="RPY",
            angle_format="DEG",
            cartesian_path=True,
            is_relative=True,
            execute=True,
        )

        robot.move_to_pose(
            -0.15, 0.0, 0,
            0, 0.0, 0.0,
            rotation_format="RPY",
            angle_format="DEG",
            cartesian_path=True,
            is_relative=True,
            execute=True,
        )
    print(robot.get_current_pose())

    robot.set_servo_on(False)
    robot.set_virtual_cage(
        enable=False
    )

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