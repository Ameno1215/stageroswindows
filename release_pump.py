import math
import time
from motion_http_client import MotionRobotClient
from math import pi
from utils.plate import load_plate_from_file
from pathlib import Path
import urllib.parse
from utils.logger_worker import tail_linux_logs, win_logger
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
    robot.init_robot(model="vs060", 
                           planning_group="arm", 
                           velocity_scale=0.1, 
                           accel_scale=0.1, 
                           planning_time=10, 
                           planning_attempts=20, 
                           allow_replanning=True, 
                           planner_id="RRTConnect")


    
    robot.pump_release()
    
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
