import math
import time
from motion_http_client import MotionRobotClient
from math import pi
from utils.plate import load_reader_plate_from_file
from pathlib import Path
import urllib.parse
from utils.logger_worker import tail_linux_logs, win_logger
import threading
import argparse
import colorsys


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
SAFETY=0.00
SPEED = 1
ACCEL = 1
PLATES = "plates"
PLATES_STAUBLI = "platesStaubli"

NUMBER_OF_TEST = 1
number_of_cards = 3
REPEAT = 3

box_source = 1
box1 = {
    "box_number": 1,
    "position": {
        "x": 0.2581,
        "y": 0.1361,
        "z": 0.008,
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
        "z": 0.008,
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
        "z": 0.006,
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
        "z": 0.006,
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
        robot.load_environnement("C:/Users/Azur_Local_User/Documents/DEV/ROS/env/tx40.json")
    else:
        robot.load_environnement("C:/Users/Azur_Local_User/Documents/DEV/ROS/env/vs060.json")
    robot.clear_trace()
    
    
    time.sleep(2)

    print(robot.set_servo_on(True))

    robot.start_trace()
    
    
    # robot.pump_release()
    # return

    # robot.pump_grab()

    # t = time.time()

    # while time.time() - t < 5:
    #     print(f'at t={time.time() - t}: {robot.pump_is_grabbed()}')
    #     if robot.pump_is_grabbed()["grabbed"]:
    #         win_logger.info("Card is grabbed")
    #         break
    
    # robot.pump_release()
    
    # return

    robot.move_to_home()


    inputStorage = None
    outputStorage = None
    if MODEL == "tx40":
        inputStorage = box1_staubli
        outputStorage = box2_staubli
    else:
        inputStorage = box1
        outputStorage = box2


    heightSafeStorage = 0.17
    cardThickness = 0.84/1000
    deltaSuctionCups = 2/1000

    for i in range(number_of_cards):
        hue = (i / max(number_of_cards, 1)) % 1.0
        r_, g_, b_ = colorsys.hsv_to_rgb(hue, 0.85, 0.95)
        robot.manage_box(box_id=f"card_{inputStorage['box_number']}_{i}", 
                                x=inputStorage["position"]["x"],
                                y=inputStorage["position"]["y"],
                                z=inputStorage["position"]["z"]-deltaSuctionCups+cardThickness*i,
                                # the position retrive in KEOLABS from the card box is like a card catched so there is actualy 
                                # the size of the suction cup beetwen card stack and tool
                                r1=0,
                                r2=inputStorage["position"]["ry"],
                                r3=inputStorage["position"]["rz"],
                                rotation_format="RPY",
                                size_y=0.085,
                                size_x=0.054,
                                size_z=cardThickness,
                                r=r_,
                                g=g_,
                                b=b_,
                                )
    for repeat in range(REPEAT): 
        for card in range(number_of_cards):
                    
                win_logger.info(f'Robot is going to take card {card} by move pose')
                boxThickness = inputStorage["position"]["z"]
                win_logger.info(f'Height is boxThickness + heightSafeStorage = {boxThickness + heightSafeStorage}')
                            
                robot.move_to_pose(
                    x=inputStorage["position"]["x"],
                    y=inputStorage["position"]["y"],
                    z=boxThickness + heightSafeStorage,
                    r1=inputStorage["position"]["rx"],
                    r2=inputStorage["position"]["ry"],
                    r3=inputStorage["position"]["rz"],
                    rotation_format="RPY",
                    reference_frame="WORLD",
                    is_relative=False,
                    cartesian_path=CARTESIAN_PATH,
                    execute=True
                )

                cardsHeight = cardThickness * (number_of_cards - card)
                delta = -cardsHeight + heightSafeStorage
                

                
                win_logger.info(f'Going down')
                win_logger.info(f'Height is boxThickness + heightSafeStorage - delta = {boxThickness + heightSafeStorage - delta}')
                robot.move_to_pose(
                    x=0,
                    y=0,
                    z=-delta,
                    r1=0,
                    r2=0,
                    r3=0,
                    rotation_format="RPY",
                    reference_frame="WORLD",
                    is_relative=True,
                    cartesian_path=CARTESIAN_ALL,
                    execute=True
                )
                robot.pump_grab()

                t = time.time()

                bool_grabbed = False
                while time.time() - t < 5:
                    print(f'at t={time.time() - t}: {robot.pump_is_grabbed()}')
                    if robot.pump_is_grabbed()["grabbed"]:
                        bool_grabbed = True
                        win_logger.info("Card is grabbed")
                        break
                
                if not bool_grabbed:
                    robot.pump_release()
                    
                # cardsHeightAfter = cardThickness * (number_of_cards - card - 1)
                robot.manage_box(box_id=f"card_{inputStorage['box_number']}_{number_of_cards - card - 1}", action="REMOVE")

                win_logger.info(f'Going up')
                win_logger.info(f'Height is boxThickness + heightSafeStorage - delta = {boxThickness + heightSafeStorage - delta}')
                robot.move_to_pose(
                    x=0,
                    y=0,
                    z=delta,
                    r1=0,
                    r2=0,
                    r3=0,
                    rotation_format="RPY",
                    reference_frame="WORLD",
                    is_relative=True,
                    cartesian_path=CARTESIAN_ALL,
                    execute=True
                )
                        
                
            
                win_logger.info(f'Robot is going to release card: {card} by move pose')
                boxThickness = outputStorage["position"]["z"]
                win_logger.info(f'Height is boxThickness + heightSafeStorage = {boxThickness + heightSafeStorage}')
                            
                robot.move_to_pose(
                    x=outputStorage["position"]["x"],
                    y=outputStorage["position"]["y"],
                    z=boxThickness + heightSafeStorage+0.01,
                    r1=outputStorage["position"]["rx"],
                    r2=outputStorage["position"]["ry"],
                    r3=outputStorage["position"]["rz"],
                    rotation_format="RPY",
                    reference_frame="WORLD",
                    is_relative=False,
                    cartesian_path=CARTESIAN_PATH,
                    execute=True
                )

                cardsHeight = cardThickness * (card)
                delta = -cardsHeight + heightSafeStorage
                
                win_logger.info(f'Going down')
                win_logger.info(f'Height is boxThickness + heightSafeStorage - delta = {boxThickness + heightSafeStorage - delta}')
                robot.move_to_pose(
                    x=0,
                    y=0,
                    z=-delta,
                    r1=0,
                    r2=0,
                    r3=0,
                    rotation_format="RPY",
                    reference_frame="WORLD",
                    is_relative=True,
                    cartesian_path=CARTESIAN_ALL,
                    execute=True
                )

        
                robot.pump_release()
                
                # cardsHeightAfter = cardThickness * (card+1)
                hue = (card / max(number_of_cards, 1)) % 1.0
                r_, g_, b_ = colorsys.hsv_to_rgb(hue, 0.85, 0.95)
                robot.manage_box(box_id=f"card_{outputStorage['box_number']}_{card}", 
                                        x=outputStorage["position"]["x"],
                                        y=outputStorage["position"]["y"],
                                        z=outputStorage["position"]["z"]-deltaSuctionCups+cardThickness*card,
                                        # the position retrive in KEOLABS from the card box is like a card catched so there is actualy 
                                        # the size of the suction cup beetwen card stack and tool
                                        r1=0,
                                        r2=outputStorage["position"]["ry"],
                                        r3=outputStorage["position"]["rz"],
                                        rotation_format="RPY",
                                        size_y=0.085,
                                        size_x=0.054,
                                        size_z=cardThickness,
                                        r=r_,
                                        g=g_,
                                        b=b_,
                                        )

                win_logger.info(f'Going up')
                win_logger.info(f'Height is boxThickness + heightSafeStorage - delta = {boxThickness + heightSafeStorage - delta}')
                robot.move_to_pose(
                    x=0,
                    y=0,
                    z=delta,
                    r1=0,
                    r2=0,
                    r3=0,
                    rotation_format="RPY",
                    reference_frame="WORLD",
                    is_relative=True,
                    cartesian_path=CARTESIAN_ALL,
                    execute=True
                )


                win_logger.info("Going home")
        robot.move_to_home()
        if MODEL == "tx40":
            if inputStorage is box1_staubli:
                inputStorage = box2_staubli
                outputStorage = box1_staubli
            else:
                inputStorage = box1_staubli
                outputStorage = box2_staubli
        else:
            if inputStorage is box1:
                inputStorage = box2
                outputStorage = box1
            else:
                inputStorage = box1
                outputStorage = box2     


    time.sleep(1)

    print(robot.set_servo_on(False))
    # robot.stop_trace()
    
    robot.clear_environment()

    
         

if __name__ == "__main__":
    try:
        run()
    except RuntimeError as e:
        win_logger.error(f"Program stopped due to motion error: {e}")
    except KeyboardInterrupt:
        win_logger.info("Program interrupted by user.")
    except Exception as e:
        win_logger.error(f"Unexpected error: {e}")


# if __name__ == "__main__":
#     total_runs = 10
#     crash_count = 0

#     for i in range(1, total_runs + 1):
#         win_logger.info(f"\n\n\n\t{i}/{total_runs}\n\n\n")
#         win_logger.info(f"--- Run {i}/{total_runs} ---")
#         try:
#             run()
#         except RuntimeError as e:
#             win_logger.error(f"Run {i} crashed (motion error): {e}")
#             crash_count += 1
#         except KeyboardInterrupt:
#             win_logger.info("Program interrupted by user.")
#             break
#         except Exception as e:
#             win_logger.error(f"Run {i} crashed (unexpected): {e}")
#             crash_count += 1

#     win_logger.info(f"\n\n\n\n\nResults: {crash_count}/{total_runs} crashes")
