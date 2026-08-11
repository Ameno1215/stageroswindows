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
OFFSET = 0.0
# OFFSET = 0.07
CARTESIAN_PATH = False
CARTESIAN_ALL = True
STAUBLI_SPEED = 0.1
SAFETY = 0.00
SPEED = 0.35
ACCEL = 0.1
PLATES = "plates"
PLATES_STAUBLI = "platesStaubli"

NUMBER_OF_TEST = 1
CARD = 2

CARD_THICKNESS = 0.0008        # m, thickness of one card (align with getCardHeightRos)
DELTA_SUCTION_CUPS = 2 / 1000  # size of the suction cup between card stack and tool
APPROACH_HEIGHT = 0.3          # m, height above the stack before descending
DROP_MARGIN = 0.0              # m, extra height when releasing (0.01 in the app)

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

    time.sleep(3)
    
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

    # robot.start_trace()
    
    robot.move_to_home()

    inputStorage = None
    outputStorage = None
    if MODEL == "tx40":
        inputStorage = box1_staubli
        outputStorage = box2_staubli
    else:
        inputStorage = box1
        outputStorage = box2

    number_of_cards = CARD

    # Number of cards currently in each box, keyed by box number.
    # Single source of truth for the Z of the moves AND for the id of the boxes,
    # so everything stays correct when input/output are swapped.
    stack_count = {
        inputStorage["box_number"]: number_of_cards,
        outputStorage["box_number"]: 0,
    }

    if MODEL == "tx40":
        plates_dir = base_path / PLATES_STAUBLI
    else:
        plates_dir = base_path / PLATES

    # Iterate over all items in the 'plates' directory that start with 'plate'
    for plate_index, plate_dir in enumerate(plates_dir.glob("plate*")):

        # Ensure it's actually a directory (and not a file named plate_something)
        if plate_dir.is_dir():
            
            # Look for the JSON file inside this sub-directory
            json_files = list(plate_dir.glob("*.json"))
            
            if json_files:
                json_path = json_files[0] # Take the first (and supposedly only) JSON file
                print(f'Loading plate from: {json_path.name}')
                
                # Load the plate
                plate = load_plate_from_file(json_path)
                win_logger.info(f"Testing plate : {plate.plate_number}")

                for reader in plate.readers:
                    for pos in reader.positions:
                        if MODEL == "tx40":
                            robot.manage_box(
                                box_id=f"{reader.reader_name}_{pos.position_label}",
                                x=pos.x-STAUBLI_PLATE_OFFSET, y=pos.y, z=pos.z,
                                r1=pos.rx, r2=pos.ry, r3=pos.rz,
                                size_x=0.06, size_y=0.09, size_z=0.02,
                                action="ADD",
                                enable_collision=False
                            )
                        else:
                            robot.manage_box(
                                box_id=f"{reader.reader_name}_{pos.position_label}",
                                x=pos.x, y=pos.y, z=pos.z,
                                r1=pos.rx, r2=pos.ry, r3=pos.rz,
                                size_x=0.06, size_y=0.09, size_z=0.02, a=0.7,
                                action="ADD",
                                enable_collision=False
                            )
                
                if MODEL == "tx40":
                    robot.manage_mesh(
                        mesh_id=f"plaque{plate.plate_number}",
                        mesh_path=to_path_real(plate.mesh_path),
                        x=0.557+0.135/2-STAUBLI_PLATE_OFFSET+plate.mesh_offset_x, y=-0.25+plate.mesh_offset_y, z=0+plate.mesh_offset_z,
                        r1=pi/180*plate.mesh_rotation_x, r2=pi/180*plate.mesh_rotation_y, r3=pi/180*plate.mesh_rotation_z,
                        rotation_format="RPY",
                        a=1, r=0, g=1, b=0,
                        action="ADD"
                    )
                else:
                    robot.manage_mesh(
                        mesh_id=f"plaque{plate.plate_number}",
                        mesh_path=to_path_real(plate.mesh_path),
                        x=0.557+0.135/2+plate.mesh_offset_x, y=-0.25+plate.mesh_offset_y, z=0+plate.mesh_offset_z,
                        r1=pi/180*plate.mesh_rotation_x, r2=pi/180*plate.mesh_rotation_y, r3=pi/180*plate.mesh_rotation_z,
                        rotation_format="RPY",
                        a=1, r=0, g=1, b=0,
                        action="ADD"
                    )       

                for reader_index, reader in enumerate(plate.readers):
                    # Initial stack: created ONCE, on the first reader of the first plate.
                    # Afterwards the boxes follow the cards through the ADD/REMOVE below.
                    if plate_index == 0 and reader_index == 0:
                        for u in range(number_of_cards):
                            hue = (u / max(number_of_cards, 1)) % 1.0
                            r_, g_, b_ = colorsys.hsv_to_rgb(hue, 0.85, 0.95)
                            robot.manage_box(box_id=f"storage_{inputStorage['box_number']}_{u}",
                                             x=inputStorage["position"]["x"],
                                             y=inputStorage["position"]["y"],
                                             # the position retrieved from the card box is like a card already
                                             # caught, so there is actually the size of the suction cup
                                             # between card stack and tool
                                             z=inputStorage["position"]["z"] + OFFSET + u*CARD_THICKNESS - DELTA_SUCTION_CUPS,
                                             r1=0,
                                             r2=inputStorage["position"]["ry"],
                                             r3=inputStorage["position"]["rz"],
                                             rotation_format="RPY",
                                             size_y=0.09,
                                             size_x=0.06,
                                             size_z=CARD_THICKNESS,
                                             r=r_,
                                             g=g_,
                                             b=b_,
                                             action="ADD",
                                             )

                    win_logger.info(f'Testing reader: {reader.reader_name}')
                    for card in range(number_of_cards):
                        if card == 0 and reader_index !=0:
                            # the card is still on the suction cup from the previous reader
                            pass
                        else:                      
                            win_logger.info(f'Robot is going to take card {card} by move pose')

                            # level of the card currently on top of the source stack
                            input_level = stack_count[inputStorage["box_number"]] - 1

                            robot.move_to_pose(
                                x=inputStorage["position"]["x"],
                                y=inputStorage["position"]["y"],
                                z=inputStorage["position"]["z"] + OFFSET + input_level*CARD_THICKNESS + APPROACH_HEIGHT,
                                r1=inputStorage["position"]["rx"],
                                r2=inputStorage["position"]["ry"],
                                r3=inputStorage["position"]["rz"],
                                rotation_format="RPY",
                                reference_frame="WORLD",
                                is_relative=False,
                                cartesian_path=CARTESIAN_PATH,
                                execute=True
                            )

                            robot.move_to_pose(
                                x=0,
                                y=0,
                                z=-APPROACH_HEIGHT,
                                r1=0,
                                r2=0,
                                r3=0,
                                rotation_format="RPY",
                                reference_frame="WORLD",
                                is_relative=True,
                                cartesian_path=CARTESIAN_ALL,
                                execute=True
                            )

                            if not SIM:
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

                            # the card left the stack: remove its box and update the count
                            robot.manage_box(box_id=f"storage_{inputStorage['box_number']}_{input_level}",
                                             action="REMOVE")
                            stack_count[inputStorage["box_number"]] -= 1

                            robot.move_to_pose(
                                x=0,
                                y=0,
                                z=APPROACH_HEIGHT,
                                r1=0,
                                r2=0,
                                r3=0,
                                rotation_format="RPY",
                                reference_frame="WORLD",
                                is_relative=True,
                                cartesian_path=CARTESIAN_ALL,
                                execute=True
                            )
                    
                        for pos_index, pos in enumerate(reader.positions):
                            win_logger.info(f'Testing card {card} on position: {pos.position_label} of reader: {reader.reader_name}')

                            if MODEL == "tx40":
                                robot.move_approach(
                                    x=pos.x-STAUBLI_PLATE_OFFSET, y=pos.y, z=pos.z,
                                    r1=pos.rx, r2=pos.ry, r3=pos.rz,
                                    z_offset=0.12,
                                    rotation_format="RPY",
                                    cartesian_path=CARTESIAN_PATH,
                                    execute=True
                                )
                            else:
                                robot.move_approach(
                                    x=pos.x, y=pos.y, z=pos.z,
                                    r1=pos.rx, r2=pos.ry, r3=pos.rz,
                                    z_offset=0.12,
                                    rotation_format="RPY",
                                    cartesian_path=CARTESIAN_PATH,
                                    execute=True
                                )

                            for k in range(NUMBER_OF_TEST):
                                robot.move_to_pose(
                                    x=0, y=0, z=0.12 - SAFETY,
                                    r1=0, r2=0, r3=0,
                                    rotation_format="RPY",
                                    reference_frame="TOOL",
                                    cartesian_path=CARTESIAN_ALL,
                                    is_relative=True,
                                    execute=True
                                )

                                robot.move_to_pose(
                                    x=0, y=0, z=-0.12 + SAFETY,
                                    r1=0, r2=0, r3=0,
                                    rotation_format="RPY",
                                    reference_frame="TOOL",
                                    cartesian_path=CARTESIAN_ALL,
                                    is_relative=True,
                                    execute=True
                                )

                        if card == number_of_cards-1 and reader_index != len(plate.readers)-1:
                            win_logger.info("Robot don't release the card to gain time")
                            pass
                        else:
                            win_logger.info(f'Robot is going to release card: {card} by move pose')

                            # level the card will occupy once dropped on the destination stack
                            output_level = stack_count[outputStorage["box_number"]]

                            robot.move_to_pose(
                                x=outputStorage["position"]["x"],
                                y=outputStorage["position"]["y"],
                                z=outputStorage["position"]["z"] + OFFSET + output_level*CARD_THICKNESS + DROP_MARGIN + APPROACH_HEIGHT,
                                r1=outputStorage["position"]["rx"],
                                r2=outputStorage["position"]["ry"],
                                r3=outputStorage["position"]["rz"],
                                rotation_format="RPY",
                                reference_frame="WORLD",
                                is_relative=False,
                                cartesian_path=CARTESIAN_PATH,
                                execute=True
                            )

                            robot.move_to_pose(
                                x=0,
                                y=0,
                                z=-APPROACH_HEIGHT,
                                r1=0,
                                r2=0,
                                r3=0,
                                rotation_format="RPY",
                                reference_frame="WORLD",
                                is_relative=True,
                                cartesian_path=CARTESIAN_ALL,
                                execute=True
                            )

                            if not SIM:
                                robot.pump_release()

                            # the card joined the destination stack: add its box and update the count
                            hue = (output_level / max(number_of_cards, 1)) % 1.0
                            r_, g_, b_ = colorsys.hsv_to_rgb(hue, 0.85, 0.95)
                            robot.manage_box(box_id=f"storage_{outputStorage['box_number']}_{output_level}",
                                             x=outputStorage["position"]["x"],
                                             y=outputStorage["position"]["y"],
                                             # the position retrieved from the card box is like a card already
                                             # caught, so there is actually the size of the suction cup
                                             # between card stack and tool
                                             z=outputStorage["position"]["z"] + OFFSET + output_level*CARD_THICKNESS - DELTA_SUCTION_CUPS,
                                             r1=0,
                                             r2=outputStorage["position"]["ry"],
                                             r3=outputStorage["position"]["rz"],
                                             rotation_format="RPY",
                                             size_y=0.09,
                                             size_x=0.06,
                                             size_z=CARD_THICKNESS,
                                             r=r_,
                                             g=g_,
                                             b=b_,
                                             action="ADD",
                                             )
                            stack_count[outputStorage["box_number"]] += 1

                            robot.move_to_pose(
                                x=0,
                                y=0,
                                z=APPROACH_HEIGHT,
                                r1=0,
                                r2=0,
                                r3=0,
                                rotation_format="RPY",
                                reference_frame="WORLD",
                                is_relative=True,
                                cartesian_path=CARTESIAN_ALL,
                                execute=True
                            )

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
                win_logger.info("Going home")
                robot.move_to_home()     

                for reader in plate.readers:
                    for pos in reader.positions:
                        robot.manage_box(
                            box_id=f"{reader.reader_name}_{pos.position_label}",
                            action="REMOVE"
                        )      

                robot.manage_mesh(
                    mesh_id=f"plaque{plate.plate_number}",
                    action="REMOVE"
                )

    time.sleep(2)

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


