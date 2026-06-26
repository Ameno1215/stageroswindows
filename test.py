import math
import time
from lib.robotcontroller.robot_controller import MotionRobotClient
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
SPEED = 0.2
ACCEL = 0.2

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
        robot.load_environnement("C:/Users/Azur_Local_User/Documents\/DEV/ROS/env/tx40.json")
    else:
        robot.load_environnement("C:/Users/Azur_Local_User/Documents/DEV/ROS/env/vs060.json")
    
    return
    
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


    # robot.move_to_pose(x=0, y=0.3, z=-0.1, r1=0, r2=0, r3=0, is_relative=True, cartesian_path=True, execute=True)
    # print(robot.get_current_pose())
    # print(robot.get_joint_state())

    # robot.move_to_pose(
    #     x=-0.05, y=-0.1, z=-0.1,
    #     r1=0, r2=0, r3=0,
    #     is_relative=True,
    #     cartesian_path=True,
    #     execute=True
    # )

    # for i in range(5):
    #     robot.move_to_pose(
    #         x=0, y=0.2, z=0,
    #         r1=0, r2=0, r3=0,
    #         is_relative=True,
    #         cartesian_path=False,
    #         execute=True
    #     )
    #     robot.move_to_pose(
    #         x=0.2, y=0, z=0,
    #         r1=0, r2=0, r3=0,
    #         is_relative=True,
    #         cartesian_path=False,
    #         execute=True
    #     )
    #     robot.move_to_pose(
    #         x=0, y=-0.2, z=0,
    #         r1=0, r2=0, r3=0,
    #         is_relative=True,
    #         cartesian_path=False,
    #         execute=True
    #     )
    #     robot.move_to_pose(
    #         x=-0.2, y=0, z=0,
    #         r1=0, r2=0, r3=0,
    #         is_relative=True,
    #         cartesian_path=False,
    #         execute=True
    #     )

    # robot.move_to_home()

    # robot.set_servo_on(False)
    # return
    # points = [
    #         {"x": 0.0, "y": 0.1, "z": -0.2, "r1": 0.0, "r2": 0.0, "r3": 0.0, "is_relative": True},
    #         {"x": 0.1, "y": 0, "z": 0.0, "r1": 0.0, "r2": 0.0, "r3": 0.0, "is_relative": True},
    #         {"x": 0.0, "y": -0.2, "z": 0.1, "r1": 0.0, "r2": 0.0, "r3": 0.0, "is_relative": True},
    #         {"x": -0.1, "y": 0, "z": 0, "r1": 0.0, "r2": 0.0, "r3": 0.0, "is_relative": True},
    #         {"x": 0.0, "y": 0.1, "z": 0.1, "r1": 0.0, "r2": 0.0, "r3": 0.0, "is_relative": True},
    #     ]
    # robot.move_waypoints(points, cartesian_path=False)

    

    robot.move_to_home()


    robot.clear_trace()
    # robot.start_trace()

    inputStorage = None
    outputStorage = None
    if MODEL == "tx40":
        inputStorage = box1_staubli
        outputStorage = box2_staubli
    else:
        inputStorage = box1
        outputStorage = box2

    number_of_cards = 1

    if MODEL == "tx40":
        plates_dir = base_path / "platesStaubliTest"
    else:
        plates_dir = base_path / "plates3"

    # Iterate over all items in the 'plates' directory that start with 'plate'
    for plate_dir in plates_dir.glob("plate*"):

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
                                size_x=0.06, size_y=0.09, size_z=0.02,
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
                    win_logger.info(f'Testing reader: {reader.reader_name}')
                    for card in range(number_of_cards):
                        if card == 0 and reader_index !=0:
                            pass
                        else:
                            # win_logger.info(f'Robot is going to take card {card} by waypoints move')
                            # storage_points = [
                            #     { "x": inputStorage["position"]["x"], "y": inputStorage["position"]["y"], "z": inputStorage["position"]["z"]+0.1,
                            #         "r1": inputStorage["position"]["rx"], "r2": inputStorage["position"]["ry"], "r3": inputStorage["position"]["rz"],
                            #         "is_relative": False, "reference_frame": "WORLD" },
                            #     { "x": inputStorage["position"]["x"], "y": inputStorage["position"]["y"], "z": inputStorage["position"]["z"],
                            #         "r1": inputStorage["position"]["rx"], "r2": inputStorage["position"]["ry"], "r3": inputStorage["position"]["rz"],
                            #         "is_relative": False, "reference_frame": "WORLD" },
                            #     { "x": inputStorage["position"]["x"], "y": inputStorage["position"]["y"], "z": inputStorage["position"]["z"]+0.1,
                            #         "r1": inputStorage["position"]["rx"], "r2": inputStorage["position"]["ry"], "r3": inputStorage["position"]["rz"],
                            #         "is_relative": False, "reference_frame": "WORLD" },
                            # ]
                            
                            # robot.move_waypoints(
                            #     waypoints=storage_points,
                            #     rotation_format="RPY",
                            #     is_relative=False, 
                            #     cartesian_path=True
                            # )

                        
                            win_logger.info(f'Robot is going to take card {card} by move pose')


                            robot.move_to_pose(
                                x=inputStorage["position"]["x"],
                                y=inputStorage["position"]["y"],
                                z=inputStorage["position"]["z"] + 0.3 + OFFSET,
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
                                z=-0.3,
                                r1=0,
                                r2=0,
                                r3=0,
                                rotation_format="RPY",
                                reference_frame="WORLD",
                                is_relative=True,
                                cartesian_path=CARTESIAN_ALL,
                                execute=True
                            )

                            # robot.pump_grab()

                            # t = time.time()

                            # bool_grabbed = False
                            # while time.time() - t < 5:
                            #     print(f'at t={time.time() - t}: {robot.pump_is_grabbed()}')
                            #     if robot.pump_is_grabbed()["grabbed"]:
                            #         bool_grabbed = True
                            #         win_logger.info("Card is grabbed")
                            #         break
                            
                            # if not bool_grabbed:
                            #     robot.pump_release()

                            robot.move_to_pose(
                                x=0,
                                y=0,
                                z=0.3,
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
                            # if pos_index > 0:
                            #     dx = pos.x - reader.positions[pos_index-1].x
                            #     dy = pos.y - reader.positions[pos_index-1].y

                            #     robot.move_to_pose(
                            #         x=dx, y=dy, z=0,
                            #         r1=0, r2=0, r3=0,
                            #         rotation_format="RPY",
                            #         reference_frame="WORLD",
                            #         cartesian_path=True,
                            #         is_relative=True,
                            #         execute=True
                            #     )

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
                                    x=0, y=0, z=0.12 - 0.005,
                                    r1=0, r2=0, r3=0,
                                    rotation_format="RPY",
                                    reference_frame="TOOL",
                                    cartesian_path=CARTESIAN_ALL,
                                    is_relative=True,
                                    execute=True
                                )

                                robot.move_to_pose(
                                    x=0, y=0, z=-0.12 + 0.005,
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

                            # win_logger.info(f'Robot is going to release card: {card} by waypoints move')
                            # storage_points = [
                            #     { "x": outputStorage["position"]["x"], "y": outputStorage["position"]["y"], "z": outputStorage["position"]["z"]+0.3+0.005,
                            #         "r1": outputStorage["position"]["rx"], "r2": outputStorage["position"]["ry"], "r3": outputStorage["position"]["rz"],
                            #         "is_relative": False, "reference_frame": "WORLD" },
                            #     { "x": outputStorage["position"]["x"], "y": outputStorage["position"]["y"], "z": outputStorage["position"]["z"],
                            #         "r1": outputStorage["position"]["rx"], "r2": outputStorage["position"]["ry"], "r3": outputStorage["position"]["rz"],
                            #         "is_relative": False, "reference_frame": "WORLD" },
                            #     { "x": outputStorage["position"]["x"], "y": outputStorage["position"]["y"], "z": outputStorage["position"]["z"]+0.3+0.005,
                            #         "r1": outputStorage["position"]["rx"], "r2": outputStorage["position"]["ry"], "r3": outputStorage["position"]["rz"],
                            #         "is_relative": False, "reference_frame": "WORLD" },
                            # ]

                            # robot.move_waypoints(
                            #     waypoints=storage_points,
                            #     rotation_format="RPY",
                            #     is_relative=False, 
                            #     cartesian_path=True
                            # )

                            win_logger.info(f'Robot is going to release card: {card} by move pose')
                            
                            robot.move_to_pose(
                                x=outputStorage["position"]["x"],
                                y=outputStorage["position"]["y"],
                                z=outputStorage["position"]["z"] + 0.3 + OFFSET,
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
                                z=-0.3,
                                r1=0,
                                r2=0,
                                r3=0,
                                rotation_format="RPY",
                                reference_frame="WORLD",
                                is_relative=True,
                                cartesian_path=CARTESIAN_ALL,
                                execute=True
                            )

                            # robot.pump_release()

                            robot.move_to_pose(
                                x=0,
                                y=0,
                                z=0.3,
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
    
    print(robot.set_virtual_fence(enable=False))  

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
