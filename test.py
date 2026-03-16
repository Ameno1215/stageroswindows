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
        x=box1["position"]["x"], y=box1["position"]["y"], z=box1["position"]["z"],
        r1=0.0, r2=0.0, r3=0.0,
        scale_x=0.001, scale_y=0.001, scale_z=0.001,
        rotation_format="RPY",
        a=1, r=1, g=0, b=0,
        action="ADD"
    ))

    print(robot.manage_mesh(
        mesh_id="box2",
        mesh_path=to_path_real(rel_path_to_stl),
        x=box2["position"]["x"], y=box2["position"]["y"], z=box2["position"]["z"],
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
    
    inputStorage = box1
    outputStorage = box2
    number_of_cards = 2



    plates_dir = base_path / "plates"

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
                        print(robot.manage_box(
                            box_id=f"{reader.reader_name}_{pos.position_label}",
                            x=pos.x, y=pos.y, z=pos.z,
                            r1=pos.rx, r2=pos.ry, r3=pos.rz,
                            size_x=0.1, size_y=0.1, size_z=0.05,
                            action="ADD",
                            enable_collision=False
                        ))
                
                # print(robot.manage_mesh(
                #     mesh_id=f"plaque{plate.plate_number}",
                #     mesh_path=to_path_real(plate.mesh_path),
                #     x=0.557+0.135/2, y=-0.25, z=0,
                #     r1=pi/180*plate.mesh_rotation_x, r2=pi/180*plate.mesh_rotation_y, r3=pi/180*plate.mesh_rotation_z,
                #     rotation_format="RPY",
                #     a=1, r=0, g=1, b=0,
                #     action="ADD"
                # ))       

                for reader_index, reader in enumerate(plate.readers):
                    win_logger.info(f'Testing reader: {reader.reader_name}')
                    for card in range(number_of_cards):
                        if card == 0 and reader_index !=0:
                            pass
                        else:
                            win_logger.info(f'Robot is going to take card: {card}')
                            # storage_points = [
                            #     { "x": inputStorage["position"]["x"], "y": inputStorage["position"]["y"], "z": inputStorage["position"]["z"]+0.3,
                            #         "r1": inputStorage["position"]["rx"], "r2": inputStorage["position"]["ry"], "r3": inputStorage["position"]["rz"],
                            #         "is_relative": False, "reference_frame": "WORLD" },
                            #     { "x": inputStorage["position"]["x"], "y": inputStorage["position"]["y"], "z": inputStorage["position"]["z"],
                            #         "r1": inputStorage["position"]["rx"], "r2": inputStorage["position"]["ry"], "r3": inputStorage["position"]["rz"],
                            #         "is_relative": False, "reference_frame": "WORLD" },
                            # ]
                            
                            # robot.move_waypoints(
                            #     waypoints=storage_points,
                            #     rotation_format="RPY",
                            #     is_relative=False, 
                            #     cartesian_path=True
                            # )

                            robot.move_to_pose(
                                x=inputStorage["position"]["x"],
                                y=inputStorage["position"]["y"],
                                z=inputStorage["position"]["z"] + 0.3 + 0.005,
                                r1=inputStorage["position"]["rx"],
                                r2=inputStorage["position"]["ry"],
                                r3=inputStorage["position"]["rz"],
                                rotation_format="RPY",
                                reference_frame="WORLD",
                                is_relative=False,
                                cartesian_path=True,
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
                                cartesian_path=True,
                                execute=True
                            )


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
                                cartesian_path=True,
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

                            robot.move_approach(
                                x=pos.x, y=pos.y, z=pos.z,
                                r1=pos.rx, r2=pos.ry, r3=pos.rz,
                                z_offset=0.12,
                                rotation_format="RPY",
                                cartesian_path=True,
                                execute=True
                            )

                            robot.move_to_pose(
                                x=0, y=0, z=0.12,
                                r1=0, r2=0, r3=0,
                                rotation_format="RPY",
                                reference_frame="TOOL",
                                cartesian_path=True,
                                is_relative=True,
                                execute=True
                            )

                            robot.move_to_pose(
                                x=0, y=0, z=-0.12,
                                r1=0, r2=0, r3=0,
                                rotation_format="RPY",
                                reference_frame="TOOL",
                                cartesian_path=True,
                                is_relative=True,
                                execute=True
                            )

                            
                            if pos_index > 0:
                                pass
                                # peut etre si au retour de la position 2 ca fait un truc bizarre

                        if card == number_of_cards-1 and reader_index != len(plate.readers)-1:
                            win_logger.info("Robot don't release the card to gain time")
                            pass
                        else:
                            # robot.move_to_pose(
                            #     x=outputStorage["position"]["x"],
                            #     y=outputStorage["position"]["y"],
                            #     z=outputStorage["position"]["z"]+0.3,
                            #     r1=outputStorage["position"]["rx"],
                            #     r2=outputStorage["position"]["ry"],
                            #     r3=outputStorage["position"]["rz"],
                            #     rotation_format="RPY",
                            #     reference_frame="WORLD",
                            #     cartesian_path=False,
                            #     execute=True
                            # )

                            win_logger.info(f'Robot is going to release card: {card}')
                            # storage_points = [
                            #     { "x": outputStorage["position"]["x"], "y": outputStorage["position"]["y"], "z": outputStorage["position"]["z"]+0.3,
                            #         "r1": outputStorage["position"]["rx"], "r2": outputStorage["position"]["ry"], "r3": outputStorage["position"]["rz"],
                            #         "is_relative": False, "reference_frame": "WORLD" },
                            #     { "x": outputStorage["position"]["x"], "y": outputStorage["position"]["y"], "z": outputStorage["position"]["z"],
                            #         "r1": outputStorage["position"]["rx"], "r2": outputStorage["position"]["ry"], "r3": outputStorage["position"]["rz"],
                            #         "is_relative": False, "reference_frame": "WORLD" },
                            # ]

                            # robot.move_waypoints(
                            #     waypoints=storage_points,
                            #     rotation_format="RPY",
                            #     is_relative=False, 
                            #     cartesian_path=True
                            # )

                            robot.move_to_pose(
                                x=outputStorage["position"]["x"],
                                y=outputStorage["position"]["y"],
                                z=outputStorage["position"]["z"] + 0.3  + 0.005,
                                r1=outputStorage["position"]["rx"],
                                r2=outputStorage["position"]["ry"],
                                r3=outputStorage["position"]["rz"],
                                rotation_format="RPY",
                                reference_frame="WORLD",
                                is_relative=False,
                                cartesian_path=True,
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
                                cartesian_path=True,
                                execute=True
                            )

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
                                cartesian_path=True,
                                execute=True
                            )
                    

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

                # print(robot.manage_mesh(
                #     mesh_id=f"plaque{plate.plate_number}",
                #     action="REMOVE"
                # ))

    time.sleep(2)


    
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
    run()
