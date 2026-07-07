import math
import random
import time
import csv
from datetime import datetime
from motion_http_client import MotionRobotClient
from math import pi
from plate import load_plate_from_file
from pathlib import Path
from logger_worker import tail_linux_logs, win_logger
import threading
import argparse


parser = argparse.ArgumentParser(description="Test file to command real robot")
parser.add_argument("--real-robot", action="store_false",
                    help="Connect to the real robot (default: simulation)")
parser.add_argument("--model", choices=["vs060", "vp5243", "tx40"], default="vs060",
                    help="Robot model to use (default: vs060)")
parser.add_argument("--accuracy-csv", default=None,
                    help="CSV file used to store desired and measured poses")

args = parser.parse_args()

SIM = args.real_robot
MODEL = args.model


STAUBLI_PLATE_OFFSET = 0.11
# STAUBLI_PLATE_OFFSET = 0.2

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
        "x": 0.05,
        "y": 0.2,
        "z": 0.0,
        "rx": 180*pi/180,
        "ry": 0,
        "rz": -90*pi/180
    }
}
box2_staubli = {
    "box_number": 2,
    "position": {
        "x": 0.05,
        "y": -0.2,
        "z": 0.0,
        "rx": 180*pi/180,
        "ry": 0,
        "rz": -90*pi/180
    }
}


base_path = Path.cwd()
rel_path_to_stl = "card_storage/boitier.STL"

# CSV layout
# -----------
# - movement       : the bridge function that was invoked
# - cartesian_path : True/False when the function exposes the flag, "" otherwise
# - reference_frame: WORLD/TOOL when the function exposes the flag, "" otherwise
# - is_relative    : True/False when the function exposes the flag, "" otherwise
# - velocity       : current velocity scaling at the time of the move, in percent
# - acceleration   : current acceleration scaling at the time of the move, in percent
# - desired_* : commanded pose or commanded joints
# - measured_* : pose/joints read back via /state/pose or /state/joints right after the move completes
CSV_FIELDNAMES = [
    "robot_model",
    "movement",
    "cartesian_path",
    "reference_frame",
    "is_relative",
    "velocity",
    "acceleration",
    "desired_x", "desired_y", "desired_z",
    "desired_roll", "desired_pitch", "desired_yaw",
    "measured_x", "measured_y", "measured_z",
    "measured_roll", "measured_pitch", "measured_yaw",
    # Joint data
    "desired_j1", "desired_j2", "desired_j3", "desired_j4", "desired_j5", "desired_j6",
    "measured_j1", "measured_j2", "measured_j3", "measured_j4", "measured_j5", "measured_j6"
]

from scipy.spatial.transform import Rotation as R
def convert_quat_targets_to_rpy(quat_targets):
    """
    Converts a list of pose dictionaries from Quaternion format to RPY.
    
    Expected input format:  r1=x, r2=y, r3=z, r4=w
    Output format:          r1=roll, r2=pitch, r3=yaw (r4 is removed)
    """
    rpy_targets = []
    
    for pose in quat_targets:
        # Create a copy to avoid modifying the original dictionary
        converted_pose = pose.copy()
        
        # Extract quaternion values (default w=1.0 for an unrotated state if missing)
        qx = converted_pose.get("r1", 0.0)
        qy = converted_pose.get("r2", 0.0)
        qz = converted_pose.get("r3", 0.0)
        qw = converted_pose.get("r4", 1.0)
        
        # Convert quaternion to Euler angles (extrinsic XYZ = Roll, Pitch, Yaw)
        rot = R.from_quat([qx, qy, qz, qw])
        roll, pitch, yaw = rot.as_euler('xyz', degrees=False)
        
        # Override the dictionary values with RPY
        converted_pose["r1"] = roll
        converted_pose["r2"] = pitch
        converted_pose["r3"] = yaw
        
        # Remove r4 as it is no longer needed in RPY format
        if "r4" in converted_pose:
            del converted_pose["r4"]
            
        rpy_targets.append(converted_pose)
        
    return rpy_targets


def _get_orientation_value(orientation, *keys):
    for key in keys:
        if key in orientation:
            return orientation[key]
    return ""


def _pose_to_row(prefix, pose):
    if not pose:
        return {
            f"{prefix}_x": "", f"{prefix}_y": "", f"{prefix}_z": "",
            f"{prefix}_roll": "", f"{prefix}_pitch": "", f"{prefix}_yaw": "",
        }

    position = pose.get("position", {})
    orientation = pose.get("orientation", pose.get("orientation_euler", {}))
    return {
        f"{prefix}_x": position.get("x", ""),
        f"{prefix}_y": position.get("y", ""),
        f"{prefix}_z": position.get("z", ""),
        f"{prefix}_roll": _get_orientation_value(orientation, "roll", "rx", "r1"),
        f"{prefix}_pitch": _get_orientation_value(orientation, "pitch", "ry", "r2"),
        f"{prefix}_yaw": _get_orientation_value(orientation, "yaw", "rz", "r3"),
    }

def _joints_to_row(prefix, joints_list):
    """Convert a list of joints to a dictionary row matching CSV headers."""
    row = {}
    for i in range(1, 7):
        row[f"{prefix}_j{i}"] = ""
        
    if joints_list:
        for i, val in enumerate(joints_list):
            if i < 6:
                row[f"{prefix}_j{i+1}"] = val
    return row

def _rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _mat_vec_mul(matrix, vector):
    return [
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    ]


def _rpy_from_kwargs(kwargs):
    r1 = kwargs.get("r1", 0.0)
    r2 = kwargs.get("r2", 0.0)
    r3 = kwargs.get("r3", 0.0)
    if kwargs.get("angle_format", "RAD").upper() == "DEG":
        return math.radians(r1), math.radians(r2), math.radians(r3)
    return r1, r2, r3


def _pose_from_values(x, y, z, roll, pitch, yaw):
    return {
        "position": {"x": x, "y": y, "z": z},
        "orientation": {"roll": roll, "pitch": pitch, "yaw": yaw},
    }


def _resolve_move_to_pose_target(robot, kwargs):
    """Compute the absolute desired pose for a move_to_pose / move_to_pose_via_joint call."""
    x = kwargs.get("x", 0.0)
    y = kwargs.get("y", 0.0)
    z = kwargs.get("z", 0.0)
    roll, pitch, yaw = _rpy_from_kwargs(kwargs)
    reference_frame = kwargs.get("reference_frame", "WORLD").upper()
    is_relative = kwargs.get("is_relative", False)

    if kwargs.get("rotation_format", "RPY").upper() != "RPY":
        return None

    if not is_relative:
        return _pose_from_values(x, y, z, roll, pitch, yaw)

    current_pose = robot.get_current_pose(output_format="euler")
    position = current_pose.get("position", {})
    orientation = current_pose.get("orientation", {})
    current_roll = _get_orientation_value(orientation, "roll", "rx")
    current_pitch = _get_orientation_value(orientation, "pitch", "ry")
    current_yaw = _get_orientation_value(orientation, "yaw", "rz")

    if reference_frame == "TOOL":
        dx, dy, dz = _mat_vec_mul(_rpy_to_matrix(current_roll, current_pitch, current_yaw), [x, y, z])
    else:
        dx, dy, dz = x, y, z

    return _pose_from_values(
        position.get("x", 0.0) + dx,
        position.get("y", 0.0) + dy,
        position.get("z", 0.0) + dz,
        current_roll + roll,
        current_pitch + pitch,
        current_yaw + yaw,
    )

def _resolve_move_joints_target(robot, kwargs):
    """Compute the absolute desired joints for a move_joints call (returns radians)."""
    target_joints = kwargs.get("joints")
    if not target_joints:
        return None

    angle_format = kwargs.get("angle_format", "RAD").upper()
    if angle_format == "DEG":
        target_joints_rad = [math.radians(j) for j in target_joints]
    else:
        target_joints_rad = list(target_joints)

    is_relative = kwargs.get("is_relative", False)
    if not is_relative:
        return target_joints_rad

    # Handle relative joint move by fetching current joints
    try:
        state = robot.get_joint_state()
        current_joints = state.get("joints", [])
        return [c + t for c, t in zip(current_joints, target_joints_rad)]
    except Exception as e:
        win_logger.error(f"Could not get current joints for relative move: {e}")
        return None

def _resolve_approach_target(robot, kwargs):
    """Compute the absolute (WORLD) approach pose using the bridge's compute_approach_pose."""
    if kwargs.get("rotation_format", "RPY").upper() != "RPY":
        return None

    approach_pose = robot.compute_approach_pose(
        x=kwargs.get("x", 0.0),
        y=kwargs.get("y", 0.0),
        z=kwargs.get("z", 0.0),
        r1=kwargs.get("r1", 0.0),
        r2=kwargs.get("r2", 0.0),
        r3=kwargs.get("r3", 0.0),
        r4=kwargs.get("r4", 0.0),
        rotation_format=kwargs.get("rotation_format", "RPY"),
        angle_format=kwargs.get("angle_format", "RAD"),
        z_offset=kwargs.get("z_offset", 0.1),
    )
    return {
        "position": approach_pose.get("position", {}),
        "orientation": approach_pose.get("orientation_euler", {}),
    }


class MovementCsvLogger:
    def __init__(self, csv_path, robot_model):
        self.csv_path = Path(csv_path)
        self.robot_model = robot_model
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._ensure_header()

    def _ensure_header(self):
        if self.csv_path.exists() and self.csv_path.stat().st_size > 0:
            return

        with self.csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
            writer.writeheader()

    def log(self, robot, function, desired_pose=None, desired_joints=None,
            reference_frame="", is_relative="", cartesian_path=""):
        """Append one row to the CSV."""
        try:
            scaling = robot.get_scaling()
            velocity = round(float(scaling.get("velocity_scale", 0.0)) * 100)
            acceleration = round(float(scaling.get("accel_scale", 0.0)) * 100)
        except Exception as exc:
            velocity = ""
            acceleration = ""
            win_logger.error(f"Could not read scaling for {function}: {exc}")

        row = {
            "robot_model": self.robot_model,
            "movement": function,
            "cartesian_path": cartesian_path,
            "reference_frame": reference_frame,
            "is_relative": is_relative,
            "velocity": velocity,
            "acceleration": acceleration,
        }
        
        # Add Cartesian Data
        row.update(_pose_to_row("desired", desired_pose))
        try:
            measured_pose = robot.get_current_pose(output_format="euler")
            row.update(_pose_to_row("measured", measured_pose))
        except Exception as exc:
            row.update(_pose_to_row("measured", None))
            win_logger.error(f"Could not measure pose after {function}: {exc}")

        # Add Joint Data
        row.update(_joints_to_row("desired", desired_joints))
        try:
            measured_state = robot.get_joint_state()
            row.update(_joints_to_row("measured", measured_state.get("joints")))
        except Exception as exc:
            row.update(_joints_to_row("measured", None))
            win_logger.error(f"Could not measure joints after {function}: {exc}")


        with self.csv_path.open("a", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
            writer.writerow(row)


# ----------------------------------------------------------------------
# Logged wrappers around the bridge client.
# ----------------------------------------------------------------------

def move_to_pose_logged(robot, csv_logger, **kwargs):
    try:
        desired_pose = _resolve_move_to_pose_target(robot, kwargs)
    except Exception as exc:
        desired_pose = None
        win_logger.error(f"Could not resolve desired pose for move_to_pose: {exc}")

    result = robot.move_to_pose(**kwargs)
    csv_logger.log(
        robot,
        function="move_to_pose",
        desired_pose=desired_pose,
        reference_frame=kwargs.get("reference_frame", "WORLD"),
        is_relative=kwargs.get("is_relative", False),
        cartesian_path=kwargs.get("cartesian_path", False),
    )
    return result


def move_to_pose_via_joint_logged(robot, csv_logger, **kwargs):
    try:
        desired_pose = _resolve_move_to_pose_target(robot, kwargs)
    except Exception as exc:
        desired_pose = None
        win_logger.error(f"Could not resolve desired pose for move_to_pose_via_joint: {exc}")

    result = robot.move_to_pose_via_joint(**kwargs)
    csv_logger.log(
        robot,
        function="move_to_pose_via_joint",
        desired_pose=desired_pose,
        reference_frame=kwargs.get("reference_frame", "WORLD"),
        is_relative=kwargs.get("is_relative", False),
        cartesian_path=False,
    )
    return result


def move_approach_logged(robot, csv_logger, **kwargs):
    try:
        desired_pose = _resolve_approach_target(robot, kwargs)
    except Exception as exc:
        desired_pose = None
        win_logger.error(f"Could not resolve desired approach pose: {exc}")

    result = robot.move_approach(**kwargs)
    csv_logger.log(
        robot,
        function="move_approach",
        desired_pose=desired_pose,
        reference_frame="WORLD",
        is_relative=False,
        cartesian_path=kwargs.get("cartesian_path", False),
    )
    return result


def move_joints_logged(robot, csv_logger, **kwargs):
    try:
        desired_joints = _resolve_move_joints_target(robot, kwargs)
    except Exception as exc:
        desired_joints = None
        win_logger.error(f"Could not resolve desired joints: {exc}")

    result = robot.move_joints(**kwargs)
    csv_logger.log(
        robot,
        function="move_joints",
        desired_pose=None,
        desired_joints=desired_joints,
        reference_frame="",
        is_relative=kwargs.get("is_relative", False),
        cartesian_path="",
    )
    return result


def to_path_real(input):
    abs_path_to_stl = base_path / input

    posix_path = abs_path_to_stl.as_posix()

    if posix_path.lower().startswith("c:/"):
        wsl_path = "/mnt/c/" + posix_path[3:]
    else:
        wsl_path = posix_path

    return f"file://{wsl_path}"


def test(robot, csv_logger):
    robot.move_to_home()
    pose_targets_abs_world_rpy = [
        {"x": 0.3,  "y": 0.1, "z": 0.22, "r1": pi, "r2":  0.00, "r3":  0},
        {"x": 0.4,  "y": 0.1, "z": 0.32, "r1": pi, "r2":  0.00, "r3":  0},
        {"x": 0.4,  "y": -0.1, "z": 0.22, "r1": pi, "r2":  0.00, "r3":  0},
        {"x": 0.3,  "y": 0, "z": 0.22, "r1": pi, "r2":  0.00, "r3":  0},
    ]
    
    for target in pose_targets_abs_world_rpy:
        move_to_pose_logged(
            robot, csv_logger,
            x=target["x"], y=target["y"], z=target["z"],
            r1=target["r1"], r2=target["r2"], r3=target["r3"],
            rotation_format="RPY", reference_frame="WORLD",
            is_relative=False, cartesian_path=False
        )
    for target in pose_targets_abs_world_rpy:
        move_to_pose_logged(
            robot, csv_logger,
            x=target["x"], y=target["y"], z=target["z"],
            r1=target["r1"], r2=target["r2"], r3=target["r3"],
            rotation_format="RPY", reference_frame="WORLD",
            is_relative=False, cartesian_path=True
        )

    
    robot.move_to_home()

    pose_targets_relative_world_rpy = [
        {"x": 0,  "y": 0.1, "z": 0, "r1": 0, "r2":  0.00, "r3":  0},
        {"x": 0.2,  "y": 0, "z": -0.2, "r1": 0, "r2":  0.00, "r3":  0},
        {"x": 0,  "y": -0.2, "z": 0.15, "r1": 0, "r2":  0.00, "r3":  0},
        {"x": -0.2,  "y": 0, "z": 0, "r1": 0, "r2":  0.00, "r3":  0},
    ]
    
    for target in pose_targets_relative_world_rpy:
        move_to_pose_logged(
            robot, csv_logger,
            x=target["x"], y=target["y"], z=target["z"],
            r1=target["r1"], r2=target["r2"], r3=target["r3"],
            rotation_format="RPY", reference_frame="WORLD",
            is_relative=True, cartesian_path=False
        )
    for target in pose_targets_relative_world_rpy:
        move_to_pose_logged(
            robot, csv_logger,
            x=target["x"], y=target["y"], z=target["z"],
            r1=target["r1"], r2=target["r2"], r3=target["r3"],
            rotation_format="RPY", reference_frame="WORLD",
            is_relative=True, cartesian_path=True
        )

    # ---------------------------------------------------------
    # 6. JOINT MOVEMENTS
    # ---------------------------------------------------------
    HOME = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]

    RELATIVE_MOVES = [
        [ 0.25,  0.00, -0.20,  0.15, -0.20,  0.10],
        [-0.25,  0.20, -0.15, -0.15, -0.25, -0.10],
        [ 0.00, -0.25, -0.25,  0.25, -0.15,  0.20],
        [ 0.30,  0.20, -0.30, -0.20, -0.30,  0.15],
        [-0.30, -0.20, -0.20,  0.20, -0.25, -0.15],
    ]

    ABSOLUTE_MOVES = [
        [ 0.25,  0.00, 1.37,  0.15, 1.37,  0.10],
        [-0.25,  0.20, 1.42, -0.15, 1.32, -0.10],
        [ 0.00, -0.25, 1.32,  0.25, 1.42,  0.20],
        [ 0.30,  0.20, 1.27, -0.20, 1.27,  0.15],
        [-0.30, -0.20, 1.37,  0.20, 1.32, -0.15],
    ]


    win_logger.info("--- Hardcoded RELATIVE moves around HOME ---")

    for i, delta in enumerate(RELATIVE_MOVES):


        move_joints_logged(
            robot,
            csv_logger,
            joints=HOME,
            angle_format="RAD",
            is_relative=False,
        )

        move_joints_logged(
            robot,
            csv_logger,
            joints=delta,
            angle_format="RAD",
            is_relative=True,
        )

        move_joints_logged(
            robot,
            csv_logger,
            joints=HOME,
            angle_format="RAD",
            is_relative=False,
        )
        
    for i, target in enumerate(ABSOLUTE_MOVES):

        move_joints_logged(
            robot,
            csv_logger,
            joints=target,
            angle_format="RAD",
            is_relative=False,
        )

        # Retour HOME après chaque test.
        move_joints_logged(
            robot,
            csv_logger,
            joints=HOME,
            angle_format="RAD",
            is_relative=False,
        )
    


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
    if args.accuracy_csv:
        csv_path = Path(args.accuracy_csv)
        if not csv_path.is_absolute():
            csv_path = base_path / csv_path
    else:
        csv_name = f"accuracy_{datetime.now().strftime('%Y-%m-%d_%H-%M')}.csv"
        csv_path = base_path / "accuracy/data" / csv_name
    csv_logger = MovementCsvLogger(csv_path, MODEL)

    win_logger.info(f"Health: {robot.health()}")
    win_logger.info(f"Initialising robot")
    robot.init_robot(model=MODEL, 
                           velocity_scale=0.1, 
                           accel_scale=0.1, 
                           planning_time=5, 
                           planning_attempts=20, 
                           allow_replanning=True, 
                           planner_id="RRTConnect")

    robot.set_scaling(velocity_scale=0.4, accel_scale=0.1)

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

    robot.set_virtual_cage(
        enable=True, 
        front=0.66, back=0.35, 
        left=0.325, right=0.325, 
        top=0.9, bottom=0.0
    )
    time.sleep(2)

    
    print(robot.set_servo_on(True))

    
    test(robot, csv_logger)
   
    
    print(robot.set_servo_on(False))
    
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