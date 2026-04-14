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
parser.add_argument("--model", choices=["vs060", "vp5243"], default="vs060",
                    help="Robot model to use (default: vs060)")


args = parser.parse_args()
MODEL = args.model
SIM = args.real_robot

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
                           planning_group="arm", 
                           velocity_scale=0.05, 
                           accel_scale=0.05, 
                           planning_time=10, 
                           planning_attempts=20, 
                           allow_replanning=True, 
                           planner_id="RRTConnect"))
    
    robot.set_servo_on(False)

    # robot.pump_release()
    # return

    
    print(robot.pump_grab())

    t = time.time()

    while time.time() - t < 5:
        print(f'at t={time.time() - t}: {robot.pump_is_grabbed()}')
        if robot.pump_is_grabbed()["grabbed"]:
            win_logger.info("Card is grabbed")
            break
    
    print(robot.pump_release())

    return
    
    robot.move_to_home()


    for i in range(10):
        robot.move_to_pose(
            0.2, 0.0, 0,
            0, 0.0, 0.0,
            rotation_format="RPY",
            angle_format="DEG",
            is_relative=True,
            joint_constraints=[
                {"joint_name": "joint_4", "min": -2, "max": 2, "relative": True},
            ]
        )

        robot.move_to_pose(
            -0.2, 0.0, 0,
            0, 0.0, 0.0,
            rotation_format="RPY",
            angle_format="DEG",
            is_relative=True,
            joint_constraints=[
                {"joint_name": "joint_4", "min": -2, "max": 2, "relative": True},
            ]
        )

    return

    # ─── 2. Single absolute constraint in degrees ───
    win_logger.info("Test 2: Single absolute constraint (DEG)")
    robot.move_to_pose(
        0.3, 0.2, 0.35,
        180, 0, 30,
        rotation_format="RPY",
        angle_format="DEG",
        joint_constraints=[
            {"joint_name": "joint_1", "min": -45, "max": 45},
        ]
    )

    # ─── 3. Multiple absolute constraints ───
    win_logger.info("Test 3: Multiple absolute constraints (RAD)")
    robot.move_to_pose(
        0.3, 0.2, 0.35,
        3.14, 0.0, 0.5,
        rotation_format="RPY",
        joint_constraints=[
            {"joint_name": "joint_4", "min": -0.1, "max": 0.1},
            {"joint_name": "joint_6", "min": -1.57, "max": 1.57},
        ]
    )

    # ─── 4. ALL 6 joints constrained (absolute, degrees) ───
    win_logger.info("Test 4: All 6 joints constrained (DEG)")
    robot.move_to_pose(
        0.35, 0.0, 0.45,
        180, 0, 0,
        rotation_format="RPY",
        angle_format="DEG",
        joint_constraints=[
            {"joint_name": "joint_1", "min": -90,  "max": 90},
            {"joint_name": "joint_2", "min": -57,  "max": 57},
            {"joint_name": "joint_3", "min": 0,    "max": 115},
            {"joint_name": "joint_4", "min": -115, "max": 115},
            {"joint_name": "joint_5", "min": 29,   "max": 143},
            {"joint_name": "joint_6", "min": -180, "max": 180},
        ]
    )

    # ═══════════════════════════════════════════════════════════════════
    #  RELATIVE CONSTRAINTS (relative=True → offset from current pos)
    # ═══════════════════════════════════════════════════════════════════

    # ─── 5. Single relative constraint in radians ───
    #   joint_6 can only move ±0.5 rad from wherever it is now
    win_logger.info("Test 5: Relative constraint (RAD)")
    robot.move_to_pose(
        0.4, 0.0, 0.4,
        3.14, 0.0, 0.0,
        rotation_format="RPY",
        joint_constraints=[
            {"joint_name": "joint_6", "min": -0.5, "max": 0.5, "relative": True},
        ]
    )

    # ─── 6. Single relative constraint in degrees ───
    #   joint_1 can only move ±20° from its current position
    win_logger.info("Test 6: Relative constraint (DEG)")
    robot.move_to_pose(
        0.3, 0.15, 0.4,
        180, 0, 0,
        rotation_format="RPY",
        angle_format="DEG",
        joint_constraints=[
            {"joint_name": "joint_1", "min": -20, "max": 20, "relative": True},
        ]
    )

    # ═══════════════════════════════════════════════════════════════════
    #  MIXED: absolute + relative in the same call
    # ═══════════════════════════════════════════════════════════════════

    # ─── 7. Mix absolute and relative constraints ───
    #   joint_1: absolute [-45°, +45°]
    #   joint_6: relative ±30° from current
    win_logger.info("Test 7: Mixed absolute + relative (DEG)")
    robot.move_to_pose(
        0.35, 0.1, 0.4,
        180, 0, 15,
        rotation_format="RPY",
        angle_format="DEG",
        joint_constraints=[
            {"joint_name": "joint_1", "min": -45, "max": 45, "relative": False},
            {"joint_name": "joint_6", "min": -30, "max": 30, "relative": True},
        ]
    )

    # ═══════════════════════════════════════════════════════════════════
    #  CONSTRAINTS ON OTHER MOTION FUNCTIONS
    # ═══════════════════════════════════════════════════════════════════

    # ─── 8. move_joints with relative constraint ───
    win_logger.info("Test 8: move_joints + relative constraint")
    robot.move_joints(
        [0.0, 0.0, 1.57, 0.0, 1.57, 0.0],
        joint_constraints=[
            {"joint_name": "joint_1", "min": -0.5, "max": 0.5, "relative": True},
        ]
    )

    # ─── 9. move_joints with absolute constraint in degrees ───
    win_logger.info("Test 9: move_joints + absolute constraint (DEG)")
    robot.move_joints(
        [0, 0, 90, 0, 90, 0],
        angle_format="DEG",
        joint_constraints=[
            {"joint_name": "joint_1", "min": -45, "max": 45},
        ]
    )

    # ─── 10. move_approach with relative wrist constraint ───
    win_logger.info("Test 10: move_approach + relative constraint")
    robot.move_approach(
        0.4, 0.15, 0.02,
        3.14, 0.0, 0.0,
        rotation_format="RPY",
        z_offset=0.1,
        cartesian_path=False,
        joint_constraints=[
            {"joint_name": "joint_6", "min": -0.785, "max": 0.785, "relative": True},
        ]
    )

    # ─── 11. move_to_pose_via_joint with mixed constraints (DEG) ───
    win_logger.info("Test 11: move_to_pose_via_joint + mixed constraints (DEG)")
    robot.move_to_pose_via_joint(
        0.5, 0.0, 0.4,
        180, 0, 0,
        rotation_format="RPY",
        angle_format="DEG",
        joint_constraints=[
            {"joint_name": "joint_1", "min": -60, "max": 60, "relative": False},
            {"joint_name": "joint_3", "min": -15, "max": 15, "relative": True},
        ]
    )

    # ═══════════════════════════════════════════════════════════════════
    #  BACKWARDS COMPATIBILITY
    # ═══════════════════════════════════════════════════════════════════

    # ─── 12. No constraints at all ───
    win_logger.info("Test 12: No constraints (backwards compatible)")
    robot.move_to_pose(0.4, 0.0, 0.4, 3.14, 0.0, 0.0, rotation_format="RPY")

    # ─── 13. Cartesian path — constraints are ignored ───
    win_logger.info("Test 13: Cartesian path — constraints ignored")
    robot.move_to_pose(
        0.4, 0.0, 0.4,
        3.14, 0.0, 0.0,
        rotation_format="RPY",
        cartesian_path=True,
        joint_constraints=[
            {"joint_name": "joint_1", "min": -0.5, "max": 0.5},
        ]
    )

    # ─── 14. Relative constraint on a move that is itself relative ───
    #   Motion is relative (TOOL +10cm Z), constraint is relative (joint_6 ±10°)
    #   The two "relative" concepts are independent
    win_logger.info("Test 14: Relative motion + relative constraint (DEG)")
    robot.move_to_pose(
        0.0, 0.0, 0.10,
        0, 0, 0,
        rotation_format="RPY",
        angle_format="DEG",
        reference_frame="TOOL",
        is_relative=True,
        joint_constraints=[
            {"joint_name": "joint_6", "min": -10, "max": 10, "relative": True},
        ]
    )

    win_logger.info("All constraint tests completed successfully!")


if __name__ == "__main__":
    try:
        run()
    except RuntimeError as e:
        win_logger.error(f"Program stopped due to motion error: {e}")
    except KeyboardInterrupt:
        win_logger.info("Program interrupted by user.")
    except Exception as e:
        win_logger.error(f"Unexpected error: {e}")