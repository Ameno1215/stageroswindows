"""
test_motion_server.py
─────────────────────
Stress-test suite for the MotionServer ROS node via the HTTP client.
Each function is independent: call them individually or via run_all_tests().

Virtual cage is ALWAYS active during tests:
  front=0.66  back=0.35  left=0.325  right=0.325  top=0.9  bottom=0.0

Robot: vs060 / planning_group=arm
"""

import time
import math
import threading
from motion_http_client import MotionRobotClient
from logger_worker import tail_linux_logs, win_logger


# ─────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────

CAGE = dict(
    enable=True,
    front=0.66, back=0.35,
    left=0.325, right=0.325,
    top=0.9,    bottom=0.0,
)

HOME_JOINTS = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]


def _log_result(test_name: str, result: dict):
    ok = result.get("success", False)
    msg = result.get("message", "")
    status = "PASS" if ok else "FAIL"
    win_logger.info(f"[{test_name}] {status} — {msg}")
    return ok


def _setup(robot: MotionRobotClient):
    """Re-enable cage and go home before each test."""
    robot.set_virtual_cage(**CAGE)
    robot.move_joints(HOME_JOINTS, is_relative=False)
    time.sleep(0.5)


# ─────────────────────────────────────────────
# 1. IK NON-REACHABLE
#    Send the robot to a pose that is clearly outside its workspace.
#    Expects success=False with an IK failure message.
# ─────────────────────────────────────────────

def test_ik_non_reachable(robot: MotionRobotClient):
    """
    Target is 3 m in front of the robot: completely out of workspace.
    Both Pose-target planning and the IK+joint fallback must fail.
    """
    win_logger.info("─── test_ik_non_reachable ───")
    _setup(robot)

    result = robot.move_to_pose(
        x=3.0, y=0.0, z=0.5,
        r1=0.0, r2=0.0, r3=0.0,
        rotation_format="RPY",
        is_relative=False,
        cartesian_path=False,
        execute=False,      # dry-run: no need to actually move
    )
    _log_result("ik_non_reachable", result)


# ─────────────────────────────────────────────
# 2. MOVE POSE  (joint-space, absolute)
#    Move to a few known-reachable absolute poses.
# ─────────────────────────────────────────────

def test_move_pose(robot: MotionRobotClient):
    """
    Sequence of absolute joint-space pose targets.
    Tests nominal planning + execution on the happy path.
    """
    win_logger.info("─── test_move_pose ───")
    _setup(robot)

    targets = [
        # (x,    y,     z,    roll,        pitch,       yaw,  label)
        (0.35,  0.10,  0.45,  0.0,          0.0,         0.0,   "center-right"),
        (0.35, -0.10,  0.45,  0.0,          0.0,         0.0,   "center-left"),
        (0.40,  0.0,   0.60,  0.0,          math.pi/6,   0.0,   "high-tilted"),
        (0.25,  0.0,   0.30,  math.pi/4,    0.0,         0.0,   "low-rolled"),
        (0.50,  0.15,  0.50, -math.pi/6,    math.pi/4,   0.3,   "skewed"),
    ]

    for x, y, z, r1, r2, r3, label in targets:
        win_logger.info(f"  → pose: {label}")
        result = robot.move_to_pose(
            x=x, y=y, z=z,
            r1=r1, r2=r2, r3=r3,
            rotation_format="RPY",
            is_relative=False,
            cartesian_path=False,
            execute=True,
        )
        _log_result(f"move_pose/{label}", result)
        time.sleep(0.3)


# ─────────────────────────────────────────────
# 3. MOVE POSE RELATIVE  (WORLD and TOOL frames)
# ─────────────────────────────────────────────

def test_move_pose_relative(robot: MotionRobotClient):
    """
    Relative incremental moves from the current position.
    Tests WORLD-frame and TOOL-frame relative motion.
    """
    win_logger.info("─── test_move_pose_relative ───")
    _setup(robot)

    # --- WORLD-frame increments ---
    world_moves = [
        (  0.05,  0.0,   0.0,  0.0, 0.0, 0.0, "WORLD +X 5cm"),
        (  0.0,   0.05,  0.0,  0.0, 0.0, 0.0, "WORLD +Y 5cm"),
        (  0.0,   0.0,   0.05, 0.0, 0.0, 0.0, "WORLD +Z 5cm"),
        ( -0.05,  0.0,   0.0,  0.0, 0.0, 0.0, "WORLD -X 5cm"),
        (  0.0,  -0.05,  0.0,  0.0, 0.0, 0.0, "WORLD -Y 5cm"),
        (  0.0,   0.0,  -0.05, 0.0, 0.0, 0.0, "WORLD -Z 5cm"),
    ]
    for x, y, z, r1, r2, r3, label in world_moves:
        result = robot.move_to_pose(
            x=x, y=y, z=z,
            r1=r1, r2=r2, r3=r3,
            rotation_format="RPY",
            reference_frame="WORLD",
            is_relative=True,
            cartesian_path=False,
            execute=True,
        )
        _log_result(f"move_pose_relative/{label}", result)
        time.sleep(0.2)

    _setup(robot)   # back home before TOOL-frame tests

    # --- TOOL-frame increments (fly-by-wire style) ---
    tool_moves = [
        (0.05,  0.0,  0.0, 0.0, 0.0, 0.0, "TOOL +X 5cm"),
        (0.0,   0.05, 0.0, 0.0, 0.0, 0.0, "TOOL +Y 5cm"),
        (0.0,   0.0,  0.05,0.0, 0.0, 0.0, "TOOL +Z 5cm"),
        (0.0,   0.0,  0.0, 0.0, 0.0, 0.2, "TOOL yaw +0.2rad"),
    ]
    for x, y, z, r1, r2, r3, label in tool_moves:
        result = robot.move_to_pose(
            x=x, y=y, z=z,
            r1=r1, r2=r2, r3=r3,
            rotation_format="RPY",
            reference_frame="TOOL",
            is_relative=True,
            cartesian_path=False,
            execute=True,
        )
        _log_result(f"move_pose_relative/{label}", result)
        time.sleep(0.2)


# ─────────────────────────────────────────────
# 4. MOVE POSE — FALLBACK (joint-space)
#    Feed poses that are valid but awkward so the standard planner
#    struggles and falls back to IK + joint-space planning.
# ─────────────────────────────────────────────

def test_move_pose_fallback(robot: MotionRobotClient):
    """
    Near-singular or wrist-flipped poses that may cause the standard
    pose-target planner to fail and trigger the IK+joint fallback.
    Success is still expected — just via the fallback path.
    """
    win_logger.info("─── test_move_pose_fallback ───")
    _setup(robot)

    # Straight-down config — close to a wrist singularity
    tricky_poses = [
        (0.40,  0.0,  0.20, 0.0,  math.pi/2, 0.0,  "wrist-singularity-down"),
        (0.30,  0.0,  0.55, 0.0, -math.pi/2, 0.0,  "wrist-singularity-up"),
        (0.55,  0.0,  0.30, math.pi, 0.0, 0.0,      "roll-180-deg"),
        (0.50,  0.20, 0.40, math.pi/2, math.pi/4, -math.pi/3, "complex-RPY"),
    ]

    for x, y, z, r1, r2, r3, label in tricky_poses:
        win_logger.info(f"  → tricky pose: {label}")
        result = robot.move_to_pose(
            x=x, y=y, z=z,
            r1=r1, r2=r2, r3=r3,
            rotation_format="RPY",
            is_relative=False,
            cartesian_path=False,
            execute=True,
        )
        _log_result(f"move_pose_fallback/{label}", result)
        _setup(robot)


# ─────────────────────────────────────────────
# 5. MOVE POSE — FALLBACK CARTESIAN
#    A Cartesian path that goes through a configuration the planner
#    finds hard (e.g. approaching singularity mid-path) so fraction<0.95
#    triggers the diagnostic path.  Not necessarily expected to succeed.
# ─────────────────────────────────────────────

def test_move_pose_fallback_cartesian(robot: MotionRobotClient):
    """
    Force the Cartesian planner to produce a low fraction by targeting
    a pose near the workspace boundary. The diagnostic + failure message
    is the interesting output here.
    """
    win_logger.info("─── test_move_pose_fallback_cartesian ───")
    _setup(robot)

    # Very large single-step Cartesian jump — likely to fail fraction check
    result = robot.move_to_pose(
        x=0.62, y=0.0, z=0.15,
        r1=0.0, r2=math.pi/2, r3=0.0,
        rotation_format="RPY",
        is_relative=False,
        cartesian_path=True,
        execute=False,
    )
    _log_result("move_pose_fallback_cartesian/boundary_jump", result)

    _setup(robot)

    # Opposite extreme: move straight down past the bottom cage limit
    result = robot.move_to_pose(
        x=0.0, y=0.0, z=-0.30,
        r1=0.0, r2=0.0, r3=0.0,
        rotation_format="RPY",
        reference_frame="WORLD",
        is_relative=True,
        cartesian_path=True,
        execute=False,
    )
    _log_result("move_pose_fallback_cartesian/below_cage", result)


# ─────────────────────────────────────────────
# 6. MOVE POSE CARTESIAN  (absolute)
# ─────────────────────────────────────────────

def test_move_pose_cartesian(robot: MotionRobotClient):
    """
    Straight Cartesian paths to absolute targets.
    Short steps so fraction stays above 0.95.
    """
    win_logger.info("─── test_move_pose_cartesian ───")
    _setup(robot)

    targets = [
        (0.35,  0.0,   0.50, 0.0, 0.0, 0.0, "forward-center"),
        (0.35,  0.10,  0.50, 0.0, 0.0, 0.0, "slight-right"),
        (0.35, -0.10,  0.50, 0.0, 0.0, 0.0, "slight-left"),
        (0.35,  0.0,   0.65, 0.0, 0.0, 0.0, "higher"),
        (0.35,  0.0,   0.35, 0.0, 0.0, 0.0, "lower"),
    ]

    for x, y, z, r1, r2, r3, label in targets:
        win_logger.info(f"  → cartesian: {label}")
        result = robot.move_to_pose(
            x=x, y=y, z=z,
            r1=r1, r2=r2, r3=r3,
            rotation_format="RPY",
            is_relative=False,
            cartesian_path=True,
            execute=True,
        )
        _log_result(f"move_pose_cartesian/{label}", result)
        time.sleep(0.3)


# ─────────────────────────────────────────────
# 7. MOVE POSE CARTESIAN RELATIVE
# ─────────────────────────────────────────────

def test_move_pose_cartesian_relative(robot: MotionRobotClient):
    """
    Small Cartesian increments in WORLD and TOOL frames.
    Good for verifying the applyVelocityScaling + TOTG pipeline.
    """
    win_logger.info("─── test_move_pose_cartesian_relative ───")
    _setup(robot)

    # Draw a small square in the XY plane (WORLD frame, Cartesian)
    square = [
        ( 0.06,  0.0,  0.0, 0.0, 0.0, 0.0, "sq→right"),
        ( 0.0,   0.06, 0.0, 0.0, 0.0, 0.0, "sq→fwd"),
        (-0.06,  0.0,  0.0, 0.0, 0.0, 0.0, "sq→left"),
        ( 0.0,  -0.06, 0.0, 0.0, 0.0, 0.0, "sq→back"),
    ]
    for x, y, z, r1, r2, r3, label in square:
        result = robot.move_to_pose(
            x=x, y=y, z=z,
            r1=r1, r2=r2, r3=r3,
            rotation_format="RPY",
            reference_frame="WORLD",
            is_relative=True,
            cartesian_path=True,
            execute=True,
        )
        _log_result(f"move_pose_cartesian_relative/{label}", result)
        time.sleep(0.2)

    _setup(robot)

    # Diagonal zig-zag (TOOL frame)
    zigzag = [
        (0.05,  0.05, 0.0, 0.0, 0.0, 0.0, "diag++"),
        (0.05, -0.05, 0.0, 0.0, 0.0, 0.0, "diag+-"),
        (0.05,  0.05, 0.0, 0.0, 0.0, 0.0, "diag++"),
        (0.05, -0.05, 0.0, 0.0, 0.0, 0.0, "diag+-"),
    ]
    for x, y, z, r1, r2, r3, label in zigzag:
        result = robot.move_to_pose(
            x=x, y=y, z=z,
            r1=r1, r2=r2, r3=r3,
            rotation_format="RPY",
            reference_frame="TOOL",
            is_relative=True,
            cartesian_path=True,
            execute=True,
        )
        _log_result(f"move_pose_cartesian_relative/{label}", result)
        time.sleep(0.2)


# ─────────────────────────────────────────────
# 8. MOVE WAYPOINTS CARTESIAN
# ─────────────────────────────────────────────

def test_move_waypoints_cartesian(robot: MotionRobotClient):
    """
    Multi-waypoint Cartesian paths.
    Tests the computeCartesianPathRobust + applyVelocityScaling pipeline
    across several waypoints in a single call.
    """
    win_logger.info("─── test_move_waypoints_cartesian ───")
    _setup(robot)

    # --- Test A: horizontal rectangle ---
    #  Each waypoint is expressed as an absolute pose (Quaternion identity).
    rect = [
        {"position": {"x": 0.40, "y":  0.10, "z": 0.50},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        {"position": {"x": 0.40, "y": -0.10, "z": 0.50},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        {"position": {"x": 0.30, "y": -0.10, "z": 0.50},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        {"position": {"x": 0.30, "y":  0.10, "z": 0.50},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
    ]
    result = robot.move_waypoints(
        waypoints=rect,
        is_relative_list=[False] * len(rect),
        reference_frame_list=["WORLD"] * len(rect),
        cartesian_path=True,
        execute=True,
    )
    _log_result("move_waypoints_cartesian/rectangle", result)

    _setup(robot)

    # --- Test B: staircase (Z steps) ---
    stairs = [
        {"position": {"x": 0.35, "y": 0.0, "z": 0.40},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        {"position": {"x": 0.40, "y": 0.0, "z": 0.50},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        {"position": {"x": 0.45, "y": 0.0, "z": 0.60},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        {"position": {"x": 0.40, "y": 0.0, "z": 0.50},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        {"position": {"x": 0.35, "y": 0.0, "z": 0.40},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
    ]
    result = robot.move_waypoints(
        waypoints=stairs,
        is_relative_list=[False] * len(stairs),
        reference_frame_list=["WORLD"] * len(stairs),
        cartesian_path=True,
        execute=True,
    )
    _log_result("move_waypoints_cartesian/staircase", result)

    _setup(robot)

    # --- Test C: mixed absolute + relative waypoints ---
    mixed = [
        {"position": {"x": 0.40, "y": 0.0, "z": 0.50},   # abs
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        {"position": {"x": 0.05, "y": 0.05, "z": 0.0},   # rel WORLD
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        {"position": {"x": 0.0,  "y": -0.05, "z": 0.05}, # rel WORLD
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
    ]
    result = robot.move_waypoints(
        waypoints=mixed,
        is_relative_list=[False, True, True],
        reference_frame_list=["WORLD", "WORLD", "WORLD"],
        cartesian_path=True,
        execute=True,
    )
    _log_result("move_waypoints_cartesian/mixed_abs_rel", result)


# ─────────────────────────────────────────────
# 9. MOVE WAYPOINTS  (joint-space, multi-segment)
# ─────────────────────────────────────────────

def test_move_waypoints(robot: MotionRobotClient):
    """
    Multi-waypoint joint-space planning.
    Exercises the segment-stitching + TOTG re-timing code path.
    """
    win_logger.info("─── test_move_waypoints ───")
    _setup(robot)

    # --- Test A: simple triangle of poses ---
    triangle = [
        {"position": {"x": 0.40, "y":  0.10, "z": 0.45},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        {"position": {"x": 0.40, "y": -0.10, "z": 0.55},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        {"position": {"x": 0.30, "y":  0.0,  "z": 0.50},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
    ]
    result = robot.move_waypoints(
        waypoints=triangle,
        is_relative_list=[False, False, False],
        reference_frame_list=["WORLD", "WORLD", "WORLD"],
        cartesian_path=False,
        execute=True,
    )
    _log_result("move_waypoints/triangle", result)

    _setup(robot)

    # --- Test B: longer sequence with orientation changes ---
    sequence = [
        {"position": {"x": 0.38, "y":  0.12, "z": 0.45},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        {"position": {"x": 0.45, "y":  0.0,  "z": 0.60},
         "orientation": {"x": 0, "y": 0.131, "z": 0, "w": 0.991}},  # ~15° pitch
        {"position": {"x": 0.38, "y": -0.12, "z": 0.45},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        {"position": {"x": 0.30, "y":  0.0,  "z": 0.35},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
    ]
    result = robot.move_waypoints(
        waypoints=sequence,
        is_relative_list=[False] * len(sequence),
        reference_frame_list=["WORLD"] * len(sequence),
        cartesian_path=False,
        execute=True,
    )
    _log_result("move_waypoints/long_sequence", result)

    _setup(robot)

    # --- Test C: relative chain (each point relative to the previous) ---
    rel_chain = [
        {"position": {"x":  0.06, "y":  0.0,  "z":  0.0},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        {"position": {"x":  0.0,  "y":  0.08, "z":  0.0},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        {"position": {"x": -0.06, "y":  0.0,  "z":  0.05},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        {"position": {"x":  0.0,  "y": -0.08, "z": -0.05},
         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
    ]
    result = robot.move_waypoints(
        waypoints=rel_chain,
        is_relative_list=[True] * len(rel_chain),
        reference_frame_list=["WORLD"] * len(rel_chain),
        cartesian_path=False,
        execute=True,
    )
    _log_result("move_waypoints/relative_chain", result)


# ─────────────────────────────────────────────
# 10. MOVE POSE VIA JOINT  (explicit IK + joint planning)
#     Forces the solveIKAndPlanJoints path (multi-seed IK search).
# ─────────────────────────────────────────────

def test_move_pose_via_joint(robot: MotionRobotClient):
    """
    Uses the /move_to_pose_via_joint endpoint which bypasses the standard
    pose-target planner and goes straight to multi-seed IK + joint planning.
    Good for checking the IK seed selection and cost function.
    """
    win_logger.info("─── test_move_pose_via_joint ───")
    _setup(robot)

    targets = [
        (0.35,  0.10,  0.45,  0.0,        0.0,        0.0,   "nominal-right"),
        (0.35, -0.10,  0.45,  0.0,        0.0,        0.0,   "nominal-left"),
        (0.45,  0.0,   0.60,  0.0,        math.pi/8,  0.0,   "high-pitched"),
        (0.30,  0.0,   0.30,  math.pi/6,  0.0,        0.0,   "low-rolled"),
        (0.50,  0.10,  0.50, -math.pi/8,  math.pi/6,  0.4,   "complex-rpy"),
        # Force IK to find a non-obvious solution
        (0.40,  0.0,   0.20,  0.0,        math.pi/2,  0.0,   "near-singular-down"),
    ]

    for x, y, z, r1, r2, r3, label in targets:
        win_logger.info(f"  → via_joint: {label}")
        result = robot.move_to_pose_via_joint(
            x=x, y=y, z=z,
            r1=r1, r2=r2, r3=r3,
            rotation_format="RPY",
            is_relative=False,
            execute=True,
        )
        _log_result(f"move_pose_via_joint/{label}", result)
        time.sleep(0.3)


# ─────────────────────────────────────────────
# RUNNER
# ─────────────────────────────────────────────

ALL_TESTS = [
    test_ik_non_reachable,
    test_move_pose,
    test_move_pose_relative,
    test_move_pose_fallback,
    test_move_pose_fallback_cartesian,
    test_move_pose_cartesian,
    test_move_pose_cartesian_relative,
    test_move_waypoints_cartesian,
    test_move_waypoints,
    test_move_pose_via_joint,
]


def run_all_tests(robot: MotionRobotClient):
    passed = 0
    failed = 0
    for fn in ALL_TESTS:
        win_logger.info(f"\n{'='*50}")
        try:
            fn(robot)
            passed += 1
        except Exception as e:
            win_logger.error(f"[EXCEPTION] {fn.__name__}: {e}")
            failed += 1
        finally:
            # Always go home and re-arm the cage between top-level tests
            try:
                robot.set_virtual_cage(**CAGE)
                robot.move_joints(HOME_JOINTS, is_relative=False)
            except Exception:
                pass

    win_logger.info(f"\n{'='*50}")
    win_logger.info(f"Results: {passed} tests ran, {failed} raised exceptions")


# ─────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────

if __name__ == "__main__":
    linux_log_path = r"\\wsl.localhost\Ubuntu-22.04\home\antonin\workspace\robot_system.log"
    log_thread = threading.Thread(
        target=tail_linux_logs,
        args=(linux_log_path,),
        daemon=True,
    )
    log_thread.start()

    robot = MotionRobotClient("http://localhost:8000")
    win_logger.info(f"Health: {robot.health()}")

    robot.init_robot(
        model="vs060",
        planning_group="arm",
        velocity_scale=0.15,
        accel_scale=0.15,
        planning_time=10,
        planning_attempts=20,
        allow_replanning=True,
        planner_id="RRTConnect",
    )

    # ── Run a single test ──────────────────────────────
    # test_move_pose(robot)

    # ── Run the full suite ─────────────────────────────
    run_all_tests(robot)

    robot.set_virtual_cage(enable=False)