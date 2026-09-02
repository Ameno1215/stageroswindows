"""
Smoke test for the UFACTORY Lite 6 — the first thing to run after starting the
stack with:

    python .\launch_controller.py --model lite6 --show-terminals

It deliberately does NOT use the pump or a tool: the Lite 6 is integrated bare
for now, so the full test.py scenario (card handling) does not apply to it yet.
What this checks is the whole chain end to end — HTTP bridge, motion_server,
MoveIt, the lite6 planning group, ros2_control — with small isolated moves:

    init -> environment -> motors -> home -> joint moves -> Cartesian move -> home

Run it in simulation first. Only pass --real-robot once every stage passes, and
keep the E-Stop within reach.
"""

import argparse
import time
from pathlib import Path

from motion_http_client import MotionRobotClient

parser = argparse.ArgumentParser(description="Lite 6 smoke test")
parser.add_argument("--real-robot", action="store_true",
                    help="Run against the real Lite 6 (default: simulation)")
parser.add_argument("--url", default="http://localhost:8000",
                    help="URL of the WSL HTTP bridge")
args = parser.parse_args()

SIM = not args.real_robot
MODEL = "lite6"

# Conservative: the Lite 6 is a 440 mm / 600 g arm, and this is a first bring-up.
SPEED = 0.15
ACCEL = 0.1

ENV_FILE = Path(__file__).resolve().parent / "env" / "lite6.json"


def step(title):
    print(f"\n=== {title} " + "=" * max(0, 60 - len(title)))


def main():
    robot = MotionRobotClient(args.url, SIM)

    step("Health")
    print(robot.health())

    step("Init")
    print(robot.init_robot(
        model=MODEL,
        velocity_scale=SPEED,
        accel_scale=ACCEL,
        planning_time=10,
        planning_attempts=20,
        allow_replanning=True,
        planner_id="RRTConnect",
    ))
    print(robot.set_scaling(velocity_scale=SPEED, accel_scale=ACCEL))

    step("Environment")
    print(robot.clear_environment())
    print(robot.load_environnement(str(ENV_FILE)))
    robot.clear_trace()
    time.sleep(1)

    step("Motors on")
    print(robot.set_servo_on(True))

    step("Home")
    print(robot.move_to_home())
    print("joints:", robot.get_joint_state())
    print("pose  :", robot.get_current_pose())

    step("Joint moves (relative, one joint at a time)")
    # Small deltas, returned immediately, so a wrong joint direction shows up
    # without the arm travelling far.
    for index, name in enumerate(["joint1", "joint2", "joint3",
                                  "joint4", "joint5", "joint6"]):
        delta = [0.0] * 6
        delta[index] = 0.2
        print(f"  {name} +0.2 rad:", robot.move_joints(delta, is_relative=True))
        delta[index] = -0.2
        print(f"  {name} -0.2 rad:", robot.move_joints(delta, is_relative=True))

    step("Cartesian move (relative, straight line)")
    print("  +5 cm Z:", robot.move_to_pose(0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
                                           rotation_format="RPY",
                                           is_relative=True,
                                           cartesian_path=True))
    print("  -5 cm Z:", robot.move_to_pose(0.0, 0.0, -0.05, 0.0, 0.0, 0.0,
                                           rotation_format="RPY",
                                           is_relative=True,
                                           cartesian_path=True))

    step("Back home, motors off")
    print(robot.move_to_home())
    print(robot.set_servo_on(False))

    print("\nSmoke test finished.")


if __name__ == "__main__":
    main()
