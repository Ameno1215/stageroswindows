import argparse
import base64
import glob
import os
import signal
import subprocess
import sys
import time
from datetime import datetime


parser = argparse.ArgumentParser(description="Launch the DENSO ROS 2 stack with Gazebo or Isaac Sim")
parser.add_argument("--show-terminals", action="store_true", help="Open WSL commands in Windows Terminal tabs")
parser.add_argument("--solver", choices=["pick_ik", "kdl"], default="pick_ik", help="IK solver to use")
parser.add_argument("--real-robot", action="store_true", help="Connect to the real robot instead of simulation")
parser.add_argument("--model", choices=["vs060", "vp5243"], default="vs060", help="Robot model to use")
parser.add_argument("--sim-backend", choices=["gazebo", "isaac"], default="gazebo", help="Simulation backend")
parser.add_argument("--launch-isaac", action="store_true", help="Launch Isaac Sim on Windows before the ROS stack")
parser.add_argument("--isaac-path", default="", help="Path to isaac-sim.bat or Isaac Sim executable")
parser.add_argument("--ros-domain-id", default="0", help="ROS_DOMAIN_ID shared between WSL and Isaac Sim")
args = parser.parse_args()

SHOW_TERMINALS = args.show_terminals
SOLVER = args.solver
SIM = "false" if args.real_robot else "true"
SIM_BACKEND = "gazebo" if args.real_robot else args.sim_backend
MODEL = args.model
ROS_DOMAIN_ID = args.ros_domain_id

launched_processes = []
isaac_process = None
LOG_STAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
RUNNER_SCRIPT = "/home/antonin/workspace/run_logged_command.sh"

if SIM_BACKEND == "isaac" and MODEL != "vs060":
    print("The Isaac integration currently targets the VS060 only.")
    sys.exit(2)


def wsl_setup() -> str:
    return (
        "cd ~/workspace/denso_ros2_ws && "
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        "export OMPL_CONSOLE_LOG_LEVEL=DEBUG && "
        "export LIBGL_ALWAYS_SOFTWARE=0 && "
        "export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA && "
        f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID} && "
        "[ -f ~/.ros/fastdds.xml ] && export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml || true"
    )


SETUP = wsl_setup()

if SIM == "true":
    TERMINAL_1 = (
        f"{SETUP} && "
        "ros2 launch denso_robot_bringup denso_robot_bringup.launch.py "
        f"model:={MODEL} sim:=true sim_backend:={SIM_BACKEND} "
        f"use_sim_time:={'true' if SIM == 'true' else 'false'}"
        f"description_file:={'denso_robot_isaac.urdf.xacro' if SIM_BACKEND == 'isaac' else 'denso_robot.urdf.xacro'} "
        f"tool:=effecteur_v2 ik_solver:={SOLVER}"
    )
else:
    TERMINAL_1 = (
        f"{SETUP} && "
        "ros2 launch denso_robot_bringup denso_robot_bringup.launch.py "
        f"model:={MODEL} sim:=false sim_backend:=gazebo use_sim_time:=false "
        "ip_address:=169.254.139.249 send_format:=256 recv_format:=258 "
        f"tool:=effecteur_v2 ik_solver:={SOLVER}"
    )

TERMINAL_2 = (
    f"{SETUP} && "
    "ros2 launch motion_control motion_server.launch.py "
    f"model:={MODEL} sim:={SIM} sim_backend:={SIM_BACKEND} "
    f"use_sim_time:={'true' if SIM == 'true' else 'false'}"
    f"description_file:={'denso_robot_isaac.urdf.xacro' if SIM_BACKEND == 'isaac' else 'denso_robot.urdf.xacro'} "
    f"tool:=effecteur_v2 ik_solver:={SOLVER}"
)

TERMINAL_3 = (
    f"{SETUP} && "
    "ros2 launch command_pump_denso pump_controller.launch.py "
    f"model:={MODEL} pump_pin:=25 valve_pin:=26 vacuum_sensor_pin:=8"
)

TERMINAL_4 = (
    "cd ~/workspace && "
    "source venv/bin/activate && "
    "source /opt/ros/humble/setup.bash && "
    "source ~/workspace/denso_ros2_ws/install/setup.bash && "
    f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID} && "
    "[ -f ~/.ros/fastdds.xml ] && export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml || true && "
    "uvicorn wsl_ros_bridge:app --host 127.0.0.1 --port 8000"
)


def launch_wsl_tab(title: str, bash_cmd: str):
    encoded_cmd = base64.b64encode(bash_cmd.encode("utf-8")).decode("ascii")
    keep_open = "1" if SHOW_TERMINALS else "0"
    runner_args = ["wsl.exe", "bash", RUNNER_SCRIPT, LOG_STAMP, title, keep_open, encoded_cmd]

    if SHOW_TERMINALS:
        proc = subprocess.Popen(
            ["wt.exe", "new-tab", "--title", title, "--"] + runner_args
        )
    else:
        proc = subprocess.Popen(
            runner_args,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            creationflags=subprocess.CREATE_NO_WINDOW,
        )

    launched_processes.append(proc)
    return proc


def find_isaac_launcher(explicit_path: str) -> str:
    if explicit_path:
        return explicit_path

    local_ov = os.environ.get("LOCALAPPDATA", "")
    candidates = sorted(
        glob.glob(os.path.join(local_ov, "ov", "pkg", "isaac-sim-*", "isaac-sim.bat")),
        reverse=True,
    )
    return candidates[0] if candidates else ""


def launch_isaac_if_requested():
    global isaac_process

    if SIM != "true" or SIM_BACKEND != "isaac" or not args.launch_isaac:
        return

    isaac_launcher = find_isaac_launcher(args.isaac_path)
    if not isaac_launcher:
        print("Isaac Sim launcher not found. Use --isaac-path to point to isaac-sim.bat.")
        return

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID
    if os.path.exists(r"C:\.ros\fastdds.xml"):
        env["FASTRTPS_DEFAULT_PROFILES_FILE"] = r"C:\.ros\fastdds.xml"

    print(f"Launching Isaac Sim from {isaac_launcher}")
    isaac_process = subprocess.Popen([isaac_launcher], env=env)


def kill_wsl_processes():
    print("\nStopping WSL processes...")

    ros_targets = [
        "denso_robot",
        "move_group",
        "robot_state_publisher",
        "rviz2",
        "follow_joint_trajectory_bridge",
        "denso_joint_trajectory_controller",
    ]
    all_targets = ros_targets + ["gzserver", "gzclient", "gazebo", "uvicorn", "pump_controller"]

    print("   Sending SIGINT to WSL nodes...")
    for target in all_targets:
        subprocess.run(
            ["wsl.exe", "bash", "-c", f"pkill -SIGINT -f {target} 2>/dev/null || true"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    wait_time = 15 if SIM == "false" else 3
    print(f"   Waiting {wait_time}s for graceful shutdown...")
    time.sleep(wait_time)

    print("   Sending SIGTERM to remaining WSL processes...")
    for target in all_targets:
        subprocess.run(
            ["wsl.exe", "bash", "-c", f"pkill -SIGTERM -f {target} 2>/dev/null || true"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    time.sleep(2)

    print("   Force-killing remaining WSL processes...")
    for target in all_targets:
        subprocess.run(
            ["wsl.exe", "bash", "-c", f"pkill -9 -f {target} 2>/dev/null || true"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    subprocess.run(
        ["wsl.exe", "bash", "-c", "ros2 daemon stop 2>/dev/null || true"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    if not SHOW_TERMINALS:
        for proc in launched_processes:
            try:
                proc.terminate()
            except Exception:
                pass

    print("   WSL processes stopped.")


def cleanup():
    global isaac_process

    kill_wsl_processes()

    if isaac_process and isaac_process.poll() is None:
        print("Stopping Isaac Sim...")
        isaac_process.terminate()

    print("\nClean shutdown complete.")
    sys.exit(0)


def handle_sigint(_sig, _frame):
    print("\n\nCtrl+C detected, shutting down...")
    cleanup()


signal.signal(signal.SIGINT, handle_sigint)


def main():
    mode = "visible" if SHOW_TERMINALS else "hidden (background)"
    print(f"Starting DENSO stack with backend={SIM_BACKEND} (mode: {mode})...\n")
    print(f"WSL logs will be written under ~/workspace/launch_logs/{LOG_STAMP}_*.log\n")

    launch_isaac_if_requested()

    steps = [
        ("Starting Bringup...", "DENSO_Bringup", TERMINAL_1, 5 if SIM_BACKEND == "gazebo" else 3),
        ("Starting Motion Server...", "DENSO_MotionServer", TERMINAL_2, 2),
    ]

    if SIM_BACKEND != "isaac" or SIM == "false":
        steps.append(("Starting Pump Control...", "Pump Control", TERMINAL_3, 0))

    steps.append(("Starting HTTP Bridge...", "DENSO_Bridge", TERMINAL_4, 0))

    for index, (label, title, command, wait_s) in enumerate(steps, start=1):
        print(f"[{index}/{len(steps)}] {label}")
        launch_wsl_tab(title, command)
        if wait_s:
            print(f"      Waiting {wait_s}s...")
            time.sleep(wait_s)

    print(f"\nAll WSL processes launched (mode: {mode}).")
    if SIM_BACKEND == "isaac" and SIM == "true":
        print("Import the Isaac URDF once, then run isaac_sim/setup_vs060_ros2_bridge.py in Isaac Sim.")
    print("Press Ctrl+C to stop everything.\n")

    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
