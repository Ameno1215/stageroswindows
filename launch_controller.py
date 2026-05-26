import subprocess
import time
import signal
import sys
import argparse

# --- CLI args ----------------------------------------------------------------

parser = argparse.ArgumentParser(description="Launch DENSO VS060 ROS 2 stack")
parser.add_argument("--show-terminals", action="store_true",
                    help="Hide WSL terminals (run in background)")
parser.add_argument("--real-robot", action="store_true",
                    help="Connect to the real robot (default: simulation)")
parser.add_argument("--model", choices=["vs060", "vp5243", "tx2_60l", "tx40"], default="vs060",
                    help="Robot model to use (default: vs060)")
parser.add_argument("--tool", default=None,
                      help="Tool name (ex: effecteur_v3, none)")
parser.add_argument("--ip", default="169.254.75.249",
                      help="Robot IP (ex: 169.254.139.249)")





args = parser.parse_args()


SHOW_TERMINALS = args.show_terminals
SOLVER = "kdl"
SIM = "false" if args.real_robot else "true"
MODEL = args.model
IP_ROBOT = args.ip
IS_STAUBLI = MODEL in {"tx2_60l", "tx40"}
DEFAULT_TOOL = "effecteur_v3"
TOOL = args.tool or DEFAULT_TOOL

if IS_STAUBLI and args.real_robot:
    parser.error("Staubli robots are simulation-only in this launcher")



# --- Commands ----------------------------------------------------------------

SETUP = (
    "cd ~/workspace/denso_ros2_ws && "
    "source /opt/ros/humble/setup.bash && "
    "source install/setup.bash && "
    "export OMPL_CONSOLE_LOG_LEVEL=DEV2 && "
    "export LIBGL_ALWAYS_SOFTWARE=0 && "
    "export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA && "
    "export GAZEBO_MODEL_DATABASE_URI= && "
    "unset IGN_FUEL_CACHE_PATH && "
    "export GZ_IP=127.0.0.1"
)

if IS_STAUBLI:
    TERMINAL_1 = (
        f"{SETUP} && "
        f"ros2 launch staubli_{MODEL}_moveit_config "
        f"staubli_{MODEL}_planning_execution_sim.launch.py tool:={TOOL}"
    )
    TERMINAL_2 = (
        f"{SETUP} && "
        "ros2 launch motion_control motion_server.launch.py "
        f"model:=staubli_{MODEL} sim:=true tool:={TOOL} ik_solver:={SOLVER}"
    )
else:
    TERMINAL_1 = (
        f"{SETUP} && "
        "ros2 launch denso_robot_bringup denso_robot_bringup.launch.py "
        f"model:={MODEL} sim:=true tool:={TOOL} ik_solver:={SOLVER}"
    )
    if SIM == "false":
        TERMINAL_1 = (
            f"{SETUP} && "
            "ros2 launch denso_robot_bringup denso_robot_bringup.launch.py "
            f"model:={MODEL} sim:=false ip_address:={IP_ROBOT} "
            f"send_format:=256 recv_format:=258 tool:={TOOL} ik_solver:={SOLVER}"
        )

    TERMINAL_2 = (
        f"{SETUP} && "
        "ros2 launch motion_control motion_server.launch.py "
        f"model:={MODEL} sim:={SIM} tool:={TOOL} ik_solver:={SOLVER}"
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
    "uvicorn wsl_ros_bridge:app --host 127.0.0.1 --port 8000"
    # "uvicorn wsl_ros_bridge:app --host 0.0.0.0 --port 8000"
)

TAB_TITLES = ["DENSO_Bringup", "DENSO_MotionServer", "Pump Control", "DENSO_Bridge"]

# --- Launched processes ------------------------------------------------------

launched_processes = []

# --- Launch ------------------------------------------------------------------

def launch_wsl_tab(title, bash_cmd):
    """Opens a visible terminal or a hidden background process depending on SHOW_TERMINALS."""

    if SHOW_TERMINALS:
        wrapped_cmd = (
            f"({bash_cmd}); "
            f"echo; "
            f"echo '--- Process ended (press Enter or Ctrl+D to close) ---'; "
            f"exec bash"
        )
        wrapped_cmd_escaped = wrapped_cmd.replace("; ", " \\; ")
        proc = subprocess.Popen(
            ["wt.exe", "-w", "denso_stack", "new-tab",
             "--title", title, "--", "wsl.exe", "bash", "-c", wrapped_cmd_escaped]
        )
    else:
        proc = subprocess.Popen(
            ["wsl.exe", "bash", "-c", bash_cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            creationflags=subprocess.CREATE_NO_WINDOW,
        )

    launched_processes.append(proc)
    return proc

# --- WSL process cleanup -----------------------------------------------------

def kill_wsl_processes():
    """Kills ROS 2, Gazebo and Uvicorn processes on the WSL side."""
    print("\nStopping WSL processes...")

    all_targets = [
        "denso_robot", "move_group", "robot_state_publisher", "rviz2",
        "motion_server", "motion_control",
        "pump_controller", "command_pump_denso",
        "gzserver", "gzclient", "gazebo", "gzweb",
        "spawn_entity", "controller_manager", "ros2_control_node",
        "uvicorn",
    ]

    # --- Step 1: SIGINT on ROS2 nodes (allows on_deactivate to run)
    print("   Sending SIGINT to ROS2 nodes...")
    for target in all_targets:
        subprocess.run(
            ["wsl.exe", "bash", "-c", f"pkill -SIGINT -f {target} 2>/dev/null || true"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )

    # --- Step 2: Long wait (RC8 controller needs time to close the b-CAP session)
    wait_time = 15 if SIM == "false" else 3
    print(f"   Waiting {wait_time}s for graceful b-CAP disconnect...")
    time.sleep(wait_time)

    # --- Step 3: SIGTERM for any remaining processes
    print("   Sending SIGTERM to remaining processes...")
    for target in all_targets:
        subprocess.run(
            ["wsl.exe", "bash", "-c", f"pkill -SIGTERM -f {target} 2>/dev/null || true"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    time.sleep(2)

    # --- Step 4: SIGKILL as last resort only
    print("   Force-killing remaining processes...")
    for target in all_targets:
        subprocess.run(
            ["wsl.exe", "bash", "-c", f"pkill -9 -f {target} 2>/dev/null || true"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )

    # Step 4.5: clean Gazebo and DDS state
    print("   Cleaning Gazebo lock files and DDS shared memory...")
    subprocess.run(
        ["wsl.exe", "bash", "-c",
         "rm -rf /tmp/gazebo-* /tmp/.gazebo* 2>/dev/null; "
         "rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null; "
         "rm -f /tmp/gz_* 2>/dev/null; "
         "true"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    # Step 5: stop ROS daemon
    subprocess.run(
        ["wsl.exe", "bash", "-c", "ros2 daemon stop 2>/dev/null || true"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    print("   Verifying cleanup...")
    result = subprocess.run(
        ["wsl.exe", "bash", "-c",
        "pgrep -af 'gazebo|gzserver|gzclient|move_group|controller_manager|ros2_control|spawn_entity|motion_server|pump_controller|uvicorn|denso_robot' "
        "| grep -v pgrep | grep -v 'bash -c' || echo CLEAN"],
        capture_output=True, text=True,
    )
    leftovers = result.stdout.strip()
    if leftovers and leftovers != "CLEAN":
        print(f"     Leftover processes detected:\n{leftovers}")
        print("   Force-killing leftovers...")
        subprocess.run(
            ["wsl.exe", "bash", "-c",
            "pgrep -f 'gazebo|gzserver|gzclient|move_group|controller_manager|ros2_control|spawn_entity|motion_server|pump_controller|uvicorn|denso_robot' "
            "| xargs -r kill -9 2>/dev/null || true"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    else:
        print("     All processes terminated")
        
    port_check = subprocess.run(
        ["wsl.exe", "bash", "-c",
         "ss -tln | grep ':11345' || echo PORT_FREE"],
        capture_output=True, text=True,
    )
    if "PORT_FREE" not in port_check.stdout:
        print(f"     Port 11345 still in use:\n{port_check.stdout.strip()}")
        print("   Waiting 5s for socket to release...")
        time.sleep(5)
    else:
        print("     Port 11345 free")

    print("   WSL processes stopped.")

# --- Global cleanup ----------------------------------------------------------

def cleanup():
    kill_wsl_processes()
    print("\nClean shutdown complete.")
    sys.exit(0)

# --- Ctrl+C signal handler ---------------------------------------------------

def handle_sigint(sig, frame):
    print("\n\nCtrl+C detected, shutting down...")
    cleanup()

signal.signal(signal.SIGINT, handle_sigint)

# --- Main --------------------------------------------------------------------

def main():
    mode = "visible" if SHOW_TERMINALS else "hidden (background)"
    print(f"Starting {MODEL} stack (mode: {mode})...\n")

    # print("Cleaning leftover state from previous runs...")
    # kill_wsl_processes()
    # time.sleep(2)


    total = 3 if IS_STAUBLI else 4
    step = 1

    print(f"[{step}/{total}] Starting Gazebo & RViz...")
    launch_wsl_tab(TAB_TITLES[0], TERMINAL_1)
    step += 1

    print("      Waiting 5s for Gazebo to start...")
    time.sleep(5)

    print(f"[{step}/{total}] Starting Motion Server...")
    launch_wsl_tab(TAB_TITLES[1], TERMINAL_2)
    step += 1

    print("      Waiting 2s...")
    time.sleep(2)

    if not IS_STAUBLI:
        print(f"[{step}/{total}] Starting Pump Control...")
        launch_wsl_tab(TAB_TITLES[2], TERMINAL_3)
        step += 1

    print(f"[{step}/{total}] Starting HTTP Bridge...")
    launch_wsl_tab(TAB_TITLES[3], TERMINAL_4)

    print(f"\nAll {total} WSL processes launched (mode: {mode}).")
    print("Press Ctrl+C to stop everything.\n")

    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()