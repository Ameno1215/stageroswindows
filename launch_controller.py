import subprocess
import time
import signal
import sys
import argparse
import shlex

from utils.ros_backend import IS_WINDOWS, ROS_WS, BRIDGE_DIR, BRIDGE_CMD, \
    run_ros, ensure_backend, stop_backend, open_ros_tab

# --- CLI args ----------------------------------------------------------------

parser = argparse.ArgumentParser(description="Launch DENSO VS060 ROS 2 stack")
parser.add_argument("--show-terminals", action="store_true",
                    help="Show one terminal tab per process (default: run in background)")
parser.add_argument("--real-robot", action="store_true",
                    help="Connect to the real robot (default: simulation)")
parser.add_argument("--accuracy", action="store_true",
                    help="log movement accuracy in debug mode.")
parser.add_argument("--model", choices=["vs060", "vp5243", "tx40", "lite6"], default="vs060",
                    help="Robot model to use (default: vs060)")
parser.add_argument("--tool", default=None,
                      help="Tool name (ex: effecteur_v3, effecteur_short, none)")
parser.add_argument("--ip", default="169.254.75.249",
                      help="Robot IP (ex: 169.254.139.249)")





args = parser.parse_args()


SHOW_TERMINALS = args.show_terminals
SOLVER = "kdl"
# SOLVER = "pick_ik"
SIM = "false" if args.real_robot else "true"
MODEL = args.model
IP_ROBOT = args.ip
ACCURACY = args.accuracy
IS_STAUBLI = MODEL in {"tx40"}
IS_UFACTORY = MODEL in {"lite6"}
DEFAULT_TOOL = "effecteur_v3"
TOOL = args.tool or DEFAULT_TOOL





# --- Commands ----------------------------------------------------------------
TERMINAL_1 = None
TERMINAL_2 = None
TERMINAL_3 = None
TERMINAL_4 = None
TERMINAL_5 = None

# GPU selection through D3D12 only makes sense under WSL
GPU_ENV = (
    "export LIBGL_ALWAYS_SOFTWARE=0 && "
    "export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA && "
) if IS_WINDOWS else ""

SETUP = (
    f"cd {ROS_WS} && "
    "source /opt/ros/humble/setup.bash && "
    "source install/setup.bash && "
    "export OMPL_CONSOLE_LOG_LEVEL=DEV2 && "
    f"{GPU_ENV}"
    "export GAZEBO_IP=127.0.0.1 &&"
    "export GAZEBO_HOSTNAME=127.0.0.1 &&"
    "export GAZEBO_MASTER_URI=http://127.0.0.1:11345 &&"
    "unset IGN_FUEL_CACHE_PATH && "
    "export GZ_IP=127.0.0.1"
)

if IS_UFACTORY:
    # TERMINAL_1: robot_state_publisher + ros2_control + move_group + RViz.
    # sim:=true uses the vendor fake hardware, sim:=false talks to the real arm.
    # The Pilz capabilities are what motion_control needs for LIN / sequence moves.
    TERMINAL_1 = (
        f"{SETUP} && "
        "ros2 launch ufactory_lite6_moveit_config ufactory_lite6_planning_execution.launch.py "
        f"sim:={SIM} tool:={TOOL} ip_address:={IP_ROBOT} "
        f"capabilities:='pilz_industrial_motion_planner/MoveGroupSequenceAction "
        f"pilz_industrial_motion_planner/MoveGroupSequenceService'"
    )

    TERMINAL_2 = (
        f"{SETUP} && "
        "ros2 launch motion_control motion_server.launch.py "
        f"model:=ufactory_{MODEL} sim:={SIM} tool:={TOOL} ik_solver:={SOLVER} accuracy:={ACCURACY}"
    )

    # No pump package for the Lite 6 yet: TERMINAL_3 stays None and the tab is
    # skipped in main(). The tool, on the other hand, is attached like on the
    # other robots — pass --tool none to run the arm bare.
    TERMINAL_3 = None

elif IS_STAUBLI:
    if SIM == "true":
        TERMINAL_1 = (
            f"{SETUP} && "
            f"ros2 launch staubli_{MODEL}_moveit_config "
            f"staubli_{MODEL}_planning_execution_sim.launch.py tool:={TOOL} "
            f"capabilities:='pilz_industrial_motion_planner/MoveGroupSequenceAction "
            f"pilz_industrial_motion_planner/MoveGroupSequenceService'"
        )
    else:
        TERMINAL_1 = (
            f"{SETUP} && "
            f"ros2 launch staubli_{MODEL}_moveit_config staubli_{MODEL}_planning_execution_real.launch.py tool:={TOOL} --debug"
        )
        TERMINAL_5 = (
            f"{SETUP} && "
            f"ros2 launch staubli_val3_driver robot_interface_streaming.launch.py robot_ip:={IP_ROBOT}"
        )
            
    TERMINAL_2 = (
        f"{SETUP} && "
        "ros2 launch motion_control motion_server.launch.py "
        f"model:=staubli_{MODEL} sim:={SIM} tool:={TOOL} ik_solver:={SOLVER} accuracy:={ACCURACY}"
    )

    TERMINAL_3 = (
        f"{SETUP} && "
        f"ros2 launch command_pump_staubli pump_controller.launch.py use_direct_io:=false "   
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
        f"model:={MODEL} sim:={SIM} tool:={TOOL} ik_solver:={SOLVER} accuracy:={ACCURACY}"
    )

    TERMINAL_3 = (
        f"{SETUP} && "
        "ros2 launch command_pump_denso pump_controller.launch.py "
        f"model:={MODEL} pump_pin:=25 valve_pin:=26 vacuum_sensor_pin:=8"
    )


TERMINAL_4 = (
    f"cd {BRIDGE_DIR} && "
    "source /opt/ros/humble/setup.bash && "
    f"source {ROS_WS}/install/setup.bash && "
    f"{BRIDGE_CMD}"
    # "uvicorn wsl_ros_bridge:app --host 0.0.0.0 --port 8000"
)

TAB_TITLES = ["Bringup", "MotionServer", "Pump Control", "WSL_Bridge", "Staubli_Connection"]

# --- Launched processes ------------------------------------------------------

launched_processes = []
started_container = False

# --- Launch ------------------------------------------------------------------

def launch_wsl_tab(title, bash_cmd):
    """Opens a visible terminal or a hidden background process depending on SHOW_TERMINALS."""
    proc = open_ros_tab(title, bash_cmd, SHOW_TERMINALS)
    launched_processes.append(proc)
    return proc

# --- WSL process cleanup -----------------------------------------------------

def kill_wsl_processes():
    """Kills ROS 2, Gazebo and Uvicorn processes on the WSL side."""
    print("\nStopping WSL processes...")

    launch_targets = [
        "ros2 launch denso_robot_bringup",
        "denso_robot_bringup.launch.py",
        "ros2 launch motion_control",
        "motion_server.launch.py",
        "ros2 launch command_pump_denso",
        "pump_controller.launch.py",
        "ros2 launch ufactory_lite6_moveit_config",
        "ufactory_lite6_planning_execution.launch.py",
        f"ros2 launch staubli_{MODEL}_moveit_config",
        f"staubli_{MODEL}_planning_execution",
        "ros2 launch staubli_val3_driver",
        "robot_interface_streaming.launch.py",
        "python wsl_ros_bridge.py",
    ]

    node_targets = [
        "denso_robot", "move_group", "robot_state_publisher", "rviz2",
        "motion_server", "motion_control",
        "pump_controller", "command_pump_denso",
        "staubli_val3_driver", "robot_interface_streaming", "robot_interface",
        "ufactory_lite6", "xarm",
        "gzserver", "gzclient", "gazebo", "gzweb",
        "spawn_entity", "controller_manager", "ros2_control_node",
        "wsl_ros_bridge",
    ]

    all_targets = launch_targets + node_targets

    def pkill(signal_name, targets):
        for target in targets:
            target = shlex.quote(target)
            run_ros(f"pkill -{signal_name} -f -- {target} 2>/dev/null || true")

    # --- Step 1: SIGINT on ros2 launch supervisors first
    # If a launch process survives, it can restart children that were just killed.
    print("   Sending SIGINT to ros2 launch supervisors...")
    pkill("SIGINT", launch_targets)
    time.sleep(1)

    # --- Step 2: SIGINT on ROS2 nodes (allows on_deactivate to run)
    print("   Sending SIGINT to ROS2 nodes...")
    pkill("SIGINT", node_targets)

    # --- Step 3: Long wait (RC8 controller needs time to close the b-CAP session)
    wait_time = 7 if SIM == "false" else 2
    print(f"   Waiting {wait_time}s for graceful controller disconnection...")
    time.sleep(wait_time)

    # --- Step 4: SIGTERM for any remaining launch supervisors and nodes
    print("   Sending SIGTERM to remaining processes...")
    pkill("SIGTERM", all_targets)
    time.sleep(1)

    # --- Step 5: SIGKILL as last resort only
    print("   Force-killing remaining processes...")
    pkill("9", all_targets)

    # Also kill orphaned ros2 launch processes that contain this workspace stack.
    launch_regex = (
        "ros2 launch (denso_robot_bringup|motion_control|command_pump_denso|"
        f"staubli_{MODEL}_moveit_config|staubli_val3_driver|ufactory_lite6_moveit_config)"
    )
    run_ros(f"pgrep -f {shlex.quote(launch_regex)} | xargs -r kill -9 2>/dev/null || true")

    # Terminate Windows-side wrappers after the WSL processes are gone.
    for proc in launched_processes:
        if proc.poll() is None:
            proc.terminate()
    time.sleep(1)
    for proc in launched_processes:
        if proc.poll() is None:
            proc.kill()

    # Step 5.5: clean Gazebo and DDS state
    print("   Cleaning Gazebo lock files and DDS shared memory...")
    run_ros(
        "rm -rf /tmp/gazebo-* /tmp/.gazebo* 2>/dev/null; "
        "rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null; "
        "rm -f /tmp/gz_* 2>/dev/null; "
        "true"
    )

    # Step 6: stop ROS daemon
    run_ros("source /opt/ros/humble/setup.bash && ros2 daemon stop 2>/dev/null || true")

    print("   Verifying cleanup...")
    verify_regex = (
        "gazebo|gzserver|gzclient|move_group|controller_manager|ros2_control|"
        "spawn_entity|motion_server|pump_controller|wsl_ros_bridge|denso_robot|"
        "staubli_|robot_interface_streaming|robot_interface|ufactory_|xarm|ros2 launch"
    )
    result = run_ros(
        f"pgrep -af {shlex.quote(verify_regex)} "
        "| grep -v pgrep | grep -v 'bash -c' || echo CLEAN",
        capture=True,
    )
    leftovers = result.stdout.strip()
    if leftovers and leftovers != "CLEAN":
        print(f"     Leftover processes detected:\n{leftovers}")
        print("   Force-killing leftovers...")
        run_ros(
            f"pgrep -f {shlex.quote(verify_regex)} "
            "| xargs -r kill -9 2>/dev/null || true"
        )
    else:
        print("     All processes terminated")
        
    port_check = run_ros("ss -tln | grep ':11345' || echo PORT_FREE", capture=True)
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
    if started_container:
        print("   Stopping the ros-humble container...")
        stop_backend()
    print("\nClean shutdown complete.")
    sys.exit(0)

# --- Ctrl+C signal handler ---------------------------------------------------

def handle_sigint(sig, frame):
    print("\n\nCtrl+C detected, shutting down...")
    cleanup()

signal.signal(signal.SIGINT, handle_sigint)

# --- Main --------------------------------------------------------------------

def main():
    global started_container
    started_container = ensure_backend()

    mode = "visible" if SHOW_TERMINALS else "hidden (background)"
    print(f"Starting {MODEL} stack (mode: {mode})...\n")

    # print("Cleaning leftover state from previous runs...")
    # kill_wsl_processes()
    # time.sleep(2)


    total = 3 if ((IS_STAUBLI and SIM == "true") or TERMINAL_3 is None) else 4
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

    if (IS_STAUBLI and SIM == "false"):
        time.sleep(5)
        print(f"[{step}/{total}] Starting Staubli robot_interface_streaming...")
        launch_wsl_tab(TAB_TITLES[4], TERMINAL_5)
        step += 1


    if TERMINAL_3 is not None:
        print(f"[{step}/{total}] Starting Pump Control...")
        launch_wsl_tab(TAB_TITLES[2], TERMINAL_3)
        step += 1

    print(f"[{step}/{total}] Starting HTTP Bridge...")
    launch_wsl_tab(TAB_TITLES[3], TERMINAL_4)

    print(f"\nAll {total} ROS processes launched (mode: {mode}).")
    print("Press Ctrl+C to stop everything.\n")

    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()
