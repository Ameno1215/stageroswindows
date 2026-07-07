import subprocess
import time
import argparse
import shlex

# --- CLI args ----------------------------------------------------------------

parser = argparse.ArgumentParser(description="Stop the DENSO/Staubli ROS 2 stack")
parser.add_argument("--model", choices=["vs060", "vp5243", "tx40"], default="vs060",
                    help="Robot model used at launch (default: vs060)")
parser.add_argument("--real-robot", action="store_true",
                    help="Real robot mode: waits longer for graceful b-CAP disconnection")
args = parser.parse_args()

MODEL = args.model
SIM = "false" if args.real_robot else "true"


# --- WSL process cleanup -----------------------------------------------------

def kill_wsl_processes():
    """Kills ROS 2, Gazebo and Uvicorn processes on the WSL side."""
    print("Stopping WSL processes...")

    launch_targets = [
        "ros2 launch denso_robot_bringup",
        "denso_robot_bringup.launch.py",
        "ros2 launch motion_control",
        "motion_server.launch.py",
        "ros2 launch command_pump_denso",
        "ros2 launch command_pump_staubli",
        "pump_controller.launch.py",
        f"ros2 launch staubli_{MODEL}_moveit_config",
        f"staubli_{MODEL}_planning_execution",
        "ros2 launch staubli_val3_driver",
        "robot_interface_streaming.launch.py",
        "uvicorn wsl_ros_bridge:app",
    ]

    node_targets = [
        "denso_robot", "move_group", "robot_state_publisher", "rviz2",
        "motion_server", "motion_control",
        "pump_controller", "command_pump_denso", "command_pump_staubli",
        "staubli_val3_driver", "robot_interface_streaming", "robot_interface",
        "gzserver", "gzclient", "gazebo", "gzweb",
        "spawn_entity", "controller_manager", "ros2_control_node",
        "uvicorn",
    ]

    all_targets = launch_targets + node_targets

    def pkill(signal_name, targets):
        for target in targets:
            target = shlex.quote(target)
            subprocess.run(
                ["wsl.exe", "bash", "-c", f"pkill -{signal_name} -f -- {target} 2>/dev/null || true"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )

    # --- Step 1: SIGINT on ros2 launch supervisors first
    print("   Sending SIGINT to ros2 launch supervisors...")
    pkill("SIGINT", launch_targets)
    time.sleep(2)

    # --- Step 2: SIGINT on ROS2 nodes (allows on_deactivate to run)
    print("   Sending SIGINT to ROS2 nodes...")
    pkill("SIGINT", node_targets)

    # --- Step 3: Long wait for the RC8 controller to close the b-CAP session
    wait_time = 10 if SIM == "false" else 2
    print(f"   Waiting {wait_time}s for graceful controller disconnection...")
    time.sleep(wait_time)

    # --- Step 4: SIGTERM for any remaining launch supervisors and nodes
    print("   Sending SIGTERM to remaining processes...")
    pkill("SIGTERM", all_targets)
    time.sleep(2)

    # --- Step 5: SIGKILL as last resort only
    print("   Force-killing remaining processes...")
    pkill("9", all_targets)

    # Kill orphaned ros2 launch processes for this workspace stack.
    launch_regex = (
        "ros2 launch (denso_robot_bringup|motion_control|command_pump_denso|"
        f"command_pump_staubli|staubli_{MODEL}_moveit_config|staubli_val3_driver)"
    )
    subprocess.run(
        ["wsl.exe", "bash", "-c",
         f"pgrep -f {shlex.quote(launch_regex)} | xargs -r kill -9 2>/dev/null || true"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    # Step 5.5: clean Gazebo and DDS state
    print("   Cleaning Gazebo lock files and DDS shared memory...")
    subprocess.run(
        ["wsl.exe", "bash", "-c",
         "rm -rf /tmp/gazebo-* /tmp/.gazebo* 2>/dev/null; "
         "rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null; "
         "rm -f /tmp/gz_* 2>/dev/null; "
         "true"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    # Step 6: stop ROS daemon
    subprocess.run(
        ["wsl.exe", "bash", "-c", "ros2 daemon stop 2>/dev/null || true"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    # --- Verification
    print("   Verifying cleanup...")
    verify_regex = (
        "gazebo|gzserver|gzclient|move_group|controller_manager|ros2_control|"
        "spawn_entity|motion_server|pump_controller|uvicorn|denso_robot|"
        "staubli_|robot_interface_streaming|robot_interface|ros2 launch"
    )
    result = subprocess.run(
        ["wsl.exe", "bash", "-c",
         f"pgrep -af {shlex.quote(verify_regex)} "
         "| grep -v pgrep | grep -v 'bash -c' || echo CLEAN"],
        capture_output=True, text=True,
    )
    leftovers = result.stdout.strip()
    if leftovers and leftovers != "CLEAN":
        print(f"     Leftover processes detected:\n{leftovers}")
        print("   Force-killing leftovers...")
        subprocess.run(
            ["wsl.exe", "bash", "-c",
             f"pgrep -f {shlex.quote(verify_regex)} "
             "| xargs -r kill -9 2>/dev/null || true"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    else:
        print("     All processes terminated")

    # --- Port check (Gazebo master 11345)
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


# --- Optional: close the 'denso_stack' Windows Terminal window ----------------

def close_terminal_window():
    """Closes the Windows Terminal tabs belonging to the denso_stack window (best effort)."""
    subprocess.run(
        ["taskkill", "/FI", "WINDOWTITLE eq *denso_stack*", "/T", "/F"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )


# --- Main --------------------------------------------------------------------

def main():
    print(f"Stopping {MODEL} stack (sim={SIM})...\n")
    kill_wsl_processes()
    # close_terminal_window()  # uncomment to also close the WT window
    print("\nClean shutdown complete.")


if __name__ == "__main__":
    main()