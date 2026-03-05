import subprocess
import time
import signal
import sys

# --- Config ------------------------------------------------------------------

SHOW_TERMINALS = True  # Set to False to hide WSL terminals

# --- Commands ----------------------------------------------------------------

SETUP = (
    "cd ~/workspace/denso_ros2_ws && "
    "source /opt/ros/humble/setup.bash && "
    "source install/setup.bash && "
    "export LIBGL_ALWAYS_SOFTWARE=1"
)

TERMINAL_1 = (
    f"{SETUP} && "
    "ros2 launch denso_robot_bringup denso_robot_bringup.launch.py "
    "model:=vs060 sim:=true tool:=effecteur_v1 ik_solver:=kdl"
)

TERMINAL_2 = (
    f"{SETUP} && "
    "ros2 launch motion_control motion_server.launch.py "
    "model:=vs060 sim:=true tool:=effecteur_v1 ik_solver:=kdl"
)

TERMINAL_3 = (
    "cd ~/workspace && "
    "source venv/bin/activate && "
    "source /opt/ros/humble/setup.bash && "
    "source ~/workspace/denso_ros2_ws/install/setup.bash && "
    "uvicorn wsl_ros_bridge:app --host 0.0.0.0 --port 8000"
)

TAB_TITLES = ["DENSO_Bringup", "DENSO_MotionServer", "DENSO_Bridge"]

# --- Launched processes ------------------------------------------------------

launched_processes = []

# --- Launch ------------------------------------------------------------------

def launch_wsl_tab(title, bash_cmd):
    """Opens a visible terminal or a hidden background process depending on SHOW_TERMINALS."""

    if SHOW_TERMINALS:
        proc = subprocess.Popen(
            ["wt.exe", "new-tab", "--title", title, "--", "wsl.exe", "bash", "-c", bash_cmd]
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

    targets = [
        "ros2",
        "gzserver",
        "gzclient",
        "gazebo",
        "rviz2",
        "uvicorn",
        "robot_state_publisher",
        "move_group",
    ]

    # First pass: SIGTERM
    for target in targets:
        subprocess.run(
            ["wsl.exe", "bash", "-c", f"pkill -f {target} 2>/dev/null || true"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    print("   Waiting for graceful shutdown (3s)...")
    time.sleep(3)

    # Second pass: SIGKILL
    for target in targets:
        subprocess.run(
            ["wsl.exe", "bash", "-c", f"pkill -9 -f {target} 2>/dev/null || true"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    # Terminate background Python-side process handles if hidden mode
    if not SHOW_TERMINALS:
        for proc in launched_processes:
            try:
                proc.terminate()
            except Exception:
                pass

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
    print(f"Starting DENSO VS060 simulation (mode: {mode})...\n")

    print("[1/3] Starting Gazebo & RViz...")
    launch_wsl_tab(TAB_TITLES[0], TERMINAL_1)

    print("      Waiting 5s for Gazebo to start...")
    time.sleep(5)

    print("[2/3] Starting Motion Server...")
    launch_wsl_tab(TAB_TITLES[1], TERMINAL_2)

    print("      Waiting 2s...")
    time.sleep(2)

    print("[3/3] Starting HTTP Bridge...")
    launch_wsl_tab(TAB_TITLES[2], TERMINAL_3)

    print(f"\nAll 3 WSL processes launched (mode: {mode}).")
    print("Press Ctrl+C to stop everything.\n")

    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()