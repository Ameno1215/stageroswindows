"""
Where the ROS 2 stack runs, seen from the client:
  - Windows : inside WSL (Ubuntu 22.04), reached with wsl.exe
  - Linux   : inside the ros-humble Docker container (see ../stageroslinux/docker_run.sh)
"""
import os
import shutil
import subprocess
from pathlib import Path

IS_WINDOWS = os.name == "nt"

PROJECT_DIR = Path(__file__).resolve().parents[2]          # .../ROBOT IN GROUPE
DOCKER_RUN_SCRIPT = PROJECT_DIR / "stageroslinux" / "docker_run.sh"
CONTAINER_NAME = "ros-humble"

if IS_WINDOWS:
    ROS_WS = "~/workspace/ros2_ws"
    BRIDGE_DIR = "~/workspace"
    BRIDGE_CMD = "source venv/bin/activate && python wsl_ros_bridge.py"
else:
    ROS_WS = "~/stageroslinux/ros2_ws"
    BRIDGE_DIR = "~/stageroslinux"
    # Python deps are installed system-wide in the image, no venv
    BRIDGE_CMD = "python3 wsl_ros_bridge.py --log-name robot_system.log"


def ros_argv(bash_cmd, tty=False):
    """argv that runs `bash_cmd` in the ROS environment (WSL or container)."""
    if IS_WINDOWS:
        return ["wsl.exe", "bash", "-c", bash_cmd]
    return ["docker", "exec"] + (["-it"] if tty else []) + [CONTAINER_NAME, "bash", "-c", bash_cmd]


def run_ros(bash_cmd, capture=False):
    """Runs a short command in the ROS environment and waits for it."""
    if capture:
        return subprocess.run(ros_argv(bash_cmd), capture_output=True, text=True)
    return subprocess.run(ros_argv(bash_cmd), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def container_running():
    result = subprocess.run(
        ["docker", "ps", "-q", "-f", f"name=^{CONTAINER_NAME}$"],
        capture_output=True, text=True,
    )
    return bool(result.stdout.strip())


def ensure_backend():
    """Linux only: starts the container if needed. Returns True if it was started here."""
    if IS_WINDOWS or container_running():
        return False
    print("   Starting the ros-humble container...")
    subprocess.run([str(DOCKER_RUN_SCRIPT), "start"], check=True)
    return True


def stop_backend():
    if not IS_WINDOWS and container_running():
        subprocess.run(["docker", "stop", CONTAINER_NAME],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def open_ros_tab(title, bash_cmd, visible):
    """Runs a long-lived command in the ROS environment, in a terminal tab or hidden."""
    if visible:
        wrapped_cmd = (
            f"({bash_cmd}); "
            f"echo; "
            f"echo '--- Process ended (press Enter or Ctrl+D to close) ---'; "
            f"exec bash"
        )
        if IS_WINDOWS:
            wrapped_cmd_escaped = wrapped_cmd.replace("; ", " \\; ")
            return subprocess.Popen(
                ["wt.exe", "-w", "denso_stack", "new-tab",
                 "--title", title, "--", "wsl.exe", "bash", "-c", wrapped_cmd_escaped]
            )
        if not shutil.which("gnome-terminal"):
            raise RuntimeError("gnome-terminal not found: run without --show-terminals")
        return subprocess.Popen(
            ["gnome-terminal", "--tab", "--title", title, "--"] + ros_argv(wrapped_cmd, tty=True)
        )

    if IS_WINDOWS:
        return subprocess.Popen(
            ros_argv(bash_cmd),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            creationflags=subprocess.CREATE_NO_WINDOW,
        )
    # Own session: Ctrl+C in the launcher must not hit the stack before the ordered shutdown
    return subprocess.Popen(
        ros_argv(bash_cmd),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
