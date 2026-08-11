# Controlling a DENSO or Stäubli Robot from Windows
### via ROS 2 Humble + MoveIt 2 (WSL) and Python (Windows)

This document provides a **step-by-step** guide on how to:
- Launch the **simulation** (or the **real robot**) under **WSL (Ubuntu + ROS 2 Humble)**.
- Launch the **MoveIt motion server**.
- Launch the **pump/IO controller** and an **HTTP bridge**.
- **Control the robot from Windows using Python**.

Supported models (`--model`):

| Model | Brand | ROS packages |
|---|---|---|
| `vs060` *(default)* | DENSO | `denso_robot_bringup`, `denso_robot_moveit_config` |
| `vp5243` | DENSO | idem |
| `tx40` | Stäubli | `staubli_tx40_moveit_config`, `staubli_val3_driver` |

To integrate a **new** robot brand/model, see [adding_a_new_robot.md](adding_a_new_robot.md).

---

## 1. General Architecture

The communication flow between the Windows host and the WSL simulation is as follows:

> **Windows (Python)** > → HTTP (`requests`) 
> → **WSL Ubuntu** > → FastAPI / Uvicorn (Bridge) 
> → ROS 2 Humble 
> → `motion_control` (C++ / MoveIt 2) 
> → Gazebo / RViz Simulation

`launch_controller.py` starts four WSL processes, one per terminal tab:

| Tab | Process | Role |
|---|---|---|
| `Bringup` | `denso_robot_bringup` / `staubli_tx40_moveit_config` | robot description, `ros2_control`, `move_group`, RViz (+ Gazebo) |
| `MotionServer` | `motion_control` | vendor-agnostic motion services on top of MoveIt |
| `Pump Control` | `command_pump_denso` / `command_pump_staubli` | vacuum pump and digital IO |
| `WSL_Bridge` | `wsl_ros_bridge.py` | FastAPI server exposing the motion services over HTTP |

On the real Stäubli robot, a fifth tab (`Staubli_Connection`) runs `staubli_val3_driver`.

---

## 2. Prerequisites

### WSL Side
- Ubuntu WSL2
- ROS 2 Humble
- Compiled workspace (`ros2_ws`)
- Packages:
  - `denso_robot_bringup`, `denso_robot_moveit_config`, `denso_robot_descriptions`
  - `staubli_tx40_moveit_config`, `staubli_val3_driver` *(Stäubli only)*
  - `motion_control` — the motion services used by the bridge
  - `command_pump_denso` / `command_pump_staubli` — pump and IO control
  - `tool` — end-effector descriptions (`effecteur_v1`, `effecteur_v2`, `effecteur_v3`)
  - `joint_monitor` — joint/TCP state publishing and trajectory plotting
- Python 3.10 or higher
- Virtual environment (`venv`)
- Python modules: `requests`, `numpy`, `uvicorn`, `fastapi`

### Windows Side
- Python 3.10
- Virtual environment (`venv`)
- Python modules: `requests`

---

## 3. WSL Setup & Installation


### Install wsl and Ubuntu 22.04
To install Ubuntu 22.04 run

```bash
wsl --install -d Ubuntu-22.04
```


### ROS 2 Humble Installation
Run the following commands in your WSL terminal to set up the locale and install ROS 2 Humble:

```bash
locale  # Check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # Verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller ros-humble-xacro
	
sudo apt install -y ros-humble-moveit ros-humble-moveit-planners-ompl ros-humble-moveit-ros-visualization ros-humble-pick-ik
```

### Testing the ROS 2 Installation
To verify that both the C++ and Python APIs are working properly, test the talker/listener nodes:

#### In WSL Terminal #1:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

#### In WSL Terminal #2:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
```

You should see the talker publishing messages and the listener receiving them.

## 4. Getting the Code & Compilation
#### Modification in rosdep to correct certificates problem on WSL (ONLY IN GROUP COMPUTER)

Use :  `which rosdep`    
to find `/usr/bin/rosdep`    
find in your wsl file `usr/lib/python/sit-packages/rosdep2/url_utils.py`    
or if your using a venv `YOUR_VENV_PATH/lib/python/sit-packages/rosdep2/url_utils.py`

JUST BEFORE the function : 

```python
def urlopen_gzip(url, **kwargs):
```

ADD THIS : 

```python
import ssl
ssl._create_default_https_context = ssl._create_stdlib_context
```

#### WSL Environment (Linux Workspace)
Fetch the repository and build the ROS 2 workspace:

```bash
mkdir ~/workspace
cd ~/workspace
git clone https://github.com/Ameno1215/stageroslinux.git .
apt install python3-venv -y
python3 -m venv venv
source venv/bin/activate
pip install requests numpy uvicorn fastapi

# Initialize and update rosdep
sudo rosdep init
sudo rosdep update

# Install dependencies and build
cd ~/workspace/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```


#### Windows Environment

PowerShell

```bash
git clone https://github.com/Ameno1215/stageroswindows.git
python -m venv venv
.\venv\Scripts\activate
pip install requests
cd stageroswindows
```


## 5. Launching the Stack (Windows)

```bash
PowerShell
.\venv\Scripts\activate
python .\launch_controller.py --model vs060
```

### Options

| Option | Default | Description |
|---|---|---|
| `--model {vs060,vp5243,tx40}` | `vs060` | Robot model to launch |
| `--real-robot` | *off* (simulation) | Connect to the physical robot instead of the simulation |
| `--ip <address>` | `169.254.75.249` | Robot IP, used with `--real-robot` |
| `--tool <name>` | `effecteur_v3` | End-effector to attach (`none`, `effecteur_v1`, `effecteur_v2`, `effecteur_v3`) |
| `--show-terminals` | *off* | Open one visible Windows Terminal tab per process instead of running hidden |
| `--accuracy` | *off* | Log the real-vs-planned trajectory error in debug mode |

Examples:

```bash
# Simulation, visible terminals (recommended while debugging)
python .\launch_controller.py --model vs060 --show-terminals

# Stäubli TX40 in simulation, no tool
python .\launch_controller.py --model tx40 --tool none

# Real DENSO robot
python .\launch_controller.py --model vs060 --real-robot --ip 169.254.75.249
```

Press `Ctrl+C` in the launcher to stop the whole stack; it sends `SIGINT`, then `SIGTERM`,
then `SIGKILL` to the WSL processes and cleans the Gazebo/DDS leftovers.

If the launcher was killed without cleaning up (or a previous run left processes behind),
run the standalone cleanup script before relaunching:

```bash
python .\stop_stack.py --model vs060
```

---

## 6. Running the Demo (Windows)

Once the stack is running in WSL, open a PowerShell terminal on Windows to execute the
robot commands:

```bash
PowerShell
.\venv\Scripts\activate
python .\test.py --model vs060
```

`test.py` accepts `--model {vs060,vp5243,tx40}` and `--real-robot`.

---

## 7. Using the Command Module

All robot commands go through **`motion_http_client.py`**, which exposes the
`MotionRobotClient` class — one Python method per HTTP endpoint of the bridge
(`init_robot`, `move_to_pose`, `move_joints`, `move_waypoints`, `move_approach`,
`set_scaling`, `manage_box`, `manage_mesh`, `set_virtual_fence`, `pump_grab`,
`pump_release`, `set_servo_on`, …).

**`EXAMPLE.py` is the reference example of how to use this module.** It is a complete,
runnable card-handling scenario that shows, in order:

1. connecting and checking the bridge (`health`),
2. initialising the robot (`init_robot`) and setting the speed/acceleration scaling,
3. loading the collision environment from `env/<model>.json` (`load_environnement`) —
   virtual fence, boxes and meshes,
4. powering the motors (`set_servo_on`) and going home (`move_to_home`),
5. adding/removing collision objects on the fly (`manage_box`, `manage_mesh`),
6. moving with `move_to_pose` / `move_approach`, in joint or Cartesian mode,
7. picking and dropping with the vacuum pump (`pump_grab`, `pump_is_grabbed`,
   `pump_release`),
8. cleaning up the scene and switching the motors off.

```bash
PowerShell
.\venv\Scripts\activate
python .\EXAMPLE.py --model vs060
```

> Before running it, adapt the absolute paths at the top of the `run()` function — the
> environment files are referenced as `C:/Users/<user>/.../env/<model>.json`. The
> environment description itself (fence dimensions, boxes, meshes) is **per robot** and
> lives in `env/vs060.json` and `env/tx40.json`; its schema is defined in `manage_env.py`.

---

## 8. Adding a New Robot

To integrate a new robot brand/model (UFACTORY, Universal Robots, ...) into this stack,
see [adding_a_new_robot.md](adding_a_new_robot.md).

---

## 9. Logging

The bridge writes its logs to `~/workspace/log/` and also prints them to the console.
It can be tuned when started manually:

```bash
python wsl_ros_bridge.py --host 0.0.0.0 --port 8000 --log-level DEBUG --add-date
```

| Option | Default | Description |
|---|---|---|
| `--host` | `127.0.0.1` | Bind address |
| `--port` | `8000` | Bind port |
| `--log-path` | `log/` next to the script | Log directory |
| `--log-name` | derived from the script name | Log file name |
| `--log-level` | `DEBUG` | Logging level |
| `--add-date` / `--date-fmt` | *off* / `%Y-%m-%d` | Append a date to the log file name |
| `--no-console` | *off* | Write to file only |

On the Windows side, `logger_worker.py` mirrors the WSL log stream into the client
terminal, so a single console shows both sides of a run.
