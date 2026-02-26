# Controlling a DENSO (VS060) Robot in Simulation from Windows
### via ROS 2 Humble + MoveIt 2 (WSL) and Python (Windows)

This document provides a **step-by-step** guide on how to:
- Launch the **simulation** of the DENSO VS060 robot under **WSL (Ubuntu + ROS 2 Humble)**.
- Launch the **MoveIt motion server**.
- Launch an **HTTP bridge**.
- **Control the robot from Windows using Python**.

---

## 1. General Architecture

The communication flow between the Windows host and the WSL simulation is as follows:

> **Windows (Python)** > → HTTP (`requests`) 
> → **WSL Ubuntu** > → FastAPI / Uvicorn (Bridge) 
> → ROS 2 Humble 
> → `denso_motion_control` (C++ / MoveIt 2) 
> → Gazebo / RViz Simulation

---

## 2. Prerequisites

### WSL Side
- Ubuntu WSL2
- ROS 2 Humble
- Compiled workspace (`denso_ros2_ws`)
- Packages:
  - `denso_robot_bringup`
  - `denso_robot_moveit_config`
  - `denso_motion_control`
- Python 3.10
- Virtual environment (`venv`)
- Python modules: `requests`, `numpy`, `uvicorn`, `fastapi`

### Windows Side
- Python 3.10
- Virtual environment (`venv`)
- Python modules: `requests`

---

## 3. WSL Setup & Installation

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
export ROS_APT_SOURCE_VERSION=$(curl -s [https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest](https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest) | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "[https://github.com/ros-infrastructure/ros-apt-source/releases/download/$](https://github.com/ros-infrastructure/ros-apt-source/releases/download/$){ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
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

### 4. Getting the Code & Compilation
#### Modification in rosdep to correct certificates problem on WSL

find in your wsl file usr/lib/python/sit-packages/rosdep2/url_utils.py
or if your using a venv YOUR_VENV_PATH/lib/python/sit-packages/rosdep2/url_utils.py
JUST BEFORE the function : 
```python
def urlopen_gzip(url, **kwargs):
```
ADD THIS : 
```ptyhon
import ssl
ssl._create_default_https_context = ssl._create_stdlib_context
```

#### WSL Environment (Linux Workspace)
Fetch the repository and build the ROS 2 workspace:

```bash
mkdir ~/workspace
cd ~/workspace
git clone [https://github.com/Ameno1215/stageroslinux](https://github.com/Ameno1215/stageroslinux)

# Initialize and update rosdep
sudo rosdep init
sudo rosdep update

# Install dependencies and build
cd ~/workspace/denso_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```


#### Windows Environment
Fetch the Windows repository and switch to the demo branch:

PowerShell
```bash
git clone [https://github.com/Ameno1215/stageroswindows](https://github.com/Ameno1215/stageroswindows)
cd stageroswindows
git switch demo
5. Launching the Simulation (WSL)
```


### 5. Launching the Simulation (WSL)
#### Important: The following setup commands must be executed in every new WSL terminal before running the launch commands:
```bash
cd ~/workspace/denso_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1

# if you've got a NVIDIA graphic card 
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
```

#### WSL Terminal #1: Launch Bringup (Gazebo & RViz)

```bash
ros2 launch denso_robot_bringup denso_robot_bringup.launch.py model:=vs060 sim:=true tool:=effecteur_v1 ik_solver:=kdl
```

#### WSL Terminal #2: Launch Motion Server
```bash
ros2 launch denso_motion_control motion_server.launch.py model:=vs060 sim:=true tool:=effecteur_v1 ik_solver:=kdl
```

#### WSL Terminal #3: Launch HTTP Bridge (FastAPI)
```bash
cd ~/workspace
source venv/bin/activate
source /opt/ros/humble/setup.bash
source ~/workspace/denso_ros2_ws/install/setup.bash
uvicorn wsl_ros_bridge:app --host 0.0.0.0 --port 8000
```

### 6. Running the Demo (Windows)
Once the simulation, motion server, and HTTP bridge are running in WSL, open a PowerShell terminal on Windows to execute the robot commands:
```bash
PowerShell
.\venv\Scripts\activate
python ./test.py
```