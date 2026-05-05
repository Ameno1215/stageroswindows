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
> → `motion_control` (C++ / MoveIt 2) 
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
  - `motion_control`
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

### 4. Getting the Code & Compilation
#### Modification in rosdep to correct certificates problem on WSL

Use :  `which rosdep`    
to find `/usr/bin/rosdep`    
find in your wsl file `usr/lib/python/sit-packages/rosdep2/url_utils.py`    
or if your using a venv `YOUR_VENV_PATH/lib/python/sit-packages/rosdep2/url_utils.py`

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
colcon build
```


#### Windows Environment
Fetch the Windows repository and switch to the demo branch:

PowerShell
```bash
git clone [https://github.com/Ameno1215/stageroswindows](https://github.com/Ameno1215/stageroswindows)
cd stageroswindows
```


### 5. Launching the Simulation (Windows)
```bash
PowerShell
.\venv\Scripts\activate
python .\launch_controller.py --model vs060
```

### 6. Running the Demo (Windows)
Once the simulation, motion server, and HTTP bridge are running in WSL, open a PowerShell terminal on Windows to execute the robot commands:
```bash
PowerShell
.\venv\Scripts\activate
python ./test.py --model vs060
```