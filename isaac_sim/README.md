# Isaac Sim setup for DENSO VS060

This workspace now includes an Isaac-specific path that does not modify the existing Gazebo workflow.

## 1. Build the new ROS 2 package

```bash
cd ~/workspace/denso_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select isaac_joint_trajectory_bridge motion_control denso_robot_bringup denso_robot_descriptions
source install/setup.bash
```

## 2. Export the Isaac-safe URDF

This URDF keeps the robot geometry and tool chain but removes the Gazebo plugin and ros2_control block.

```bash
cd ~/workspace/denso_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
bash -lc 'python3 /home/antonin/workspace/isaac_sim/export_isaac_bundle.py'
```

This creates a Windows-friendly bundle by copying the URDF and all referenced meshes to:

`C:\Users\33648\Desktop\STAGE_ROS\isaac_sim\import_bundle`

Import `C:\Users\33648\Desktop\STAGE_ROS\isaac_sim\import_bundle\vs060_isaac.urdf` with the Isaac Sim URDF importer.

## 3. Configure Isaac Sim

- Enable the ROS 2 bridge extension.
- Make sure Windows and WSL use the same `ROS_DOMAIN_ID`.
- Set `FASTRTPS_DEFAULT_PROFILES_FILE` on both Windows and WSL if you are using the WSL2 bridge path.
- Import the VS060 URDF and note the articulation prim path.
- Edit `ROBOT_PRIM_PATH` in `/home/antonin/workspace/isaac_sim/setup_vs060_ros2_bridge.py` if your imported prim path is different.
- Run `/home/antonin/workspace/isaac_sim/setup_vs060_ros2_bridge.py` once from Isaac Sim Script Editor.

The Isaac graph publishes `/joint_states_raw` and subscribes to `/joint_command`.
The ROS side restamps `/joint_states_raw` to `/joint_states` for MoveIt.

## 3.b Fast DDS files for Windows and WSL

You need the same `fastdds.xml` content on both sides:

- WSL target: `~/.ros/fastdds.xml`
- Windows target: `C:\.ros\fastdds.xml`

Templates and installers are included here:

- template: `/home/antonin/workspace/isaac_sim/fastdds/fastdds.xml`
- WSL installer: `/home/antonin/workspace/isaac_sim/install_fastdds_wsl.sh`
- Windows installer: `/home/antonin/workspace/isaac_sim/install_fastdds_windows.ps1`

WSL install:

```bash
bash /home/antonin/workspace/isaac_sim/install_fastdds_wsl.sh 0
```

Windows install:

```powershell
powershell -ExecutionPolicy Bypass -File .\isaac_sim\install_fastdds_windows.ps1 0
```

If your workspace is not at `C:\Users\antonin\workspace`, update `SourceFile` inside `install_fastdds_windows.ps1`.

## 3.c WSL2 port forwarding

If Isaac Sim still does not see ROS 2 topics across Windows/WSL, NVIDIA documents adding port proxy rules for Fast DDS ports `7400`, `7410`, and `9387`.

In elevated PowerShell:

```powershell
$Windows_IP = "<WINDOWS_IP>"
$WSL2_IP = "<WSL2_IP>"
netsh interface portproxy add v4tov4 listenport=7400 listenaddress=$Windows_IP connectport=7400 connectaddress=$WSL2_IP
netsh interface portproxy add v4tov4 listenport=7410 listenaddress=$Windows_IP connectport=7410 connectaddress=$WSL2_IP
netsh interface portproxy add v4tov4 listenport=9387 listenaddress=$Windows_IP connectport=9387 connectaddress=$WSL2_IP
```

## 4. Launch the ROS stack

From Windows:

```powershell
python .\launch_denso_stack.py --sim-backend isaac --launch-isaac --show-terminals
```

If Isaac Sim is not found automatically:

```powershell
python .\launch_denso_stack.py --sim-backend isaac --launch-isaac --isaac-path "C:\Users\<you>\AppData\Local\ov\pkg\isaac-sim-...\isaac-sim.bat"
```

## 5. What changed

- `denso_robot_isaac.urdf.xacro` is a new robot description dedicated to Isaac Sim.
- `isaac_joint_trajectory_bridge` exposes `denso_joint_trajectory_controller/follow_joint_trajectory` for MoveIt and translates trajectories to `/joint_command`.
- The default `denso_robot_bringup.launch.py` still starts Gazebo by default.
