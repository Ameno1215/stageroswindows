# Isaac Sim Stack Launch Guide — VS060 / ROS 2

> Full procedure to bring up the Denso stack in **Isaac Sim mode**.

---

## Recommended Order

1. Build WSL
2. Export URDF bundle
3. Install Fast DDS (Windows)
4. Import URDF in Isaac Sim
5. Run `setup_vs060_ros2_bridge.py`
6. Hit **Play**
7. Launch `launch_denso_stack.py`

---

## Step 1 — Build (WSL)

```bash
cd ~/workspace/denso_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
    isaac_joint_trajectory_bridge \
    motion_control \
    denso_robot_bringup \
    denso_robot_descriptions
source install/setup.bash
bash /home/antonin/workspace/isaac_sim/install_fastdds_wsl.sh 0
```

---

## Step 2 — Export URDF Bundle (WSL)

```bash
python3 /home/antonin/workspace/isaac_sim/export_isaac_bundle.py \
    --output-dir /mnt/c/Users/33648/Desktop/STAGE_ROS/isaac_sim/import_bundle
```

**Output on Windows side:**

```
C:\Users\33648\Desktop\STAGE_ROS\isaac_sim\import_bundle\
├── vs060_isaac.urdf
└── meshes\...
```

---

## Step 3 — Update Windows Files

Make sure the following files are up to date in `C:\Users\33648\Desktop\STAGE_ROS`:

| File | Location |
|------|----------|
| `launch_denso_stack.py` | `STAGE_ROS\` |
| `setup_vs060_ros2_bridge.py` | `STAGE_ROS\isaac_sim\` |
| `install_fastdds_windows.ps1` | `STAGE_ROS\isaac_sim\` |
| `fastdds.xml` | `STAGE_ROS\isaac_sim\fastdds\` |

In `install_fastdds_windows.ps1`, verify the source path:

```powershell
$SourceFile = "C:\Users\33648\Desktop\STAGE_ROS\isaac_sim\fastdds\fastdds.xml"
```

---

## Step 4 — Install Fast DDS (Windows)

In a **PowerShell** window from `C:\Users\33648\Desktop\STAGE_ROS`:

```powershell
powershell -ExecutionPolicy Bypass -File .\isaac_sim\install_fastdds_windows.ps1 0
```

> ⚠️ **Open a new PowerShell window after this step** so the environment variables take effect.

---

## Step 5 — Import Robot in Isaac Sim

1. **Enable** the ROS 2 Bridge extension.
2. **Import** the URDF:
   ```
   C:\Users\33648\Desktop\STAGE_ROS\isaac_sim\import_bundle\vs060_isaac.urdf
   ```
3. **Check** the robot's prim path in the Stage panel.  
   The script assumes:
   ```
   ROBOT_PRIM_PATH = "/World/vs060"
   ```
   If different, edit `setup_vs060_ros2_bridge.py` accordingly.
4. **Run** `setup_vs060_ros2_bridge.py` in the Script Editor.
5. **Press Play** ▶️

> The Isaac script publishes on `/joint_states_raw` and listens on `/joint_command`.

---

## Step 6 — Launch the ROS Stack (Windows)

In PowerShell from `C:\Users\33648\Desktop\STAGE_ROS`:

```powershell
python .\launch_denso_stack.py --sim-backend isaac --launch-isaac --show-terminals
```

---

## Step 7 — Verify Everything is Up

In the WSL terminals, check that the following are running without crashes:

- `follow_joint_trajectory_bridge`
- `isaac_joint_state_relay`
- `move_group`
- `motion_server`

Then verify in WSL:

```bash
source ~/workspace/denso_ros2_ws/install/setup.bash
ros2 topic echo /joint_states --once
ros2 service list | grep init_robot
```

**Expected results:**

| Check | Expected |
|-------|----------|
| `/joint_states` | Publishing joint positions |
| `/init_robot` | Present in service list |