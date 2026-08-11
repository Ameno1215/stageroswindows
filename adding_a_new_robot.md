# Adding a New Robot to the Stack

### Worked example: integrating a **UFACTORY xArm 6**

This document explains everything that has to be created, modified and verified to add a
new robot brand/model to this project, on **both** repositories:

| Repository | Role |
|---|---|
| [`stageroslinux`](https://github.com/Ameno1215/stageroslinux) | WSL side — ROS 2 workspace (`ros2_ws`) + HTTP bridge (`wsl_ros_bridge.py`) |
| [`stageroswindows`](https://github.com/Ameno1215/stageroswindows) | Windows side — stack launcher, Python client, environment descriptions |

Two robot families are already integrated and act as reference implementations:

* **DENSO** (`vs060`, `vp5243`, `cobotta`, `hsr065a1_n32`) — model-parameterised inside a single
  set of packages.
* **Stäubli** (`tx40`) — one dedicated package set per model.

UFACTORY is used throughout as the example, but the procedure applies to any vendor
(Universal Robots, Fanuc, Kuka…).

---

## 1. How a "model" flows through the stack

```
Windows                                  WSL (Ubuntu 22.04 + ROS 2 Humble)
─────────────────────────────────────    ────────────────────────────────────────────────
launch_controller.py --model xarm6
   │
   ├─ TERMINAL_1  ───────────────────►   bringup / planning_execution launch
   │                                       └─ robot_state_publisher + ros2_control
   │                                          + move_group + RViz (+ Gazebo)
   │
   ├─ TERMINAL_2  ───────────────────►   ros2 launch motion_control motion_server.launch.py
   │                                       model:=ufactory_xarm6 sim:=… tool:=… ik_solver:=…
   │                                       └─ builds URDF + SRDF via xacro
   │                                       └─ MoveGroupInterface on <planning_group>
   │
   ├─ TERMINAL_3  ───────────────────►   ros2 launch command_pump_<vendor> …
   │
   └─ TERMINAL_4  ───────────────────►   python wsl_ros_bridge.py   (FastAPI :8000)
                                            └─ reads back the `model` parameter from
                                               motion_server to build hardware service names

test.py --model xarm6
   └─ motion_http_client.py ── HTTP ──►  wsl_ros_bridge  ── ROS srv ──►  motion_server
```

**The model string is the join key of the whole system.** It is:

1. passed on the Windows CLI (`--model`),
2. forwarded to `motion_server.launch.py` as `model:=…`,
3. declared as a ROS parameter on the `motion_server` node,
4. re-read by `wsl_ros_bridge.py` (`_read_motion_server_param("model")`) to build
   `/<model>/SetServoOn`, `/<model>/pump/grab`, …,
5. used by `RobotHealthMonitor` to build `/<model>/RobotError`.

> ⚠️ Note the existing asymmetry: DENSO uses the bare model name (`vs060`), Stäubli uses a
> vendor-prefixed name (`staubli_tx40`) built in `launch_controller.py`
> (`model:=staubli_{MODEL}`). **For any new vendor, use a prefixed name**
> (`ufactory_xarm6`) — vendor detection everywhere is prefix-based.

---

## 2. The integration contract

`motion_control` is vendor-agnostic. A new robot is integrated correctly as soon as it
satisfies **all** of the following:

| # | Contract | Where it is consumed |
|---|---|---|
| C1 | A top-level **URDF xacro** accepting the args `model`, `sim`, `namespace`, `tool` | `motion_server.launch.py` → `robot_description` |
| C2 | A top-level **SRDF xacro** accepting the args `model`, `namespace`, `tool` | `motion_server.launch.py` → `robot_description_semantic` |
| C3 | A **MoveIt planning group** whose name has an entry in a `config/kinematics.yaml` | `merged_kinematics`, `MoveGroupInterface` |
| C4 | A **tool dispatcher** in the URDF that includes `$(find tool)/$(arg tool)/$(arg tool).xacro` and attaches the macro to the flange link | tool packages (`effecteur_v1..v3`) |
| C5 | The SRDF group tip is `tool_link` when `tool != none`, flange link otherwise | Cartesian moves, TCP poses |
| C6 | A running `move_group` node + a `joint_trajectory_controller` publishing `/joint_states` | `MoveGroupInterface`, `RobotHealthMonitor` watchdog |
| C7 | Pilz capabilities loaded (`MoveGroupSequenceAction` / `MoveGroupSequenceService`) | `planCartesianLin`, `/sequence_move_group` |

Anything else (vacuum pump, drive power, error topics) is **vendor-specific** and lives in
optional branches described in §7–§9.

---

## 3. Step 0 — Get the vendor packages

```bash
cd ~/workspace/ros2_ws/src
git clone -b humble https://github.com/xArm-Developer/xarm_ros2.git --recursive
cd ~/workspace/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

Inspect what you got — do not trust this document over the actual files:

```bash
# Link / joint names of the arm
xacro src/xarm_ros2/xarm_description/urdf/xarm_device.urdf.xacro dof:=6 robot_type:=xarm \
  | grep -E '<(link|joint) name'

# Planning groups offered by the vendor MoveIt config
grep -n '<group name' src/xarm_ros2/xarm_moveit_config/srdf/*.xacro
```

For an xArm 6 you should find joints `joint1…joint6`, links `link_base`, `link1…link6`,
and the flange link `link_eef`. **Write these three values down** — they are the only
vendor-specific inputs to the rest of the procedure:

| Symbol used below | xArm 6 value |
|---|---|
| `<BASE_LINK>` | `link_base` |
| `<FLANGE_LINK>` | `link_eef` |
| `<JOINTS>` | `joint1 … joint6` |
| `<GROUP>` | `xarm6` |

---

## 4. Step 1 — Choose a package layout

Two layouts exist in the workspace; pick the one that matches your need.

**A. Model-parameterised** (DENSO style) — one package set covers many models,
selected by the `model` arg through `robots/<model>/…` subfolders:

```
denso_robot_descriptions/
├── urdf/denso_robot.urdf.xacro          ← dispatcher, reads $(arg model)
└── robots/vs060/urdf/denso_robot_macro.xacro
```

**B. One package per model** (Stäubli style) — simpler, more duplication:

```
staubli_tx40_description/
staubli_tx40_moveit_config/
```

👉 **Recommended for UFACTORY: layout A**, because UFACTORY sells several arms sharing one
description (xArm5/6/7, Lite6, 850) that differ only by a `dof` xacro arg.

Create:

```
ros2_ws/src/ufactory_robot/
├── ufactory_descriptions/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── urdf/ufactory_robot.urdf.xacro           ← top level, contract C1 + C4
│   └── robots/xarm6/urdf/ufactory_robot_macro.xacro
├── ufactory_moveit_config/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/kinematics.yaml                   ← contract C3
│   ├── config/ompl_planning.yaml
│   ├── config/pilz_cartesian_limits.yaml
│   ├── config/pilz_industrial_motion_planner_planning.yaml
│   ├── robots/xarm6/config/joint_limits.yaml
│   ├── robots/xarm6/config/moveit_controllers.yaml
│   ├── robots/xarm6/config/ufactory_robot_controllers.yaml
│   └── srdf/ufactory_robot.srdf.xacro           ← top level, contract C2
└── ufactory_bringup/
    ├── CMakeLists.txt
    ├── package.xml
    └── launch/ufactory_robot_bringup.launch.py
```

Copy `CMakeLists.txt` / `package.xml` from `denso_robot_descriptions` and
`denso_robot_moveit_config` and rename — they only install shared folders.

---

## 5. Step 2 — Top-level URDF xacro (C1 + C4)

`ufactory_descriptions/urdf/ufactory_robot.urdf.xacro` — mirror of
`denso_robot_descriptions/urdf/denso_robot.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="$(arg model)">

  <xacro:arg name="model"     default="ufactory_xarm6"/>
  <xacro:arg name="namespace" default=""/>
  <xacro:arg name="sim"       default="true"/>
  <xacro:arg name="tool"      default="none"/>
  <xacro:arg name="ip_address" default="192.168.1.200"/>

  <!-- model dispatcher: ufactory_xarm6 -> robots/xarm6/... -->
  <xacro:property name="bare_model"
                  value="${'$(arg model)'.replace('ufactory_','')}"/>

  <xacro:include filename="$(find ufactory_descriptions)/robots/${bare_model}/urdf/ufactory_robot_macro.xacro"/>

  <xacro:ufactory_robot
    model="${bare_model}"
    namespace="$(arg namespace)"
    sim="$(arg sim)"
    ip_address="$(arg ip_address)"/>

  <!-- ── Tool dispatcher (contract C4) ────────────────────────────── -->
  <xacro:property name="tool_parent_link" value="$(arg namespace)link_eef"/>

  <xacro:unless value="${'$(arg tool)' == 'none'}">
    <xacro:include filename="$(find tool)/$(arg tool)/$(arg tool).xacro"/>
    <xacro:call macro="$(arg tool)"
                namespace="$(arg namespace)"
                parent_link="${tool_parent_link}"/>
  </xacro:unless>

</robot>
```

`robots/xarm6/urdf/ufactory_robot_macro.xacro` wraps the vendor macro and adds the
`ros2_control` block:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro">
  <xacro:include filename="$(find xarm_description)/urdf/xarm_device_macro.xacro"/>

  <xacro:macro name="ufactory_robot" params="model namespace sim ip_address">

    <xacro:xarm_device dof="6" robot_type="xarm" prefix="${namespace}"
                       ros2_control_plugin="${sim == 'true'
                           ? 'mock_components/GenericSystem'
                           : 'uf_robot_hardware/UFRobotSystemHardware'}"
                       robot_ip="${ip_address}"/>
  </xacro:macro>
</robot>
```

> The vendor macro already emits `<ros2_control>`; if you write your own hardware block,
> copy the structure of `denso_robot_descriptions/robots/vs060/urdf/denso_robot.ros2_control.xacro`.

**Verify before going further:**

```bash
xacro src/ufactory_robot/ufactory_descriptions/urdf/ufactory_robot.urdf.xacro \
      model:=ufactory_xarm6 sim:=true namespace:='' tool:=effecteur_v3 > /tmp/out.urdf
check_urdf /tmp/out.urdf
grep -n 'tool_link' /tmp/out.urdf     # must exist when tool != none
```

---

## 6. Step 3 — Top-level SRDF xacro (C2 + C3 + C5)

`ufactory_moveit_config/srdf/ufactory_robot.srdf.xacro`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="$(arg model)">
  <xacro:arg name="model"     default="ufactory_xarm6"/>
  <xacro:arg name="namespace" default=""/>
  <xacro:arg name="tool"      default="none"/>

  <xacro:property name="bare_model" value="${'$(arg model)'.replace('ufactory_','')}"/>

  <xacro:include filename="$(find ufactory_moveit_config)/robots/${bare_model}/srdf/ufactory_robot_macro.srdf.xacro"/>

  <xacro:ufactory_robot_srdf name="$(arg namespace)$(arg model)"
                             namespace="$(arg namespace)"
                             tool="$(arg tool)"/>
</robot>
```

`robots/xarm6/srdf/ufactory_robot_macro.srdf.xacro` — note the **tip-link switch**
(contract C5), copied from `staubli_tx40.srdf.xacro`:

```xml
<xacro:macro name="ufactory_robot_srdf" params="name namespace tool">

  <group name="xarm6">
    <xacro:if value="${tool == 'none'}">
      <chain base_link="${namespace}link_base" tip_link="${namespace}link_eef"/>
    </xacro:if>
    <xacro:unless value="${tool == 'none'}">
      <chain base_link="${namespace}link_base" tip_link="${namespace}tool_link"/>
    </xacro:unless>
  </group>

  <group_state name="allZeros" group="xarm6">
    <joint name="${namespace}joint1" value="0"/>
    <!-- … joint2 … joint6 … -->
  </group_state>

  <!-- Generated by the MoveIt Setup Assistant, or copied from xarm_moveit_config -->
  <disable_collisions link1="${namespace}link_base" link2="${namespace}link1" reason="Adjacent"/>
  <!-- … -->

  <xacro:unless value="${tool == 'none'}">
    <disable_collisions link1="${namespace}link6" link2="${namespace}tool_link"
                        reason="Attached by fixed joint"/>
  </xacro:unless>
</xacro:macro>
```

`ufactory_moveit_config/config/kinematics.yaml` (contract C3) — **the top-level key must
be exactly the planning group name**:

```yaml
xarm6:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.01
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 5
  position_only_ik: false
```

Copy `ompl_planning.yaml`, `pilz_cartesian_limits.yaml` and
`pilz_industrial_motion_planner_planning.yaml` from `denso_robot_moveit_config/config/`
and rename the group keys to `xarm6`.

> **Group-name collisions matter.** `motion_server.launch.py` merges the kinematics maps of
> every vendor into one dict. `arm` (DENSO) and `manipulator` (Stäubli) are already taken —
> using `xarm6` keeps them distinct. Never reuse an existing group name for a new robot.

---

## 7. Step 4 — Bringup launch (TERMINAL_1)

Create `ufactory_bringup/launch/ufactory_robot_bringup.launch.py`. The simplest correct
version follows the Stäubli pattern (`staubli_tx40_planning_execution_sim.launch.py`)
and uses `MoveItConfigsBuilder`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def _launch_setup(context):
    model = LaunchConfiguration("model").perform(context)
    tool  = LaunchConfiguration("tool").perform(context)
    sim   = LaunchConfiguration("sim").perform(context)

    moveit_config = (
        MoveItConfigsBuilder("ufactory_robot", package_name="ufactory_moveit_config")
        .robot_description(
            file_path="../ufactory_descriptions/urdf/ufactory_robot.urdf.xacro",
            mappings={"model": model, "tool": tool, "sim": sim, "namespace": ""},
        )
        .robot_description_semantic(
            file_path="srdf/ufactory_robot.srdf.xacro",
            mappings={"model": model, "tool": tool, "namespace": ""},
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path=f"robots/{model.replace('ufactory_','')}/config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config).entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="ufactory_xarm6"),
        DeclareLaunchArgument("tool",  default_value="none"),
        DeclareLaunchArgument("sim",   default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
```

If you need Gazebo (not just `mock_components`), copy
`denso_robot_bringup/launch/denso_robot_bringup.launch.py` instead — it already wires
`gazebo_ros2_control`, `spawn_entity` and the controller spawners.

**Pilz capabilities (C7)** must be enabled — either through
`.planning_pipelines(pipelines=[..., "pilz_industrial_motion_planner"])` as above, or by
passing on the CLI, as `launch_controller.py` does for Stäubli:

```
capabilities:='pilz_industrial_motion_planner/MoveGroupSequenceAction
               pilz_industrial_motion_planner/MoveGroupSequenceService'
```

---

## 8. Step 5 — Wire the robot into `motion_control` ⭐

This is **the single most important file**:
`ros2_ws/src/motion_control/launch/motion_server.launch.py`.

Today it branches with inline `PythonExpression` tests such as
`"'staubli_tx40_moveit_config' if 'tx40' in model else 'denso_robot_moveit_config'"`.
Adding a third vendor that way becomes unreadable. **Refactor to a registry** — this is
the recommended change:

```python
# --- Robot registry -------------------------------------------------------
# One entry per supported vendor. Matching is done on a prefix / substring of
# the `model` launch argument.
ROBOT_REGISTRY = {
    "staubli_": dict(
        description_package="staubli_tx40_moveit_config",
        description_folder="config",
        description_file="staubli_tx40.urdf.xacro",
        moveit_config_package="staubli_tx40_moveit_config",
        srdf_folder="config",
        moveit_config_file="staubli_tx40.srdf.xacro",
        planning_group="manipulator",
        kinematics_package="staubli_tx40_moveit_config",
        require_drives_powered=True,
        use_health_monitor_in_sim=False,
    ),
    "ufactory_": dict(
        description_package="ufactory_descriptions",
        description_folder="urdf",
        description_file="ufactory_robot.urdf.xacro",
        moveit_config_package="ufactory_moveit_config",
        srdf_folder="srdf",
        moveit_config_file="ufactory_robot.srdf.xacro",
        planning_group="xarm6",
        kinematics_package="ufactory_moveit_config",
        require_drives_powered=False,
        use_health_monitor_in_sim=False,
    ),
    "": dict(  # default = DENSO
        description_package="denso_robot_descriptions",
        description_folder="urdf",
        description_file="denso_robot.urdf.xacro",
        moveit_config_package="denso_robot_moveit_config",
        srdf_folder="srdf",
        moveit_config_file="denso_robot.srdf.xacro",
        planning_group="arm",
        kinematics_package="denso_robot_moveit_config",
        require_drives_powered=False,
        use_health_monitor_in_sim=True,
    ),
}


def resolve_robot(model: str) -> dict:
    for prefix, cfg in ROBOT_REGISTRY.items():
        if prefix and model.startswith(prefix):
            return cfg
    return ROBOT_REGISTRY[""]
```

Because the resolution now needs the *value* of `model`, wrap the body of
`generate_launch_description()` in an `OpaqueFunction` so you can call
`LaunchConfiguration("model").perform(context)` and use plain Python instead of
`PythonExpression`. The Stäubli sim launch shows the pattern.

Then merge **every** registry kinematics file, not just two:

```python
merged_kinematics = {}
for cfg in ROBOT_REGISTRY.values():
    merged_kinematics.update(load_yaml(cfg["kinematics_package"], "config/kinematics.yaml") or {})
```

And extend the per-group solver overrides that are passed to the node — one line per group:

```python
"robot_description_kinematics.arm.kinematics_solver":         kinematics_plugin_name,
"robot_description_kinematics.manipulator.kinematics_solver": kinematics_plugin_name,
"robot_description_kinematics.xarm6.kinematics_solver":       kinematics_plugin_name,   # NEW
```

Mirror that declaration in `motion_control/src/motion_server.cpp` (constructor, ~line 30):

```cpp
this->declare_parameter<std::string>(
    "robot_description_kinematics.xarm6.kinematics_solver",
    "kdl_kinematics_plugin/KDLKinematicsPlugin");
```

> **Minimal alternative** — if you do not want to refactor, just add `ufactory` cases to
> each of the seven existing `PythonExpression` blocks (`description_package`,
> `description_folder`, `description_file`, `moveit_config_package`, `srdf_folder`,
> `moveit_config_file`, `planning_group`) plus `merged_kinematics`,
> `use_health_monitor` and `require_drives_powered`. It works, but it does not scale.

---

## 9. Step 6 — Health monitor (optional, vendor-specific)

`motion_control/src/robot_health_monitor.cpp` currently knows two families:

* it always subscribes to `/<model>/RobotError` and `/<model>/RobotErrorDescription`
  (DENSO b-CAP topics — harmless no-ops for other vendors),
* it always watches `/joint_states` staleness (**generic, works for any robot**),
* if `model` starts with `staubli_`, it additionally requires
  `industrial_msgs/RobotStatus` on `/robot_status`,
* it relays `/rosout` FATAL messages coming from DENSO driver nodes.

For a first UFACTORY integration **do nothing here**: the joint-state watchdog alone gives
you E-Stop / comms-loss detection. Just make sure `use_health_monitor` is set correctly in
the registry.

If you later want xArm error reporting, add a `is_ufactory_` flag next to `is_staubli_` in
`robot_health_monitor.hpp/.cpp` and subscribe to the vendor's diagnostic topic
(`/xarm/robot_states` → `err`/`warn` fields), calling `triggerError()` on a non-zero code.

---

## 10. Step 7 — Vacuum pump / digital IO

Each vendor has its own IO transport, so each gets its own package:

| Vendor | Package | Transport |
|---|---|---|
| DENSO | `command_pump_denso` | GPIO pins via the RC8 (`pump_pin`, `valve_pin`, `vacuum_sensor_pin`) |
| Stäubli | `command_pump_staubli` | `/io_interface/write_single_io` service or direct socket on port 11003 |
| UFACTORY | `command_pump_ufactory` *(to create)* | `xarm_msgs/srv/SetInt16` on `/xarm/set_tgpio_digital` |

The **service names are the contract** — `wsl_ros_bridge.py` builds them from the model:

```
/<model>/pump/grab        std_srvs/SetBool
/<model>/pump/release     std_srvs/SetBool
/<model>/pump/is_grabbed  std_srvs/SetBool
```

So `command_pump_ufactory` must advertise `/ufactory_xarm6/pump/grab`, etc. Copy
`command_pump_staubli/command_pump_staubli/pump_controller_node.py` and replace the IO
backend; keep the node name, the `model` parameter and the three service names identical.

Add the launch file `command_pump_ufactory/launch/pump_controller.launch.py` following the
same argument style.

If the new robot has no pump at all, you can skip this package entirely — the pump clients
are only created when `sim == false`, and missing services only produce a warning.

---

## 11. Step 8 — HTTP bridge (`wsl_ros_bridge.py`)

Three places to touch, all vendor-detection by prefix:

**1. Motors on/off.** `call_set_servo_on()` currently does:

```python
if self.model.startswith("staubli_"):
    ...  # SetDrivePower on /system_interface/set_drive_power
else:
    ...  # SetServoOn on /<model>/SetServoOn   (DENSO)
```

Add an explicit UFACTORY branch calling `xarm_msgs/srv/SetInt16` on
`/xarm/motion_enable` + `/xarm/set_state`, instead of falling through to the DENSO path.

**2. Client creation in `init_robot()`** (~line 370):

```python
is_staubli  = model.startswith("staubli_")
is_ufactory = model.startswith("ufactory_")          # NEW

if not self.sim and is_ufactory and self.motion_enable_cli is None:
    self.motion_enable_cli = self.create_client(SetInt16, "/xarm/motion_enable")
```

Also guard the DENSO branch, which is currently `if not is_staubli` — it must become
`if not is_staubli and not is_ufactory`, otherwise the bridge will wait 10 s for a
non-existent `/ufactory_xarm6/SetServoOn` service.

**3. `robot_family` label** used in logs (~line 395) — cosmetic but keep it accurate.

> The top-of-file import `from denso_robot_core_interfaces.srv import SetServoOn` makes the
> bridge hard-depend on the DENSO packages. If you ever build a workspace without them,
> move these vendor imports into a `try/except ImportError` guard.

---

## 12. Step 9 — Windows side (`stageroswindows`)

**`launch_controller.py`**

```python
parser.add_argument("--model", choices=["vs060", "vp5243", "tx40", "xarm6"],   # + xarm6
                    default="vs060")
...
IS_STAUBLI  = MODEL in {"tx40"}
IS_UFACTORY = MODEL in {"xarm6", "lite6"}             # NEW
```

Add the branch that builds the four terminals:

```python
elif IS_UFACTORY:
    TERMINAL_1 = (
        f"{SETUP} && "
        "ros2 launch ufactory_bringup ufactory_robot_bringup.launch.py "
        f"model:=ufactory_{MODEL} sim:={SIM} tool:={TOOL}"
    )
    TERMINAL_2 = (
        f"{SETUP} && "
        "ros2 launch motion_control motion_server.launch.py "
        f"model:=ufactory_{MODEL} sim:={SIM} tool:={TOOL} "
        f"ik_solver:={SOLVER} accuracy:={ACCURACY}"
    )
    TERMINAL_3 = (
        f"{SETUP} && "
        "ros2 launch command_pump_ufactory pump_controller.launch.py "
        f"model:=ufactory_{MODEL}"
    )
```

Then extend the shutdown lists in `kill_wsl_processes()` — otherwise processes survive
`Ctrl+C` and the next launch fails on a busy port 11345:

* `launch_targets` → `"ros2 launch ufactory_bringup"`, `"ufactory_robot_bringup.launch.py"`,
  `"ros2 launch command_pump_ufactory"`
* `node_targets` → `"xarm"`, `"ufactory"`
* `launch_regex` → add `|ufactory_bringup|command_pump_ufactory`
* `verify_regex` → add `|ufactory_|xarm`

**`env/xarm6.json`** — the collision environment (virtual fence, boxes, meshes) is
**per-robot**, because the fence is expressed in the robot base frame. Copy `env/tx40.json`
and adjust `fence` to the xArm 6 reach (850 mm), then the boxes/meshes of your cell.
Schema is defined in `manage_env.py`.

**`test.py` / `EXAMPLE.py`**

```python
parser.add_argument("--model", choices=["vs060", "vp5243", "tx40", "xarm6"], default="vs060")
...
robot.init_robot(model=MODEL, ...)
...
robot.load_environnement(".../env/xarm6.json")     # add the branch
```

**`motion_http_client.py`** — check `go_home()` (~line 444): the home joint configuration is
model-dependent and currently hardcoded for `vp5243` vs everything else. Add an xArm 6
entry, otherwise `go_home()` will command a pose that may be outside its joint limits.

---

## 13. Step 10 — Build and validate

```bash
cd ~/workspace/ros2_ws
colcon build --symlink-install --packages-select \
  ufactory_descriptions ufactory_moveit_config ufactory_bringup command_pump_ufactory motion_control
source install/setup.bash
```

Validate incrementally — **never** start with the full stack.

Bring the components up **one at a time, in separate WSL terminals, each one added on top of
the previous ones** (keep the earlier terminals running). If a stage fails, fix it before
adding the next one: a failure is then necessarily caused by the component you just added.

### Stage 1 — Bringup alone

```bash
ros2 launch ufactory_bringup ufactory_robot_bringup.launch.py \
  model:=ufactory_xarm6 sim:=true tool:=effecteur_v3
```

RViz must show the arm with the tool attached, and `ros2 topic hz /joint_states` must
publish. Nothing else is running at this point.

### Stage 2 — Add the motion server

New terminal, bringup still running:

```bash
ros2 launch motion_control motion_server.launch.py \
  model:=ufactory_xarm6 sim:=true tool:=effecteur_v3 ik_solver:=kdl
```

No `Unknown planning group` in the log, and:

```bash
ros2 param get /motion_server model          # -> ufactory_xarm6
ros2 service list | grep -E 'init_robot|move_'
```

### Stage 3 — Add the HTTP bridge

New terminal:

```bash
cd ~/workspace && source venv/bin/activate && python wsl_ros_bridge.py
curl -X POST localhost:8000/init_robot -d '{}'      # -> {"success": true}
```

### Stage 4 — Add the pump control

New terminal (skip if the robot has no pump):

```bash
ros2 launch command_pump_ufactory pump_controller.launch.py model:=ufactory_xarm6
ros2 service list | grep pump      # -> /ufactory_xarm6/pump/{grab,release,is_grabbed}
```

### Stage 5 — Everything at once, from Windows

Close all the WSL terminals, then start the whole stack through the launcher and read every
tab looking for errors:

```powershell
python .\launch_controller.py --model xarm6 --show-terminals
```

Then check that `Ctrl+C` really cleans up:

```powershell
wsl pgrep -af 'xarm|ufactory|move_group'      # -> nothing left
```

### Stage 6 — Small test moves, in simulation

Still in simulation, send a few isolated commands (one joint at a time, then a short
Cartesian move, then the tool) rather than a full program. This is where wrong joint
directions, a wrong TCP or a badly placed fence show up.

### Stage 7 — Full test program, in simulation

Run the complete scenario (`test.py`) in simulation, with the collision environment loaded
from `env/xarm6.json`. Repeat it a few times: intermittent failures usually mean planning
timeouts or a fence that is too tight.

### Stage 8 — Real robot

Only once stage 7 passes reliably. Start at low velocity/acceleration scaling, keep the
E-Stop within reach, and verify the emergency path (E-Stop pressed → the health monitor
reports the fault and `move_group` stops) **before** running the full program.

```powershell
python .\launch_controller.py --model xarm6 --real-robot --ip <robot_ip> --show-terminals
```

---

## 14. Files touched — summary checklist

### `stageroslinux`

- [ ] `ros2_ws/src/xarm_ros2/` — vendor packages (git clone, or submodule)
- [ ] `ros2_ws/src/ufactory_robot/ufactory_descriptions/` — **new**
- [ ] `ros2_ws/src/ufactory_robot/ufactory_moveit_config/` — **new**
- [ ] `ros2_ws/src/ufactory_robot/ufactory_bringup/` — **new**
- [ ] `ros2_ws/src/command_pump_ufactory/` — **new** (only if there is a pump)
- [ ] `ros2_ws/src/motion_control/launch/motion_server.launch.py` — **modified** (registry) ⭐
- [ ] `ros2_ws/src/motion_control/src/motion_server.cpp` — modified (declare the new group param)
- [ ] `ros2_ws/src/motion_control/src/robot_health_monitor.cpp` — optional
- [ ] `wsl_ros_bridge.py` — modified (vendor branches: servo, pump, clients)

### `stageroswindows`

- [ ] `launch_controller.py` — modified (choices, terminals, kill lists) ⭐
- [ ] `env/xarm6.json` — **new**
- [ ] `test.py`, `EXAMPLE.py` — modified (choices, env path)
- [ ] `motion_http_client.py` — modified (`go_home` joint configuration)
- [ ] `readme.md` — document the new `--model` value

---

## 15. Common pitfalls

| Symptom | Cause |
|---|---|
| `Unknown planning group: arm` | `planning_group` not resolved for the new model in `motion_server.launch.py` |
| IK always fails, `No kinematics solver instantiated` | the group name is missing from `merged_kinematics`, or the key in `kinematics.yaml` does not match the SRDF group name exactly |
| Cartesian / `LIN` moves fail with "capability not available" | Pilz capabilities not loaded in the bringup (contract C7) |
| TCP poses are at the flange, not the suction cup | the SRDF tip is still the flange — the `tool != none` branch (C5) is missing |
| `tool_link` not found in URDF | tool dispatcher missing or `parent_link` points at a non-existent link |
| The bridge hangs 10 s at `init_robot` then warns about `SetServoOn` | the DENSO branch in `wsl_ros_bridge.py` was not excluded for the new vendor |
| Second launch fails, port 11345 busy | `kill_wsl_processes()` patterns not updated in `launch_controller.py` |
| Robot jumps at startup / joint limit violations | `joint_limits.yaml` copied from another robot without adjusting values |
| Fence clips the robot | `env/<model>.json` copied without adjusting to the new reach |

---

## 16. Reference implementations

| Question | Look at |
|---|---|
| Model-parameterised description | `denso_robot_descriptions/urdf/denso_robot.urdf.xacro` |
| Per-model description | `Staubli_ROS2-humble/staubli_tx40_moveit_config/config/staubli_tx40.urdf.xacro` |
| Tool dispatcher + SRDF tip switch | `staubli_tx40_moveit_config/config/staubli_tx40.srdf.xacro` |
| Tool macro to imitate | `ros2_ws/src/tool/effecteur_v3/effecteur_v3.xacro` |
| Full Gazebo bringup | `denso_robot_bringup/launch/denso_robot_bringup.launch.py` |
| Minimal MoveIt bringup | `staubli_tx40_moveit_config/launch/staubli_tx40_planning_execution_sim.launch.py` |
| Vendor-agnostic motion API | `motion_control/srv/*.srv` |
| Pump abstraction | `command_pump_staubli/command_pump_staubli/pump_controller_node.py` |
