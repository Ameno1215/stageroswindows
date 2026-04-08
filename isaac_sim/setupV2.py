# ── Run this inside Isaac Sim Script Editor after importing URDF via UI ──

import omni.usd
import omni.graph.core as og
import carb
from pxr import UsdPhysics, PhysxSchema

# ── Configuration ──
ROBOT_PRIM_PATH = "/World/vs060"
GRAPH_PATH = "/World/DensoActionGraph"
JOINT_STATES_TOPIC = "/joint_states"
JOINT_COMMAND_TOPIC = "/joint_command"
TARGET_STIFFNESS = 1000000000.0
TARGET_DAMPING = 50000000.0
TARGET_MAX_FORCE = 1e12
ON_PHYSICS_STEP_NODE_TYPES = (
    "isaacsim.core.nodes.OnPhysicsStep",
    "omni.isaac.core_nodes.OnPhysicsStep",
)


def configure_physics():
    stage = omni.usd.get_context().get_stage()

    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            prim.GetAttribute("physxScene:timeStepsPerSecond").Set(240)
            print(f"PhysicsScene {prim.GetPath()} → 240 steps/s")
            break

    # carb_settings = carb.settings.get_settings()
    # carb_settings.set("/persistent/simulation/minFrameRate", 240)
    # carb_settings.set("/app/renderer/vsync", False)
    # print("V-Sync disabled, min frame rate → 240")


def configure_drives():
    stage = omni.usd.get_context().get_stage()
    count = 0

    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            drive_api = UsdPhysics.DriveAPI.Get(prim, "angular")
            if not drive_api:
                drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")

            drive_api.GetStiffnessAttr().Set(TARGET_STIFFNESS)
            drive_api.GetDampingAttr().Set(TARGET_DAMPING)
            drive_api.GetMaxForceAttr().Set(TARGET_MAX_FORCE)

            print(f"  {prim.GetPath()}: stiffness={TARGET_STIFFNESS}, damping={TARGET_DAMPING}")
            count += 1

    print(f"Joint drives configured ({count} joints)")


def setup_action_graph():
    stage = omni.usd.get_context().get_stage()

    prim = stage.GetPrimAtPath(GRAPH_PATH)
    if prim and prim.IsValid():
        stage.RemovePrim(GRAPH_PATH)

    # Find ArticulationRoot under robot
    resolved = None
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            path_str = str(p.GetPath())
            if path_str.startswith(ROBOT_PRIM_PATH):
                resolved = path_str
                break

    if not resolved:
        # Fallback: use ROBOT_PRIM_PATH directly
        resolved = ROBOT_PRIM_PATH
        print(f"WARNING: No ArticulationRoot found, using {resolved}")
    else:
        print(f"ArticulationRoot at: {resolved}")

    last_error = None
    for on_physics_step_node in ON_PHYSICS_STEP_NODE_TYPES:
        prim = stage.GetPrimAtPath(GRAPH_PATH)
        if prim and prim.IsValid():
            stage.RemovePrim(GRAPH_PATH)

        try:
            og.Controller.edit(
                {
                    "graph_path": GRAPH_PATH,
                    "evaluator_name": "execution",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                },
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPhysicsStep",          on_physics_step_node),
                        ("ReadSimTime",            "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("PublishClock",           "isaacsim.ros2.bridge.ROS2PublishClock"),
                        ("PublishJointState",      "isaacsim.ros2.bridge.ROS2PublishJointState"),
                        ("SubscribeJointState",    "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                        ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPhysicsStep.outputs:step",    "PublishClock.inputs:execIn"),
                        ("OnPhysicsStep.outputs:step",    "PublishJointState.inputs:execIn"),
                        ("OnPhysicsStep.outputs:step",    "SubscribeJointState.inputs:execIn"),
                        ("OnPhysicsStep.outputs:step",    "ArticulationController.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                        ("SubscribeJointState.outputs:jointNames",      "ArticulationController.inputs:jointNames"),
                        ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                        ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                        ("SubscribeJointState.outputs:effortCommand",   "ArticulationController.inputs:effortCommand"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("ArticulationController.inputs:robotPath", resolved),
                        ("PublishJointState.inputs:targetPrim",     resolved),
                        ("PublishJointState.inputs:topicName",      JOINT_STATES_TOPIC),
                        ("SubscribeJointState.inputs:topicName",    JOINT_COMMAND_TOPIC),
                    ],
                },
            )
            print(f"Action graph created at {GRAPH_PATH}")
            break
        except Exception as exc:
            last_error = exc
    else:
        raise RuntimeError(f"Failed to create action graph: {last_error}")


# ── Run everything ──
print("=" * 50)
print("Configuring Isaac Sim for VS060 ROS2 bridge")
print("=" * 50)

configure_physics()
configure_drives()
setup_action_graph()

print("\n=== Done! ===")
print("  Physics: 240 Hz | V-Sync: off")
print(f"  Publishing {JOINT_STATES_TOPIC}")
print(f"  Subscribing {JOINT_COMMAND_TOPIC}")
print("  Click Play, then launch ROS2 with use_sim_time:=true")