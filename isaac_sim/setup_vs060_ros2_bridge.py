import omni.graph.core as og
import omni.usd
from pxr import UsdPhysics


ROBOT_PRIM_PATH = "/World/vs060"
GRAPH_PATH = "/World/DensoActionGraph"
JOINT_STATES_TOPIC = "/joint_states_raw"
JOINT_COMMAND_TOPIC = "/joint_command"


def delete_existing_graph():
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(GRAPH_PATH)
    if prim and prim.IsValid():
        stage.RemovePrim(GRAPH_PATH)


def find_articulation_candidates():
    stage = omni.usd.get_context().get_stage()
    candidates = []
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            candidates.append(str(prim.GetPath()))
    return candidates


def resolve_robot_prim_path():
    stage = omni.usd.get_context().get_stage()
    configured = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if configured and configured.IsValid() and configured.HasAPI(UsdPhysics.ArticulationRootAPI):
        return ROBOT_PRIM_PATH

    candidates = find_articulation_candidates()
    if not candidates:
        raise RuntimeError(
            "No articulation root found in the stage. Import the URDF first, then press Play once. "
            f"Configured path was {ROBOT_PRIM_PATH}."
        )

    configured_name = ROBOT_PRIM_PATH.split("/")[-1].lower()
    for candidate in candidates:
        if configured_name and configured_name in candidate.lower():
            print(f"Configured articulation not found, using closest match: {candidate}")
            return candidate

    print("Configured articulation not found. Available articulation roots:")
    for candidate in candidates:
        print(f"  - {candidate}")
    print(f"Using first articulation root: {candidates[0]}")
    return candidates[0]


def setup_action_graph():
    delete_existing_graph()
    resolved_robot_path = resolve_robot_prim_path()

    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ArticulationController.inputs:robotPath", resolved_robot_path),
                ("PublishJointState.inputs:targetPrim", resolved_robot_path),
                ("PublishJointState.inputs:topicName", JOINT_STATES_TOPIC),
                ("SubscribeJointState.inputs:topicName", JOINT_COMMAND_TOPIC),
            ],
        },
    )

    print(f"Isaac ROS2 graph created on {GRAPH_PATH} for {resolved_robot_path}")
    print(f"Publishing joint states on {JOINT_STATES_TOPIC}")
    print(f"Subscribing joint commands on {JOINT_COMMAND_TOPIC}")


setup_action_graph()
