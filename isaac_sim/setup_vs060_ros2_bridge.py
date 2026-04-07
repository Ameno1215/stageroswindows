import omni.graph.core as og
import omni.usd
from pxr import UsdPhysics


ROBOT_PRIM_PATH = "/World/vs060"
GRAPH_PATH = "/World/DensoActionGraph"
JOINT_STATES_TOPIC = "/joint_states"
JOINT_COMMAND_TOPIC = "/joint_command"
ON_PHYSICS_STEP_NODE_TYPES = (
    "isaacsim.core.nodes.OnPhysicsStep",
    "omni.isaac.core_nodes.OnPhysicsStep",
)


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

    last_error = None
    for on_physics_step_node in ON_PHYSICS_STEP_NODE_TYPES:
        delete_existing_graph()
        try:
            og.Controller.edit(
                {
                    "graph_path": GRAPH_PATH,
                    "evaluator_name": "execution",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                },
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPhysicsStep",         on_physics_step_node),
                        ("ReadSimTime",           "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        # ---- Clock publisher: makes /clock available so ROS nodes can use use_sim_time:=true ----
                        ("PublishClock",          "isaacsim.ros2.bridge.ROS2PublishClock"),
                        ("PublishJointState",     "isaacsim.ros2.bridge.ROS2PublishJointState"),
                        ("SubscribeJointState",   "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                        ("ArticulationController","isaacsim.core.nodes.IsaacArticulationController"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        # Physics step drives everything so ROS feedback follows the simulator state rate,
                        # not the potentially slower viewport/render refresh rate.
                        ("OnPhysicsStep.outputs:step",    "PublishClock.inputs:execIn"),
                        ("OnPhysicsStep.outputs:step",    "PublishJointState.inputs:execIn"),
                        ("OnPhysicsStep.outputs:step",    "SubscribeJointState.inputs:execIn"),
                        ("OnPhysicsStep.outputs:step",    "ArticulationController.inputs:execIn"),
                        # Sim time feeds both the clock publisher and the joint state publisher
                        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                        # Joint command loop
                        ("SubscribeJointState.outputs:jointNames",       "ArticulationController.inputs:jointNames"),
                        ("SubscribeJointState.outputs:positionCommand",  "ArticulationController.inputs:positionCommand"),
                        ("SubscribeJointState.outputs:velocityCommand",  "ArticulationController.inputs:velocityCommand"),
                        ("SubscribeJointState.outputs:effortCommand",    "ArticulationController.inputs:effortCommand"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("ArticulationController.inputs:robotPath", resolved_robot_path),
                        ("PublishJointState.inputs:targetPrim",     resolved_robot_path),
                        ("PublishJointState.inputs:topicName",      JOINT_STATES_TOPIC),
                        ("SubscribeJointState.inputs:topicName",    JOINT_COMMAND_TOPIC),
                    ],
                },
            )
            break
        except Exception as exc:
            last_error = exc
    else:
        raise RuntimeError(
            f"Failed to create Isaac action graph with any OnPhysicsStep node type: {last_error}"
        )

    print(f"Isaac ROS2 graph created on {GRAPH_PATH} for {resolved_robot_path}")
    print(f"  Publishing /clock              (sim time → ROS clock)")
    print(f"  Publishing {JOINT_STATES_TOPIC:<20s} (joint feedback)")
    print(f"  Subscribing {JOINT_COMMAND_TOPIC:<20s} (joint commands)")
    print()
    print("IMPORTANT: Launch the ROS2 stack with use_sim_time:=true for Isaac backend.")


setup_action_graph()
