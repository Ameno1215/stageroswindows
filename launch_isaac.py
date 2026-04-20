import os
from isaacsim.simulation_app import SimulationApp
import numpy as np
simulation_app = SimulationApp({"headless": False})

import omni.usd
import omni.graph.core as og
import carb
from isaacsim.core.utils.extensions import enable_extension
from pxr import UsdPhysics, UsdGeom, UsdLux, Gf, UsdShade, PhysxSchema

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.importer.urdf")
enable_extension("omni.kit.window.script_editor")
from isaacsim.asset.importer.urdf import _urdf


# -- Configuration --
URDF_PATH = r"C:\Users\33648\Desktop\STAGE_ROS\isaac_sim\import_bundle\vs060_isaac.urdf"
ROBOT_PRIM_PATH = "/World/vs060"
GRAPH_PATH = "/World/DensoActionGraph"
JOINT_STATES_TOPIC = "/joint_states"
JOINT_COMMAND_TOPIC = "/joint_command"
TARGET_STIFFNESS = 30000000000.0
TARGET_DAMPING = 300000000.0
TARGET_MAX_FORCE = 1e16

def import_urdf():
    urdf_interface = _urdf.acquire_urdf_interface()
    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.fix_base = True
    import_config.make_default_prim = False
    import_config.distance_scale = 1.0

    asset_root = os.path.dirname(URDF_PATH)
    urdf_filename = os.path.basename(URDF_PATH)

    print(f"Importing URDF from {URDF_PATH}...")
    result = urdf_interface.parse_urdf(asset_root, urdf_filename, import_config)
    
    # Import the robot exactly at ROBOT_PRIM_PATH
    path = urdf_interface.import_robot(asset_root, urdf_filename, result, import_config, ROBOT_PRIM_PATH)
    print(f"URDF imported at {path}")

    # Log where the ArticulationRootAPI was applied
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            print(f"  ArticulationRoot found at: {prim.GetPath()}")
    
    return path


def configure_physics():
    stage = omni.usd.get_context().get_stage()

    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            prim.GetAttribute("physxScene:timeStepsPerSecond").Set(240)
            print(f"PhysicsScene {prim.GetPath()} -> 240 steps/s")
            break

    carb_settings = carb.settings.get_settings()
    carb_settings.set("/persistent/simulation/minFrameRate", 240)
    carb_settings.set("/app/renderer/vsync", False)
    print("V-Sync disabled, min frame rate -> 240")


def configure_drives():
    stage = omni.usd.get_context().get_stage()

    HEADROOM = 10.0

    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            drive_api = UsdPhysics.DriveAPI.Get(prim, "angular")
            if not drive_api:
                drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")

            drive_api.GetStiffnessAttr().Set(TARGET_STIFFNESS)
            drive_api.GetDampingAttr().Set(TARGET_DAMPING)
            drive_api.GetMaxForceAttr().Set(TARGET_MAX_FORCE)

            # Increase velocity limit for PD tracking headroom
            physx_joint = PhysxSchema.PhysxJointAPI(prim)
            if physx_joint:
                current_vel = physx_joint.GetMaxJointVelocityAttr().Get()
                new_vel = current_vel * HEADROOM
                physx_joint.GetMaxJointVelocityAttr().Set(new_vel)
                print(f"  {prim.GetPath()}: stiffness={TARGET_STIFFNESS}, damping={TARGET_DAMPING}, maxVel={current_vel:.1f}->{new_vel:.1f}")
            else:
                print(f"  {prim.GetPath()}: stiffness={TARGET_STIFFNESS}, damping={TARGET_DAMPING}")

    print("Joint drives configured")


def setup_action_graph():
    stage = omni.usd.get_context().get_stage()

    # Clean existing graph if it exists
    prim = stage.GetPrimAtPath(GRAPH_PATH)
    if prim and prim.IsValid():
        stage.RemovePrim(GRAPH_PATH)

    # ---------------------------------------------------------
    # Resolve the EXACT path that holds the ArticulationRootAPI
    # ---------------------------------------------------------
    resolved = None
    for p in stage.Traverse():
        # Look for the API
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            path_str = str(p.GetPath())
            # Ensure it belongs to our specific robot
            if path_str.startswith(ROBOT_PRIM_PATH):
                resolved = path_str
                print(f"Using articulation root: {resolved}")
                break
                
    if not resolved:
        raise RuntimeError(f"No articulation root found under {ROBOT_PRIM_PATH}. Check your URDF import.")

    try:
        # Create Action Graph
        og.Controller.edit(
            {
                "graph_path": GRAPH_PATH,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPhysicsStep",          "isaacsim.core.nodes.OnPhysicsStep"),
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
                    # Feed the exact ArticulationRoot path to the controller and publisher
                    ("ArticulationController.inputs:robotPath", resolved),
                    ("PublishJointState.inputs:targetPrim",     resolved),
                    ("PublishJointState.inputs:topicName",      JOINT_STATES_TOPIC),
                    ("SubscribeJointState.inputs:topicName",    JOINT_COMMAND_TOPIC),
                ],
            },
        )
        print(f"Action graph created at {GRAPH_PATH}")
    except Exception as exc:
        raise RuntimeError(f"Failed to create action graph: {exc}")

def main():
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    # Dome light (ambient)
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.GetIntensityAttr().Set(1000)

    # Distant light (directional, like sun)
    distant = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    distant.GetIntensityAttr().Set(3000)
    distant.GetAngleAttr().Set(0.53)

    # Create /World as default prim
    world_prim = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world_prim.GetPrim())

    # Create an Environment prim to strictly match the UI tree requested
    UsdGeom.Xform.Define(stage, "/World/Environment")

    # 1. Import URDF
    import_urdf()

    # 2. Start simulation world BEFORE configuring physics (forces the PhysicsScene creation)
    #    physics_prim_path ensures the scene is created at /World/physicsScene instead of root
    from isaacsim.core.api.world import World
    world = World(physics_dt=1.0 / 240.0, rendering_dt=1.0 / 60.0, physics_prim_path="/World/physicsScene")

    # Add the default ground plane
    world.scene.add_default_ground_plane()
    
    plane_prim = stage.GetPrimAtPath("/World/defaultGroundPlane")
    if plane_prim.IsValid():
        UsdShade.MaterialBindingAPI(plane_prim).UnbindAllBindings()
        
        UsdGeom.Gprim(plane_prim).CreateDisplayColorAttr().Set([Gf.Vec3f(0.2, 0.2, 0.2)])

    world.reset()

    # 3. Configure physics (240 Hz, V-Sync off) now that /World/physicsScene exists
    configure_physics()

    # 4. Configure joint drives
    configure_drives()

    # 5. Setup ROS2 action graph
    setup_action_graph()

    print("\n=== Isaac Sim ready ===")
    print("  Ctrl+C to stop\n")

    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()

if __name__ == "__main__":
    main()