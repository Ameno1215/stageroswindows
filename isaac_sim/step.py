import omni.kit.app
import omni.usd
import carb
from pxr import UsdPhysics, PhysxSchema

stage = omni.usd.get_context().get_stage()

# 1. Physics à 240 Hz
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.Scene):
        prim.GetAttribute("physxScene:timeStepsPerSecond").Set(240)
        print(f"PhysicsScene {prim.GetPath()} → 240 steps/s")
        break

# 3. Désactiver V-Sync
carb_settings = carb.settings.get_settings()
carb_settings.set("/app/renderer/vsync", False)
print("V-Sync disabled")

# 4. PhysicsScene Synchronous
physics_scene = stage.GetPrimAtPath("/physicsScene")
if physics_scene.IsValid():
    physx_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene)
    physx_api.GetEnableSceneQuerySupportAttr().Set(True)
    print("PhysicsScene set to Synchronous")
else:
    print("WARNING: /World/physicsScene not found — check the path")