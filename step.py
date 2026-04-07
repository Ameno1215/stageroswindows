import omni.kit.app
from pxr import UsdPhysics
import omni.usd

# 1. Set physics scene timestep
stage = omni.usd.get_context().get_stage()
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.Scene):
        prim.GetAttribute("physxScene:timeStepsPerSecond").Set(240)
        print(f"PhysicsScene {prim.GetPath()} → 240 steps/s")
        break

# 2. Align the simulation frame rate
settings = omni.kit.app.get_app().get_settings()
settings.set("/persistent/simulation/minFrameRate", 240)
print("Min Simulation Frame Rate → 240")