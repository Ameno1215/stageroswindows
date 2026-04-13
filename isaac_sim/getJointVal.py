from pxr import UsdPhysics, PhysxSchema
import omni.usd

stage = omni.usd.get_context().get_stage()
joint_paths = [f"/World/vs060/joints/joint_{i}" for i in range(1, 7)]

HEADROOM = 2.0

for i in range(1, 7):
    prim = stage.GetPrimAtPath(f"/World/vs060/joints/joint_{i}")
    physx_joint = PhysxSchema.PhysxJointAPI(prim)
    current = physx_joint.GetMaxJointVelocityAttr().Get()
    new_val = current * HEADROOM
    physx_joint.GetMaxJointVelocityAttr().Set(new_val)
    print(f"joint_{i}: {current:.1f} -> {new_val:.1f} deg/s")
    
for jp in joint_paths:
    prim = stage.GetPrimAtPath(jp)
    if not prim.IsValid():
        print(f"{jp}: NOT FOUND")
        continue
    
    print(f"\n--- {jp} ---")
    
    rev = UsdPhysics.RevoluteJoint(prim)
    if rev:
        print(f"  lower={rev.GetLowerLimitAttr().Get()}, upper={rev.GetUpperLimitAttr().Get()}")
    
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if drive:
        print(f"  stiffness={drive.GetStiffnessAttr().Get()}")
        print(f"  damping={drive.GetDampingAttr().Get()}")
        print(f"  maxForce={drive.GetMaxForceAttr().Get()}")
    
    physx_joint = PhysxSchema.PhysxJointAPI(prim)
    if physx_joint:
        print(f"  maxJointVelocity={physx_joint.GetMaxJointVelocityAttr().Get()}")