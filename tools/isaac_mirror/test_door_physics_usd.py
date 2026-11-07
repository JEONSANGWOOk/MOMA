"""Run with system Python 3.12 + installed Isaac USD libraries; no robot/Kit launch."""
import importlib.util
import json
import os
from pathlib import Path
import sys

install=Path(sys.argv[1])
libs=next((install/'extscache').glob('omni.usd.libs-*'))
schema=next((install/'extscache').glob('omni.usd.schema.physx-*'))
sys.path.insert(0,str(libs));handles=[os.add_dll_directory(str(libs/'bin')),os.add_dll_directory(str(install/'kit'))]
import pxr
pxr.__path__.append(str(schema/'pxr'))
if (schema/'bin').exists():handles.append(os.add_dll_directory(str(schema/'bin')))
from pxr import Plug
for info in (schema/'plugins').rglob('plugInfo.json'):Plug.Registry().RegisterPlugins(str(info))
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf
spec=importlib.util.spec_from_file_location('door_physics',Path(__file__).with_name('isaac_door_physics.py'))
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
parameters=json.loads((Path(__file__).resolve().parents[2]/'config/isaac_door_physics.json').read_text())
stage=Usd.Stage.CreateInMemory();root='/World/MOMA_Mirror';UsdGeom.Xform.Define(stage,root)
rows={};ops={}
def matrix(rows):return Gf.Matrix4d(*[rows[j][i] for i in range(4) for j in range(4)])
identity=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
for name,role in [('left','door'),('right','door'),('tool','tool'),('body','robot'),('cad','equipment')]:
    xf=UsdGeom.Xform.Define(stage,root+'/'+name);ops[name]=xf.AddTransformOp();ops[name].Set(Gf.Matrix4d(1))
    mesh=UsdGeom.Mesh.Define(stage,root+'/'+name+'/Geometry')
    mesh.CreatePointsAttr([Gf.Vec3f(0,0,0),Gf.Vec3f(.2,0,0),Gf.Vec3f(0,0,.5)])
    mesh.CreateFaceVertexCountsAttr([3]);mesh.CreateFaceVertexIndicesAttr([0,1,2])
    row=dict(id=name,matrix=identity,physics_role=role)
    if role=='door':row['door']=dict(side=name,sign=-1 if name=='left' else 1,hinge_world=identity,body_world=identity)
    rows[name]=row
rig=module.DoorPhysics(stage,root,parameters,matrix)
assert rig.setup(rows,ops)
for side in ('left','right'):
    path=root+'/Physics/Doors/'+side
    assert UsdPhysics.MassAPI(stage.GetPrimAtPath(path)).GetMassAttr().Get()==3.
    joint=UsdPhysics.RevoluteJoint.Get(stage,path+'/Hinge')
    assert joint.GetBody1Rel().GetTargets()[0].pathString==path
    assert abs(PhysxSchema.PhysxJointAPI(joint.GetPrim()).GetJointFrictionAttr().Get()-.05)<1e-6
    assert joint.GetLowerLimitAttr().Get()==(-120 if side=='left' else 0)
    assert joint.GetUpperLimitAttr().Get()==(0 if side=='left' else 120)
    assert UsdPhysics.MeshCollisionAPI(stage.GetPrimAtPath(path+'/'+side+'/Geometry')).GetApproximationAttr().Get()=='convexDecomposition'
assert UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(root+'/tool')).GetKinematicEnabledAttr().Get()
assert UsdPhysics.MeshCollisionAPI(stage.GetPrimAtPath(root+'/tool/Geometry')).GetApproximationAttr().Get()=='none'
scene_api=PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath(root+'/Physics/Scene'))
assert scene_api.GetEnableCCDAttr().Get() is True
assert scene_api.GetSolverTypeAttr().Get()=='TGS'
for name in ('tool','body'):
    api=PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath(root+'/'+name))
    assert api.GetEnableSpeculativeCCDAttr().Get() is True
    assert api.GetEnableCCDAttr().Get() is False
for side in ('left','right'):
    api=PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath(root+'/Physics/Doors/'+side))
    assert api.GetEnableCCDAttr().Get() is True
    assert api.GetEnableSpeculativeCCDAttr().Get() is True
for side,angle in (('left',-35.),('right',50.)):
    rows[side]['door'].update(contact_assist=True,target_angle_deg=angle)
rig.update_contact_assist(rows)
for side,angle in (('left',-35.),('right',50.)):
    drive=UsdPhysics.DriveAPI(stage.GetPrimAtPath(root+'/Physics/Doors/'+side+'/Hinge'),'angular')
    assert drive.GetTargetPositionAttr().Get()==angle
    assert drive.GetMaxForceAttr().Get()==40.
rows['left']['door']['target_angle_deg']=-200.
rig.update_contact_assist(rows)
assert rig.drives['left'].GetTargetPositionAttr().Get()==-120.
rows['left']['door']['contact_assist']=False
rig.update_contact_assist(rows)
assert rig.drives['left'].GetMaxForceAttr().Get()==0.
for side in ('left','right'):
    pairs=UsdPhysics.FilteredPairsAPI(stage.GetPrimAtPath(root+'/Physics/Doors/'+side)).GetFilteredPairsRel().GetTargets()
    assert [str(p) for p in pairs]==[root+'/cad/Geometry']
    assert not any('/tool' in str(p) or '/body' in str(p) for p in pairs)
assert scene_api.GetEnableGPUDynamicsAttr().Get() is False
assert scene_api.GetBroadphaseTypeAttr().Get()=='MBP'
for name,other in (('tool','body'),('body','tool')):
    pairs=UsdPhysics.FilteredPairsAPI(stage.GetPrimAtPath(root+'/'+name)).GetFilteredPairsRel().GetTargets()
    assert str(root+'/'+other) in [str(p) for p in pairs]
    assert not any('/Doors/' in str(p) for p in pairs)
for side in ('left','right'):
    mesh=stage.GetPrimAtPath(root+'/Physics/Doors/'+side+'/'+side+'/Geometry')
    assert PhysxSchema.PhysxConvexDecompositionCollisionAPI(mesh).GetMaxConvexHullsAttr().Get()==32
    api=PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath(root+'/Physics/Doors/'+side))
    assert abs(api.GetMaxDepenetrationVelocityAttr().Get()-.2)<1e-6
rig.check_layout(rows)
invalid=dict(parameters,door_mass_kg=-1)
try:module.validate_parameters(invalid)
except ValueError:pass
else:raise AssertionError('Invalid mass was accepted')
print('PASS: both USD door bodies, hinges, mass, friction, signed limits, concave tool collider; signed hook/roller assist targets, limit clamp, assist disable; no physics stepping or robot commands')
