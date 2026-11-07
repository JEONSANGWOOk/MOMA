"""Experimental PhysX doors driven only by kinematic robot contact."""
import math
from pxr import Gf,UsdGeom,UsdPhysics,UsdShade,PhysxSchema,Sdf


def validate_parameters(data):
    required=('door_mass_kg','static_friction','dynamic_friction','restitution','hinge_friction_coefficient',
              'angular_damping_per_s','opening_limit_deg','contact_offset_m','physics_hz')
    for key in required:
        if type(data.get(key)) not in (int,float) or not math.isfinite(data[key]):raise ValueError('Invalid physics parameter: '+key)
    if not .1<=data['door_mass_kg']<=100:raise ValueError('Door mass must be 0.1..100kg')
    if not 0<=data['dynamic_friction']<=data['static_friction']<=2:raise ValueError('Require 0 <= dynamic friction <= static friction <= 2')
    if not 0<=data['restitution']<=1 or not 0<=data['hinge_friction_coefficient']<=1:raise ValueError('Invalid restitution/hinge friction')
    if not 0<=data['angular_damping_per_s']<=100 or not 1<=data['opening_limit_deg']<=170:raise ValueError('Invalid damping/angle')
    if not .0001<=data['contact_offset_m']<=.02 or not 60<=data['physics_hz']<=240:raise ValueError('Invalid contact offset/physics rate')
    for key,default in (('contact_assist_stiffness',120.),('contact_assist_damping',20.),('contact_assist_max_torque_nm',40.)):
        value=data.get(key,default)
        if type(value) not in (int,float) or not math.isfinite(value) or not 0<=value<=1000:
            raise ValueError('Invalid contact assist parameter: '+key)
    if type(data.get('contact_assist_enabled',True)) is not bool:raise ValueError('Invalid contact assist switch')
    return data


class DoorPhysics:
    def __init__(self,stage,root,parameters,matrix):
        self.stage=stage;self.root=root;self.p=validate_parameters(parameters);self.matrix=matrix
        self.door_ids=set();self.anchors={};self.drives={};self.assist_started=set();self.initialized=False
    def material(self):
        mat=UsdShade.Material.Define(self.stage,self.root+'/Physics/ContactMaterial')
        api=UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr(self.p['static_friction']);api.CreateDynamicFrictionAttr(self.p['dynamic_friction'])
        api.CreateRestitutionAttr(self.p['restitution'])
        return mat
    def collision(self,prim,material,approximation):
        collision_api=UsdPhysics.CollisionAPI.Apply(prim)
        collision_api.CreateCollisionEnabledAttr(True)
        collision_api.CreateSimulationOwnerRel().SetTargets([Sdf.Path(self.root+'/Physics/Scene')])
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approximation)
        if approximation=='convexDecomposition':
            decomposition=PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
            decomposition.CreateMaxConvexHullsAttr(32)
            decomposition.CreateHullVertexLimitAttr(32)
        collision=PhysxSchema.PhysxCollisionAPI.Apply(prim)
        collision.CreateContactOffsetAttr(self.p['contact_offset_m']);collision.CreateRestOffsetAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material,UsdShade.Tokens.weakerThanDescendants,'physics')
    def setup(self,instances,transforms):
        door_rows=[r for r in instances.values() if r.get('physics_role')=='door']
        if not door_rows:return False
        # An explicit PhysicsScene is local to this mirror. No articulation drives on doors.
        scene=UsdPhysics.Scene.Define(self.stage,self.root+'/Physics/Scene')
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));scene.CreateGravityMagnitudeAttr(9.81)
        scene_api=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        scene_api.CreateTimeStepsPerSecondAttr(int(self.p['physics_hz']))
        # High-detail CAD triangle meshes can exhaust GPU contact-stack memory.
        # Use CPU PhysX for this small mechanism; RTX rendering remains on GPU.
        scene_api.CreateEnableGPUDynamicsAttr(False)
        scene_api.CreateBroadphaseTypeAttr('MBP')
        scene_api.CreateEnableCCDAttr(True)
        scene_api.CreateSolverTypeAttr('TGS')
        material=self.material()
        for side in ('left','right'):
            rows=[r for r in door_rows if r['door']['side']==side]
            if not rows:continue
            info=rows[0]['door'];body_matrix=self.matrix(info['body_world']);hinge=self.matrix(info['hinge_world'])
            if any(abs(v-1)>1e-5 for v in Gf.Transform(body_matrix).GetScale()):
                raise ValueError('Physics mode requires CAD scale=1; bake model scale first.')
            self.anchors[side]=hinge
            body_path=self.root+'/Physics/Doors/'+side
            body=UsdGeom.Xform.Define(self.stage,body_path);body.AddTransformOp().Set(body_matrix)
            rb=UsdPhysics.RigidBodyAPI.Apply(body.GetPrim());rb.CreateRigidBodyEnabledAttr(True);rb.CreateKinematicEnabledAttr(False)
            rb.CreateSimulationOwnerRel().SetTargets([scene.GetPath()])
            UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(self.p['door_mass_kg'])
            phys=PhysxSchema.PhysxRigidBodyAPI.Apply(body.GetPrim())
            phys.CreateAngularDampingAttr(self.p['angular_damping_per_s'])
            phys.CreateMaxDepenetrationVelocityAttr(.2)
            phys.CreateMaxAngularVelocityAttr(180.)
            phys.CreateSolverPositionIterationCountAttr(32);phys.CreateSolverVelocityIterationCountAttr(8)
            phys.CreateEnableCCDAttr(True)
            phys.CreateEnableSpeculativeCCDAttr(True)
            for row in rows:
                name=row['id'];self.door_ids.add(name)
                part=UsdGeom.Xform.Define(self.stage,body_path+'/'+name)
                part.AddTransformOp().Set(self.matrix(row['matrix'])*body_matrix.GetInverse())
                mesh_path=body_path+'/'+name+'/Geometry'
                copy=self.stage.DefinePrim(mesh_path)
                copy.GetReferences().AddInternalReference(self.root+'/'+name+'/Geometry')
                self.collision(copy,material,'convexDecomposition')
                UsdGeom.Imageable(transforms[name].GetAttr().GetPrim()).MakeInvisible()
            joint=UsdPhysics.RevoluteJoint.Define(self.stage,body_path+'/Hinge')
            joint.CreateBody1Rel().SetTargets([Sdf.Path(body_path)])
            joint.CreateAxisAttr('Z');joint.CreateLocalPos0Attr(Gf.Vec3f(*hinge.ExtractTranslation()))
            q=Gf.Transform(hinge).GetRotation().GetQuat()
            joint.CreateLocalRot0Attr(Gf.Quatf(q.GetReal(),Gf.Vec3f(*q.GetImaginary())))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0));joint.CreateLocalRot1Attr(Gf.Quatf(1))
            joint.CreateLowerLimitAttr(0. if info['sign']>0 else -self.p['opening_limit_deg'])
            joint.CreateUpperLimitAttr(self.p['opening_limit_deg'] if info['sign']>0 else 0.)
            PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim()).CreateJointFrictionAttr(self.p['hinge_friction_coefficient'])
        for name,row in instances.items():
            if name in self.door_ids:continue
            root=transforms[name].GetAttr().GetPrim();mesh=self.stage.GetPrimAtPath(self.root+'/'+name+'/Geometry')
            role=row.get('physics_role')
            if role in ('tool','robot'):
                body=UsdPhysics.RigidBodyAPI.Apply(root);body.CreateKinematicEnabledAttr(True)
                body.CreateSimulationOwnerRel().SetTargets([scene.GetPath()])
                phys=PhysxSchema.PhysxRigidBodyAPI.Apply(root)
                # Sweep CCD does not work on kinematic bodies. Speculative CCD
                # covers the moving hook/roller, including angular sweeps.
                phys.CreateEnableCCDAttr(False)
                phys.CreateEnableSpeculativeCCDAttr(True)
                # Concave hook geometry must not be replaced by a convex solid.
                self.collision(mesh,material,'none' if role=='tool' else 'convexHull')
            elif role=='equipment':self.collision(mesh,material,'none')
        # The CAD frame and moving leaf meet/overlap at their assembled edges.
        # Exclude only these assembly pairs, retaining robot/tool-door contact.
        fixed=[Sdf.Path(self.root+'/'+name+'/Geometry') for name,row in instances.items()
               if row.get('physics_role')=='equipment']
        for side in self.anchors:
            prim=self.stage.GetPrimAtPath(self.root+'/Physics/Doors/'+side)
            UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(fixed)
        # Each CAD material group is a separate kinematic rigid body. They
        # overlap within the same assembly and must not collide with each other.
        moving=[self.stage.GetPrimAtPath(self.root+'/'+name) for name,row in instances.items()
                if row.get('physics_role') in ('tool','robot')]
        for prim in moving:
            others=[other.GetPath() for other in moving if other!=prim]
            UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(others)
        self.initialized=True
        self.stage.GetPrimAtPath(self.root).SetCustomDataByKey('doorMode','PHYSICS - estimated parameters')
        print('[MOMA physics] Both doors use the same estimated mass/friction. Simulation starts according to auto_play. No real robot commands.')
        self.update_contact_assist(instances)
        return True
    def update_contact_assist(self,instances):
        """Drive the hinge using desktop hook/roller contact estimates, not force sensing.

        The door remains a dynamic body. This hybrid assist bridges intermittent
        triangle contacts while preserving the real robot's mirrored pose.
        """
        seen=set()
        for row in instances.values():
            if row.get('physics_role')!='door':continue
            info=row['door'];side=info['side']
            if side in seen:continue
            seen.add(side)
            joint=self.stage.GetPrimAtPath(self.root+'/Physics/Doors/'+side+'/Hinge')
            if not joint:continue
            drive=self.drives.get(side)
            if drive is None:
                drive=UsdPhysics.DriveAPI.Apply(joint,'angular');self.drives[side]=drive
                drive.CreateTypeAttr('force')
                drive.CreateStiffnessAttr(float(self.p.get('contact_assist_stiffness',120.)))
                drive.CreateDampingAttr(float(self.p.get('contact_assist_damping',20.)))
                drive.CreateMaxForceAttr(0.)
            angle=info.get('target_angle_deg')
            enabled=(self.p.get('contact_assist_enabled',True) is True and info.get('contact_assist') is True
                     and type(angle) in (float,int) and math.isfinite(angle))
            if enabled and abs(angle)>.05:self.assist_started.add(side)
            if enabled and side in self.assist_started:
                sign=info['sign'];limit=self.p['opening_limit_deg']
                target=max(0.,min(limit,angle*sign))*sign
                drive.CreateTargetPositionAttr(target)
                drive.CreateMaxForceAttr(float(self.p.get('contact_assist_max_torque_nm',40.)))
            else:drive.CreateMaxForceAttr(0.)
        self.stage.GetPrimAtPath(self.root).SetCustomDataByKey('doorMode','HYBRID contact physics + hook/roller angle assist (estimated)')

    def check_layout(self,instances):
        for row in instances.values():
            if row.get('physics_role')!='door':continue
            side=row['door']['side'];current=self.matrix(row['door']['hinge_world'])
            if side in self.anchors and any(abs(current[i][j]-self.anchors[side][i][j])>1e-5 for i in range(4) for j in range(4)):
                raise ValueError('Equipment layout changed. Stop and rebuild the physics scene.')
