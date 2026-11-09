import importlib.util
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
from seer_control.geometry3d import transform,multiply
from seer_control.marker_placement import rigid,solve,scope,update,solve_anchor
from seer_control.runtime_feedback import RuntimeArmFeedback

ROOT=Path(__file__).resolve().parents[1]

class MarkerPlacementTests(unittest.TestCase):
    def test_delta_is_left_multiplied_and_does_not_accumulate(self):
        ref=transform((1,2,.7),(0,0,.3));delta=transform((.04,-.02,0),(0,0,.1))
        measured=multiply(delta,ref);origin=[2,3,.2,0,0,.4]
        result=solve(ref,measured,origin)
        expected=multiply(delta,transform(origin[:3],origin[3:]))
        self.assertAlmostEqual(result[0],expected[0][3]);self.assertAlmostEqual(result[1],expected[1][3])
        self.assertAlmostEqual(result[5],.5);self.assertEqual(result[2],.2)
        self.assertEqual(result,solve(ref,measured,origin))

    def test_reject_scale_reflection_nan_and_tilt(self):
        for matrix in (transform((float('nan'),0,0)),((2,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1)),((-1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))):
            with self.assertRaises(ValueError):rigid(matrix)
        with self.assertRaises(ValueError):solve(transform(),transform(rpy=(.5,0,0)),[0]*6)

    def test_runtime_requires_fresh_valid_marker(self):
        feedback=RuntimeArmFeedback('http://localhost:8000')
        feedback._sample=(time.monotonic(),())
        feedback._payload={'robot':{'equipment_marker':{'valid':True,'age_s':.1}}}
        self.assertIsNotNone(feedback.marker_observation())
        feedback._payload['robot']['equipment_marker']['age_s']=3
        self.assertIsNone(feedback.marker_observation())
        feedback._sample=None;self.assertIsNone(feedback.marker_observation())

    def fixture(self):
        runtime=NS(base='http://robot',marker_observation=lambda:dict(valid=True,age_s=.1,capture_id='new',marker_id=0,frame='rbase_m',matrix=transform((.3,0,.5))))
        app=NS(real=True,connected=True,last_state=time.monotonic(),runtime_arm_feedback=runtime,host=NS(get=lambda:'robot'),current_robot_map='map',map=NS(name='map'),lidar_map_alignment=[0,0,0],lidar_mount_overrides=[],spatial_assets={},_spatial_save=Mock(),log=Mock())
        view=NS(cad_asset=NS(name='cell',faces=[]),scales={'cad':1},mount=[0]*6,cad_origin=[1,0,0,0,0,0],reset_real_doors=Mock())
        view.marker_placement=dict(enabled=True,scope=scope(app,view),reference=transform((.3,0,.5)),origin=[1,0,0,0,0,0])
        return app,view

    def test_base_rotation_propagates_and_token_only_applies_once(self):
        app,view=self.fixture();state=dict(x=.02,y=0,theta=.1,speed=0)
        update(app,view,state)
        self.assertAlmostEqual(view.cad_origin[0],.02+math.cos(.1));self.assertAlmostEqual(view.cad_origin[1],math.sin(.1))
        self.assertEqual(view.cad_placement_confirmation['source'],'real_marker_registered')
        update(app,view,dict(state,x=.08))
        self.assertEqual(app._spatial_save.call_count,1)

    def test_motion_scope_change_and_large_jump_hold_pose(self):
        for kind in ('motion','scope','jump'):
            app,view=self.fixture();state=dict(x=0,y=0,theta=0,speed=0)
            if kind=='motion':state['speed']=.1
            elif kind=='scope':app.current_robot_map='other'
            else:state['x']=2
            update(app,view,state)
            self.assertEqual(view.cad_origin,[1,0,0,0,0,0]);app._spatial_save.assert_not_called()

    def test_installer_changes_only_main4_and_is_idempotent(self):
        path=ROOT/'tools/runtime_viewer_patches/install_marker_bridge.py'
        spec=importlib.util.spec_from_file_location('installer',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)
            (folder/'scenario.py').write_text('def main4(moma):\n    marker = camera.target_pose(moma, exc.COORD.WORLD)\n    moma.SetLocal(0, marker)\ndef main5(moma):\n    pass\n',encoding='utf-8')
            (folder/'viewer_moma.py').write_text('def get_status():\n    return {"robot": {}}\n',encoding='utf-8')
            changes=module.prepare(folder)
            for p,text in changes:p.write_text(text,encoding='utf-8')
            self.assertIn('equipment_marker_bridge.capture',changes[0][1])
            self.assertEqual(changes,module.prepare(folder))

class MarkerAnchorTests(unittest.TestCase):
    @staticmethod
    def marker_frame(center):
        # Marker X across door, Y up, normal out of CAD front (-Y).
        return ((1,0,0,center[0]),(0,0,-1,center[1]),(0,1,0,center[2]),(0,0,0,1))

    def test_absolute_anchor_recovers_cad_without_previous_placement(self):
        center=(-.17,-.7,.9);cad=transform((2,3,.1),(0,0,.6))
        measured=multiply(cad,self.marker_frame(center))
        result=solve_anchor(measured,center,[999,-999,.1,0,0,-2])
        self.assertAlmostEqual(result[0],2);self.assertAlmostEqual(result[1],3);self.assertAlmostEqual(result[5],.6)
        self.assertAlmostEqual(result[2],.1)

    def test_scale_is_applied_before_translation(self):
        center=(-.1,-.5,.8);cad=transform((1,-2,0),(0,0,-1))
        measured=multiply(cad,self.marker_frame(tuple(2*v for v in center)))
        result=solve_anchor(measured,center,[0]*6,2)
        self.assertAlmostEqual(result[0],1);self.assertAlmostEqual(result[1],-2)

    def test_ambiguous_axes_do_not_silently_place_equipment(self):
        with self.assertRaises(ValueError):solve_anchor(transform(),[0,0,0],[0]*6)

    def test_low_localization_confidence_prevents_caching_marker(self):
        app,view=MarkerPlacementTests().fixture()
        app._confidence_value=lambda state:.1058
        view.marker_placement['mode']='marker_anchor'
        update(app,view,dict(x=0,y=0,theta=0,speed=0))
        self.assertFalse(hasattr(view,'_marker_observation'))
        app._spatial_save.assert_not_called()
        self.assertIn('신뢰도',view.marker_placement_status)

class MarkerJumpTests(unittest.TestCase):
    def test_first_hook_measurement_cannot_swing_equipment(self):
        from seer_control.door_teaching import reference
        app,view=MarkerPlacementTests().fixture();app._confidence_value=lambda state:.9
        face=((-.1,0,0),(.1,0,0),(.1,0,1))
        view.cad_asset.door_parts=[{'name':'left:1','asset':NS(faces=[face],face_materials=[36])}]
        view.marker_placement.update(mode='hook_teaching',teaching_sha256=reference()['source_sha256'])
        basis=((1,0,0,10),(0,0,1,10),(0,-1,0,1),(0,0,0,1))
        app.runtime_arm_feedback.marker_observation=lambda:dict(valid=True,age_s=0.,capture_id='bad-first',marker_id=0,frame='rbase_m',matrix=transform(),map_frame='controller_world_m',map_matrix=basis)
        update(app,view,dict(x=0,y=0,theta=0,speed=0))
        self.assertEqual(view.cad_origin,[1,0,0,0,0,0]);app._spatial_save.assert_not_called()
        self.assertIn('급변',view.marker_placement_status)

class BridgeTests(unittest.TestCase):
    def test_controller_bridge_converts_world_to_arm_base_and_mm_to_m(self):
        import sys
        from unittest.mock import patch
        from seer_control.arm_physics import inverse
        path=ROOT/'tools/runtime_viewer_patches/equipment_marker_bridge.py'
        spec=importlib.util.spec_from_file_location('bridge',path);bridge=importlib.util.module_from_spec(spec);spec.loader.exec_module(bridge)
        class Matrix:
            def __init__(self,m):self.m=transform(m) if len(m)==3 else m
            def __mul__(self,other):return Matrix(multiply(self.m,other.m))
            def inverse(self):return Matrix(inverse(self.m))
            def xyz(self):return NS(x=self.m[0][3],y=self.m[1][3],z=self.m[2][3])
        sdk=NS(COORD=NS(RBASE='arm',WORLD='world'),Matrix=Matrix,Vector3=lambda *v:v)
        tcp=Matrix(transform((100,200,300)))
        world_from_arm=Matrix(transform((1000,2000,0),(0,0,.4)))
        marker=Matrix(transform((300,400,500),(0,0,.2)))
        moma=NS(GetTCP=lambda coord:tcp if coord=='arm' else world_from_arm*tcp)
        with patch.dict(sys.modules,{'sdc_extctrlpy':sdk}):bridge.capture(moma,world_from_arm*marker)
        data=bridge.status();self.assertTrue(data['valid'])
        self.assertAlmostEqual(data['matrix'][0][3],.3);self.assertAlmostEqual(data['matrix'][1][3],.4)
        self.assertAlmostEqual(data['matrix'][0][0],math.cos(.2));self.assertEqual(data['frame'],'rbase_m')
        self.assertLess(data['age_s'],1)

if __name__=='__main__':unittest.main()
