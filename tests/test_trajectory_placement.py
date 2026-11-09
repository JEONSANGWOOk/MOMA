import copy
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
from seer_control.trajectory_placement import evaluate,search,validate_record,LinkRecording,placement_scope,apply_saved
from seer_control.door_feedback import SurfaceDoorFollower
from seer_control.geometry3d import transform,point


def fixture_trace():
    faces=[((.1,0,0),(1,0,0),(1,0,1)),((.1,0,0),(1,0,1),(.1,0,1))]
    door=dict(name='left:1',pivot=(0,0,0),sign=1,handle=(9,9,9),surface_faces=faces,hook_faces=faces,hook_open_only=True,roller_open_angle=True,roller_capture_gap=.04,roller_after_hook_release=True,roller_close=True)
    rows=[];far=(9,9,9)
    def rotated(p,a):return point(transform(rpy=(0,0,math.radians(a))),p)
    def add(h=far,r=far):
        rows.append(dict(t=len(rows)*.1,hook=h,roller=r,q=[0]*6,pose=dict(x=0,y=0,theta=0),confidence=.9))
    add((.7,0,.5))
    for i in range(1,8):add(rotated((.7,0,.5),i*5))
    add(r=rotated((.7,-.035,.5),35));add(r=rotated((.7,-.026,.5),35))
    add(r=rotated((.7,-.12,.5),35));add(r=rotated((.7,-.026,.5),35))
    for i in range(1,46):add(r=rotated((.7+i*.002,-.026,.5),35+2*i))
    add()
    add(r=rotated((.7,.06,.5),120));add(r=rotated((.7,.025,.5),120));add(r=rotated((.7,.013,.5),120))
    for i in range(1,62):add(r=rotated((.7+i*.001,.013,.5),120-2*i))
    add()
    return dict(format='moma-full-link-v1',complete=True,frames=rows,scope={},origin=[0]*6,roller_radius=.025),[door]


class TrajectoryFitTests(unittest.TestCase):
    def test_same_contact_model_verifies_hook_roller_and_close(self):
        data,doors=fixture_trace();template=SurfaceDoorFollower(doors)
        metrics=evaluate(template,data['frames'],[0]*6)
        self.assertTrue(metrics['ok'],metrics)
        self.assertEqual(template.angles['left:1'],0) # trials do not contaminate template
        self.assertEqual(metrics,evaluate(template,data['frames'],[0]*6))
        wrong=evaluate(template,data['frames'],[2,0,0,0,0,0])
        self.assertFalse(wrong['ok'])

    def test_missing_closing_is_not_a_successful_full_trace(self):
        data,doors=fixture_trace();data['frames']=data['frames'][:57]
        self.assertFalse(evaluate(SurfaceDoorFollower(doors),data['frames'],[0]*6)['ok'])

    def test_bounded_search_finds_virtual_sequence_without_writing_robot(self):
        data,doors=fixture_trace();result=search(data,doors,[0]*6)
        self.assertTrue(result['metrics']['ok'],result)
        self.assertLessEqual(math.hypot(*result['origin'][:2]),.25)
        self.assertLessEqual(abs(result['origin'][5]),math.radians(12))

    def test_failed_trace_gaps_and_low_confidence_block_fit(self):
        for kind in ('failed','gap','confidence','nan'):
            data,doors=fixture_trace()
            if kind=='failed':data['complete']=False
            if kind=='gap':data['frames'][5]['t']+=.7
            if kind=='confidence':data['frames'][5]['confidence']=.1
            if kind=='nan':data['frames'][5]['q'][0]=float('nan')
            with self.assertRaises(ValueError):search(data,doors,[0]*6)


class RecordingTests(unittest.TestCase):
    def app(self):
        app=NS(real=True,connected=True,last_state=0,map=NS(name='map'),host=NS(get=lambda:'robot'),current_robot_map='map',lidar_map_alignment=[0]*3,lidar_mount_overrides=[],spatial_assets={},log=Mock(),after=Mock(),_confidence_value=lambda state:.9,_spatial_save=Mock())
        view=NS(app=app,cad_asset=NS(name='cad',door_parts=[]),arm_asset=NS(name='arm',movable=lambda:[{'name':str(i)} for i in range(6)],link_transforms=lambda q,world:{'J6':world}),tool_asset=NS(name='tool'),scales={'cad':1,'tool':1},mount=[0]*6,cad_origin=[0]*6,reset_real_doors=Mock())
        app.world3d=view;app.current_state=lambda:dict(x=1.,y=2.,theta=0.,speed=0)
        app.runtime_arm_feedback=NS(joints=lambda *a,**kw:{str(i):i*.1 for i in range(6)},signal=lambda p:False,marker_observation=lambda:None)
        return app

    def test_recorder_uses_raw_q_and_map_pose_without_render_window(self):
        import time
        app=self.app();app.last_state=time.monotonic()
        r=LinkRecording(app)
        with patch('seer_control.trajectory_placement.tool_offsets',return_value=((0,0,0),(0,0,.1),.025)),patch('seer_control.trajectory_placement.door_definitions',return_value=[{}]):
            r.begin()
        self.assertEqual(r.data['frames'][0]['q'],[i*.1 for i in range(6)])
        self.assertEqual(r.data['frames'][0]['hook'],(1.,2.,0.));app.after.assert_called_once()
        r.active=False

    def test_failed_record_does_not_replace_completed_latest(self):
        import time
        app=self.app();r=LinkRecording(app);data,_=fixture_trace();r.data=data;r.started=time.monotonic();r.active=True
        with tempfile.TemporaryDirectory() as temp,patch('seer_control.app.USER_DIR',Path(temp)):
            folder=Path(temp)/'door_trajectory_sessions';folder.mkdir();path=folder/'latest.json';path.write_text('keep')
            r.finish(False);self.assertEqual(path.read_text(),'keep')
            self.assertEqual(len(list(folder.glob('*.json'))),2)

    def test_verified_fit_restores_cad_and_preserves_marker_relation(self):
        app=self.app();view=app.world3d
        reference=transform((1,2,3))
        view.trajectory_placement=dict(enabled=True,scope=placement_scope(app),origin=[.1,.2,0,0,0,.3],marker_reference=reference,metrics={'ok':True})
        self.assertTrue(apply_saved(app))
        self.assertEqual(view.cad_origin,[.1,.2,0,0,0,.3])
        self.assertEqual(view.marker_placement['reference'],reference)
        self.assertEqual(view.marker_placement['mode'],'trajectory_registered')
        app._spatial_save.assert_called_once()

    def test_saved_placement_rejects_wrong_context_and_live_link(self):
        app=self.app();app.world3d.trajectory_placement=dict(enabled=True,scope={})
        with self.assertRaises(ValueError):apply_saved(app)
        app._departure_link=NS(thread=NS(is_alive=lambda:True))
        with self.assertRaises(ValueError):apply_saved(app)
        app._spatial_save.assert_not_called()

if __name__=='__main__':unittest.main()
