import copy
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
from seer_control.offline_scene import ScenarioArchive,latest_real_map,validate_recording,start_replay,use_last_real_map

def recording(name='main2'):
    return {'format':'moma-offline-replay-v1','scenario':name,'frames':[
        {'t':0.,'q':[0.]*6,'pose':{'x':1.,'y':2.,'theta':0.},'io':{'2':True,'3':False,'4':True,'5':False}},
        {'t':1.,'q':[.1]*6,'pose':{'x':1.1,'y':2.,'theta':0.},'io':{'2':False,'3':False,'4':False,'5':False}}]}

class OfflineSceneTests(unittest.TestCase):
    def test_missing_record_is_explicit_not_example(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(ValueError):ScenarioArchive(d).load('main2')
    def test_record_validation_rejects_code_and_invalid_time(self):
        with self.assertRaises(ValueError):validate_recording({'code':'print(1)'})
        for mutation in ('time','nan','short'):
            data=recording()
            if mutation=='time':data['frames'][1]['t']=0.
            elif mutation=='nan':data['frames'][0]['q'][0]=float('nan')
            else:data['frames'][0]['q']=[]
            with self.assertRaises(ValueError):validate_recording(data)
    def test_failed_or_empty_record_never_overwrites_good_record(self):
        with tempfile.TemporaryDirectory() as d:
            archive=ScenarioArchive(d);archive.folder.mkdir(exist_ok=True)
            archive.path('main2').write_text(json.dumps(recording()))
            archive.begin('main2');archive.finish('main2',False)
            self.assertEqual(len(archive.load('main2')['frames']),2)
    def test_last_map_index_and_fallback_without_network(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);(root/'maps').mkdir();a=root/'maps/a.smap';a.write_text('{}')
            self.assertEqual(latest_real_map(root),a)
            (root/'last_real_map.json').write_text(json.dumps({'path':str(a)}))
            self.assertEqual(latest_real_map(root),a)
    def test_replay_local_joint_io_and_base_without_network(self):
        with tempfile.TemporaryDirectory() as d:
            archive=ScenarioArchive(d);archive.folder.mkdir(exist_ok=True);archive.path('main2').write_text(json.dumps(recording()))
            callbacks=[];clock=[0.]
            arm=NS(state='IDLE',kin=NS(clamp=lambda q:list(q)),q=[0.]*6,io={'DO':{}})
            app=NS(real=False,_scenario_archive=archive,map=NS(name='map'),arm_dev_sim=arm,sim=NS(route=[],state=NS()),world3d=NS(cad_doors_auto=Mock(),arm_follow=Mock(),real_doors_follow=Mock()),_aw_sync_main=Mock())
            app.after=lambda delay,fn:callbacks.append(fn)
            with patch('seer_control.offline_scene.time.monotonic',side_effect=lambda:clock[0]),patch('socket.create_connection',side_effect=AssertionError('network forbidden')):
                future=start_replay(app,'main2');callbacks.pop(0)()
                clock[0]=.5;callbacks.pop(0)()
                self.assertAlmostEqual(arm.q[0],.05)
                self.assertEqual(arm.io['DO'][2],1)
                clock[0]=1.;callbacks.pop(0)()
                self.assertEqual(future.result()['source'],'recorded-replay')
                self.assertEqual(arm.io['DO'][2],0)
                self.assertAlmostEqual(app.sim.state.x,1.1)
    def test_actual_smap_load_preserves_nodes_and_headings(self):
        source={'header':{'mapName':'saved-real'},'advancedPointList':[
            {'className':'LocationMark','instanceName':'LM1','pos':{'x':0,'y':0},'dir':0},
            {'className':'LocationMark','instanceName':'LM2','pos':{'x':1,'y':0},'dir':.5}], 'normalPosList':[{'x':0,'y':1}]}
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);(root/'maps').mkdir();(root/'maps/saved.smap').write_text(json.dumps(source))
            app=NS(real=False,task_running=False,sim=NS(route=[]),arm_dev_sim=NS(state='IDLE'),decision_journal=None,refresh_tasks=Mock(),refresh_nodes=Mock(),fit_map=Mock(),_spatial_sync_cad_lidar=Mock(),log=Mock())
            with patch('seer_control.app.USER_DIR',root),patch('socket.create_connection',side_effect=AssertionError('network forbidden')):use_last_real_map(app)
            self.assertEqual(set(app.map.nodes),{'LM1','LM2'})
            self.assertEqual(app.sim.state.last_node,'LM2')
            self.assertAlmostEqual(app.sim.state.theta,.5)
            self.assertIs(app.sim.map,app.map)

    def test_successful_real_scene_capture_saved_and_loaded(self):
        with tempfile.TemporaryDirectory() as d:
            archive=ScenarioArchive(d);clock=[0.]
            arm=NS(name='FR5',movable=lambda:[{'name':'J'+str(i)} for i in range(1,7)])
            app=NS(map=NS(name='real-map'),runtime_arm_feedback=NS(signal=lambda path:False))
            view=NS(app=app,arm_asset=arm,cad_asset=None,cad_origin=[0.]*6,mount=[0.]*6)
            with patch('seer_control.offline_scene.time.monotonic',side_effect=lambda:clock[0]):
                archive.begin('main3')
                for t in (0.,.1):
                    clock[0]=t;archive.capture(view,{'x':0.,'y':0.,'theta':0.},{'J'+str(i):t for i in range(1,7)})
                self.assertTrue(archive.finish('main3',True))
            data=archive.load('main3')
            self.assertEqual(len(data['frames']),2)
            self.assertEqual(data['metadata']['map_name'],'real-map')
            self.assertEqual(data['frames'][1]['q'],[.1]*6)
