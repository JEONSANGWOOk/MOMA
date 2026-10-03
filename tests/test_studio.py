import copy
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
from seer_control.model import MapModel, Simulator
from seer_control.smap import make_path_record, build_smap_from_model, load_smap_data
from seer_control.studio_core import (EditHistory, Recorder, MissionRunner, rigid_calibration,
                                    validate_model, flatten_actions, validate_actions, collision_reason)
from seer_control.studio_devices import substitute, validate_operation, tcp_operation, arm_call, DeviceBridge
from seer_control.studio_ui import ConsoleAdapter, StudioMixin


def map_fixture():
    m=MapModel(dict(format='amr-console-map-v1',name='M',nodes=[
        dict(id='A',x=0,y=0,kind='station'),dict(id='B',x=2,y=0,kind='station'),
        dict(id='CP',x=0,y=2,kind='dock')],edges=[['A','B'],['A','CP']],walls=[]))
    raw=make_path_record(m.nodes['A'],m.nodes['B'],properties={'maxspeed':.2,'maxacc':.3,'maxdec':.4})
    m.path_records=[dict(a='A',b='B',raw=raw,controls=[(2/3,0),(4/3,0)],properties={'maxspeed':.2,'maxacc':.3,'maxdec':.4}),
                    dict(a='B',b='A',raw=make_path_record(m.nodes['B'],m.nodes['A']),properties={}),
                    dict(a='A',b='CP',raw=make_path_record(m.nodes['A'],m.nodes['CP']),properties={}),
                    dict(a='CP',b='A',raw=make_path_record(m.nodes['CP'],m.nodes['A']),properties={})]
    return m


class DynamicsTests(unittest.TestCase):
    def test_speed_accel_decel_and_arrival_heading(self):
        m=map_fixture();m.nodes['B'].update(r=math.pi/2,spin=True)
        s=Simulator(m);s.navigate('B');speeds=[];headings=[]
        for _ in range(400):s.tick(.1);speeds.append(abs(s.state.speed));headings.append(s.state.theta)
        self.assertLessEqual(max(speeds),.2+1e-8)
        self.assertLessEqual(max(b-a for a,b in zip(speeds,speeds[1:])),.03+1e-8)
        self.assertEqual(s.state.task,'완료');self.assertAlmostEqual(s.state.theta,math.pi/2)
        self.assertTrue(any(0<h<math.pi/2 for h in headings))
    def test_obstacle_stop_and_resume(self):
        m=map_fixture();m.obstacles=[dict(x=1,y=0,radius=.2)];s=Simulator(m);s.navigate('B')
        for _ in range(200):s.tick(.1)
        self.assertTrue(s.state.blocked);self.assertLess(s.state.x,.6);self.assertEqual(s.state.speed,0)
        m.obstacles=[]
        for _ in range(200):s.tick(.1)
        self.assertFalse(s.state.blocked);self.assertEqual(s.state.last_node,'B')
    def test_forbidden_and_virtual_wall(self):
        m=map_fixture();m.virtual_walls=[[1,-1,1,1]]
        self.assertTrue(collision_reason(m,1,0,.23))
        m.virtual_walls=[];m.area_records=[dict(id='NO',points=[(.8,-1),(1.2,-1),(1.2,1),(.8,1)],properties={'forbidden':True})]
        s=Simulator(m);s.navigate('B')
        for _ in range(200):s.tick(.1)
        self.assertTrue(s.state.blocked);self.assertLess(s.state.x,.8)
    def test_area_speed_limit_and_storage(self):
        m=map_fixture();m.area_records=[dict(id='SLOW',points=[(-1,-1),(3,-1),(3,1),(-1,1)],properties={'maxspeed':.1})]
        m.virtual_walls=[[5,5,6,5]];m.obstacles=[dict(x=9,y=9,radius=.25)]
        m2=MapModel(copy.deepcopy(m.data()));s=Simulator(m2);s.navigate('B')
        for _ in range(60):s.tick(.1);self.assertLessEqual(abs(s.state.speed),.1+1e-8)
        self.assertEqual(m2.virtual_walls,m.virtual_walls);self.assertEqual(m2.area_records,m.area_records)
    def test_autocharge_returns_to_original_goal(self):
        m=map_fixture();s=Simulator(m);s.state.battery=10
        s.auto_charge.update(enabled=True,low=20,high=25,rate=10)
        s.navigate('B');phases=set()
        for _ in range(500):s.tick(.1);phases.add(s.auto_charge['phase'])
        self.assertIn('CHARGING',phases);self.assertEqual(s.state.last_node,'B')
        self.assertGreaterEqual(s.state.battery,24)
    def test_lidar_obstacle_and_wheel_model(self):
        m=map_fixture();m.obstacles=[dict(x=1,y=0,radius=.25)];s=Simulator(m)
        self.assertAlmostEqual(s.scan()[0][0],.75)
        m.obstacles=[];m.robot_model['wheel_scale']=2;s.drive(.1,0);s.tick(.1)
        self.assertAlmostEqual(s.state.x,.02)


class StudioCoreTests(unittest.TestCase):
    def test_history_preserves_smap_and_properties(self):
        m=map_fixture();m.smap_source={'header':{'mapName':'M'},'unknown':[1,2]}
        h=EditHistory();before=h.capture(m);m.nodes['A']['x']=.3
        h.commit(before,m,'move');h.undo(m);self.assertEqual(m.nodes['A']['x'],0)
        h.redo(m);self.assertEqual(m.nodes['A']['x'],.3);self.assertEqual(m.smap_source['unknown'],[1,2])
    def test_calibration_known_transform(self):
        pairs=[[0,0,2,3],[1,0,2,4],[0,1,1,3]]
        r=rigid_calibration(pairs);self.assertAlmostEqual(r['x'],2);self.assertAlmostEqual(r['y'],3)
        self.assertAlmostEqual(r['yaw'],math.pi/2);self.assertLess(r['rms'],1e-8)
        with self.assertRaises(ValueError):rigid_calibration([[0,0,1,1],[0,0,1,1]])
    def test_record_roundtrip_and_validation(self):
        r=Recorder(limit=2);r.recording=True
        for i in range(3):r.append(i,dict(x=i,y=0,theta=0))
        self.assertEqual(len(r.frames),2)
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'record.json';r.save(p);other=Recorder();other.load(p)
            self.assertEqual(other.frames,r.frames)
    def test_model_and_device_validation(self):
        with self.assertRaises(ValueError):validate_model({'radius':-1})
        with self.assertRaises(ValueError):validate_operation({'operations':{'set_do':{'verified':False}}},'set_do')
        self.assertEqual(substitute({'p':['${channel}','${value}']},{'channel':2,'value':False}),{'p':[2,False]})
    def test_flatten_respects_checked_flags(self):
        chain={'tasks':[{'checked':False,'groups':[{'actions':[{'type':'Wait'}]}]},
                        {'groups':[{'checked':False,'actions':[{'type':'Wait'}]},{'actions':[{'type':'Set DO'}]}]}]}
        self.assertEqual([a['type'] for a in flatten_actions(chain)],['Set DO'])
    def test_virtual_wall_smap_export(self):
        raw={'header':{'mapName':'M'},'advancedPointList':[
            {'className':'LandMark','instanceName':'A','pos':{'x':0,'y':0}},
            {'className':'LandMark','instanceName':'B','pos':{'x':2,'y':0}}],
            'advancedCurveList':[],'normalPosList':[]}
        m=load_smap_data(raw);m.virtual_walls=[[1,-1,1,1]]
        out=build_smap_from_model(m)
        self.assertTrue(out['advancedAreaList'][0]['instanceName'].startswith('StudioVirtualWall'))
        self.assertEqual(len(out['advancedAreaList'][0]['posGroup']),4)


class MissionTests(unittest.TestCase):
    def make_runner(self):
        m=map_fixture();s=Simulator(m)
        c=SimpleNamespace(sim=s,map=m,real=False,connected=True,control_enabled=False,sim_powered=True,
            studio_arm_safe=True,current_state=s.status,log=Mock(),studio_bridge=Mock(),
            studio_config={'arm':{}},studio_results={})
        c._studio_io_value=lambda kind,channel:getattr(s,kind)[channel]
        c.studio_runner=MissionRunner(ConsoleAdapter(c))
        return c,c.studio_runner
    def drive(self,c,r,n=100,now=0):
        for i in range(n):c.sim.tick(.1);r.tick(now+i*.1,.1)
    def test_all_sim_actions_and_arm_interlock(self):
        c,r=self.make_runner()
        r.start([{'type':'Set DO','channel':2,'value':True},{'type':'Wait DI Trigger','channel':3,'value':True},
                 {'type':'Translation','distance_m':.2,'speed_mps':.1},{'type':'Rotation','angle_deg':90,'speed_dps':30},
                 {'type':'Wait','duration_s':.2},{'type':'Arm Action','operation':'work','duration_s':.3},
                 {'type':'Arm Action','operation':'safe_pose','duration_s':.3},{'type':'Path Nav','goal':'B'}])
        self.drive(c,r,5);self.assertEqual(r.index,1);self.assertTrue(c.sim.do[2])
        c.sim.di[3]=True;self.drive(c,r,400,.5)
        self.assertEqual(r.status,'COMPLETED',r.error);self.assertEqual(c.sim.state.last_node,'B')
        self.assertTrue(c.studio_arm_safe)
    def test_arm_without_safe_pose_blocks_motion(self):
        c,r=self.make_runner();r.start([{'type':'Arm Action','operation':'work','duration_s':.1},{'type':'Path Nav','goal':'B'}])
        self.drive(c,r,20);self.assertEqual(r.status,'FAILED');self.assertIn('safe_pose',r.error)
    def test_timeout_does_not_repeat_device_action(self):
        adapter=Mock();adapter.poll.return_value=False;r=MissionRunner(adapter)
        r.start([{'type':'Wait DI Trigger','timeout_s':.2}]);r.tick(0,.1);r.tick(1,.1)
        self.assertEqual(r.status,'FAILED');adapter.begin.assert_called_once();adapter.cancel.assert_called_once()
    def test_pause_excludes_time_and_resume(self):
        adapter=Mock();adapter.poll.return_value=False;r=MissionRunner(adapter)
        r.start([{'type':'Wait','timeout_s':2}]);r.tick(0,.1);r.pause(1);r.resume(20);r.tick(20,.1)
        self.assertEqual(r.status,'RUNNING');self.assertAlmostEqual(r.elapsed,1)
    def test_branch_and_repeat(self):
        c,r=self.make_runner();c.sim.di[0]=True
        r.start([{'type':'Branch DI','target':3},{'type':'Set DO','channel':1,'value':True},
                 {'type':'Set DO','channel':2,'value':True}],repeat=2)
        self.drive(c,r,20);self.assertEqual(r.status,'COMPLETED');self.assertEqual(r.cycle,2)
        self.assertFalse(c.sim.do[1]);self.assertTrue(c.sim.do[2])
    def test_invalid_actions_rejected_before_execution(self):
        for actions in ([{'type':'Branch DI','target':5}],[{'type':'Set DO','channel':100}],[{'type':'Wait','duration_s':-1}]):
            with self.assertRaises(ValueError):validate_actions(actions)


class PeripheralTests(unittest.TestCase):
    def test_tcp_operation_payload_and_reply_type(self):
        cfg={'operations':{'set_do':dict(verified=True,role='write',api=6001,port=19210,response_type=16001,
                                        payload={'channel':'${channel}','value':'${value}'})}}
        with patch('seer_control.studio_devices.PersistentSeerSession') as session:
            session.return_value.request.return_value={'_response_type':16001,'ret_code':0}
            result=tcp_operation('127.0.0.1',cfg,'set_do',dict(channel=2,value=True))
            session.return_value.request.assert_called_once_with(6001,dict(channel=2,value=True))
            self.assertEqual(result['ret_code'],0);session.return_value.close.assert_called_once()
            session.return_value.request.return_value={'_response_type':16002}
            with self.assertRaises(ValueError):tcp_operation('127.0.0.1',cfg,'set_do',dict(channel=2,value=True))
    def test_xmlrpc_method_mapping_and_structured_status(self):
        cfg=dict(verified=True,endpoint='http://127.0.0.1:20003/RPC2',execute_method='run',status_method='state',
                 success_value=0,success_path='code',status_path='status',operations={'work':[3]})
        with patch('seer_control.studio_devices.xmlrpc.client.ServerProxy') as proxy:
            client=proxy.return_value.__enter__.return_value
            client.run.return_value={'code':0};client.state.return_value={'status':'COMPLETED'}
            arm_call(cfg,'execute','work');client.run.assert_called_once_with(3)
            self.assertEqual(arm_call(cfg,'status'),'COMPLETED')
            with self.assertRaises(ValueError):arm_call(dict(cfg,verified=False),'execute','work')
            with self.assertRaises(ValueError):arm_call(cfg,'execute','unknown')
    def test_device_bridge_error_is_reported(self):
        bridge=DeviceBridge()
        try:
            token=bridge.submit(lambda:1/0)
            got=bridge.results.get(timeout=2)
            self.assertEqual(got[0],token);self.assertFalse(got[1]);self.assertIn('division',got[2])
        finally:bridge.close()
    def test_real_autocharge_checks_charging_before_resume(self):
        state=dict(x=0,y=0,battery=10,speed=0,charging=False)
        runner=SimpleNamespace(active=True,status='RUNNING',entered=False,error='',adapter=Mock())
        m=map_fixture()
        c=SimpleNamespace(studio_runner=runner,studio_config={'auto_charge':dict(enabled=True,low=20,high=80)},
            current_state=lambda:state,_studio_real_charge_phase='IDLE',connected=True,control_enabled=True,
            map=m,robot_stations={'CP':{}},send_command=Mock(),_station_nav_payload=lambda n:dict(id=n['id']),log=Mock(),last_state=100)
        self.assertTrue(StudioMixin._studio_charge_tick(c,100))
        c.send_command.assert_called_once_with('navigate',dict(id='CP'))
        state['charging']=True
        self.assertTrue(StudioMixin._studio_charge_tick(c,101));self.assertEqual(c._studio_real_charge_phase,'CHARGING')
        state['battery']=85
        self.assertFalse(StudioMixin._studio_charge_tick(c,102));self.assertEqual(c._studio_real_charge_phase,'IDLE')
    def test_cloud_erase_preserves_remaining_smap_metadata(self):
        m=map_fixture();m.cloud=[(0,0)]
        m.smap_source={'normalPosList':[dict(x=0,y=0,quality=.9),dict(x=1,y=1,quality=.8)]}
        StudioMixin._studio_sync_cloud(SimpleNamespace(map=m))
        self.assertEqual(m.smap_source['normalPosList'],[dict(x=0,y=0,quality=.9)])


if __name__=='__main__':unittest.main()
