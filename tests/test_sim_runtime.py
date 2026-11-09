import copy
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
from seer_control.sim_runtime import sim_programs,SimScenario,SimDepartureLink

def config():
    return {'operations':{n:{'method':'MoveJ','target':[0]*6} for n in ('safe_pose','demo_a','demo_b')},'programs':{}}

def app_fixture():
    callbacks=[]
    app=NS(real=False,sim_scenario_source='program',arm_dev_config=config(),world3d=NS(cad_doors_auto=Mock()),arm_dev_sim=NS(state='IDLE',io={'DO':{}},error='',start=Mock(),stop=Mock()))
    app.after=lambda delay,fn:callbacks.append(fn)
    return app,callbacks

class SimRuntimeTests(unittest.TestCase):
    def test_program_defaults_do_not_modify_original_or_override_user(self):
        data=config();result=sim_programs(data)
        self.assertEqual(data['programs'],{})
        self.assertEqual([s['operation'] for s in result['programs']['main2']],['safe_pose','demo_a'])
        result['programs']['main2']=[{'operation':'safe_pose'}]
        self.assertEqual(sim_programs(result)['programs']['main2'],[{'operation':'safe_pose'}])
    def test_missing_operations_requires_program_mapping(self):
        with self.assertRaises(ValueError):sim_programs({'operations':{},'programs':{}})
    def test_local_arm_execution_never_opens_network(self):
        app,callbacks=app_fixture();manager=SimScenario(app)
        with patch('urllib.request.OpenerDirector.open',side_effect=AssertionError('network prohibited')),patch('socket.create_connection',side_effect=AssertionError('socket prohibited')):
            future=manager.start('http://real-robot:8000','main2')
            app.arm_dev_sim.start.assert_called_once()
            self.assertEqual(app.arm_dev_sim.start.call_args.kwargs['program'],'main2')
            app.arm_dev_sim.state='COMPLETED';callbacks.pop()()
            self.assertEqual(future.result()['source'],'SIM')
    def test_mode_change_stops_only_local_arm_and_fails_future(self):
        app,callbacks=app_fixture();future=SimScenario(app).start('','main2')
        app.real=True;callbacks.pop()()
        self.assertIsNotNone(future.exception());app.arm_dev_sim.stop.assert_called_once()
    def test_failed_local_arm_blocks_followup(self):
        app,callbacks=app_fixture();future=SimScenario(app).start('','main3')
        app.arm_dev_sim.state='FAILED';app.arm_dev_sim.error='collision';callbacks.pop()()
        self.assertIn('collision',str(future.exception()))
    def test_real_mode_rejects_sim_before_any_arm_movement(self):
        app,callbacks=app_fixture();app.real=True
        with self.assertRaises(ValueError):SimScenario(app).start('','main2')
        app.arm_dev_sim.start.assert_not_called()
    def test_sim_departure_sequences_main2_delay_main3_without_network(self):
        from concurrent.futures import Future
        app,callbacks=app_fixture()
        state=NS(x=1.,y=0.,speed=0.,last_node='LM2',target='',stopped=False)
        app.sim=NS(state=state,route=[]);app.map=NS(nodes={'LM1':{'x':0,'y':0},'LM2':{'x':1,'y':0}})
        app.target=Mock()
        def navigate():app.sim.route=['LM1'];state.target='LM1'
        app.navigate=navigate;calls=[];clock=[0.]
        def run(url,name):
            calls.append((name,clock[0]));f=Future();f.set_result({'status':'success'});return f
        scenario=NS(start=Mock(side_effect=run));link=SimDepartureLink(app,scenario)
        with patch('seer_control.sim_runtime.time.monotonic',side_effect=lambda:clock[0]),patch('socket.create_connection',side_effect=AssertionError('network prohibited')):
            link.start();state.speed=.2;clock[0]=.1;link.tick()
            self.assertEqual([c[0] for c in calls],['main2'])
            app.sim.route=[];state.last_node='LM1';state.speed=0.;clock[0]=1.;link.tick()
            clock[0]=2.9;link.tick();self.assertEqual(len(calls),1)
            clock[0]=3.;link.tick();self.assertEqual([c[0] for c in calls],['main2','main3'])
            link.tick();self.assertFalse(link.active)

    def test_actual_local_arm_simulator_completes_both_programs(self):
        import math
        from pathlib import Path
        from seer_control.geometry3d import RobotDescription
        from seer_control.arm_simulation import ArmSimulator
        from seer_control.fairino_api import profile
        asset=RobotDescription.load(Path(__file__).resolve().parents[1]/'examples/fairino_fr5.urdf')
        arm=ArmSimulator(asset)
        arm.set_joints([math.radians(v) for v in [0,-90,90,-90,-90,0]])
        cfg=profile();cfg['operations']={
            'safe_pose':dict(method='MoveJ',target=[0,-90,90,-90,-90,0],vel=20),
            'demo_a':dict(method='MoveJ',target=[30,-100,100,-90,-70,20],vel=20),
            'demo_b':dict(method='MoveJ',target=[-30,-80,70,-100,-100,-20],vel=20)}
        cfg=sim_programs(cfg)
        with patch('socket.create_connection',side_effect=AssertionError('network prohibited')):
            for name in ('main2','main3'):
                arm.start(cfg,program=name)
                for _ in range(500):
                    arm.tick(.05)
                    if arm.state!='RUNNING':break
                self.assertEqual(arm.state,'COMPLETED',arm.error)
