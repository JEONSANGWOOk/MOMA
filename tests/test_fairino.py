import copy
import math
from pathlib import Path
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from seer_control.fairino_api import SDKEngine, profile, validate, checked
from seer_control.fairino_client import FairinoClient
from seer_control.fairino_ui import FairinoUIMixin
from seer_control.studio_ui import ConsoleAdapter
from seer_control.studio_core import MissionRunner


class FakeRobot:
    def __init__(self):
        self.joints=[0.]*6;self.tcp=[0.]*6;self.done=1;self.emergency=0;self.safety=[0,0];self.errors=[0,0];self.comm=0
        self.MoveJ=Mock(return_value=0);self.MoveL=Mock(return_value=0)
        self.StopMotion=Mock(return_value=0);self.PauseMotion=Mock(return_value=0);self.ResumeMotion=Mock(return_value=0)
    def GetSDKComState(self):return 0,self.comm
    def GetActualJointPosDegree(self):return 0,self.joints
    def GetActualTCPPose(self):return 0,self.tcp
    def GetRobotMotionDone(self):return 0,self.done
    def GetRobotEmergencyStopState(self):return 0,self.emergency
    def GetSafetyStopState(self):return 0,self.safety
    def GetRobotErrorCode(self):return 0,self.errors


def config():
    value=profile();value.update(verified=True,version_confirmed=True,controller_version='3.9.4',operations={
        'work':dict(method='MoveJ',target=[10,20,30,40,50,60],tool=0,user=0,vel=10),
        'safe_pose':dict(method='MoveJ',target=[0]*6,tool=0,user=0,vel=10),
        'linear':dict(method='MoveL',target=[100,200,300,0,90,0],vel=5)})
    return value


class FairinoTests(unittest.TestCase):
    def setUp(self):self.cfg=config();self.robot=FakeRobot();self.engine=SDKEngine(self.cfg,self.robot)
    def test_nonblocking_movej_signature(self):
        self.engine.execute('work')
        self.robot.MoveJ.assert_called_once_with([10.,20.,30.,40.,50.,60.],tool=0,user=0,vel=10.,ovl=100.,blendT=0.)
    def test_nonblocking_movel_signature(self):
        self.engine.execute('linear')
        self.robot.MoveL.assert_called_once_with([100.,200.,300.,0.,90.,0.],tool=0,user=0,vel=5.,ovl=100.,blendR=0.)
    def test_completion_requires_arrival_not_old_done_bit(self):
        self.engine.execute('work');self.engine.command_time-=1
        self.assertEqual(self.engine.status()['status'],'RUNNING')
        self.robot.joints=[10,20,30,40,50,60];self.robot.done=0
        self.assertEqual(self.engine.status()['status'],'RUNNING')
        self.robot.done=1;self.assertEqual(self.engine.status()['status'],'COMPLETED')
    def test_units_feedback(self):
        self.robot.joints=[180]*6
        self.assertEqual(self.engine.status()['joints_rad'],[math.pi]*6)
    def test_error_emergency_and_safety_never_move(self):
        for field,value in [('emergency',1),('safety',[1,0]),('errors',[3,0]),('comm',1),('done',0)]:
            with self.subTest(field=field):
                robot=FakeRobot();setattr(robot,field,value);engine=SDKEngine(self.cfg,robot)
                with self.assertRaises((ValueError,RuntimeError)):engine.execute('work')
                robot.MoveJ.assert_not_called()
    def test_motion_requires_version_confirmation(self):
        for key,value in [('version_confirmed',False),('verified',False),('controller_version','')]:
            self.engine.config=copy.deepcopy(self.cfg);self.engine.config[key]=value
            with self.assertRaises(ValueError):self.engine.execute('work')
        self.robot.MoveJ.assert_not_called()
    def test_invalid_targets_rejected(self):
        for target in ([1,2], [float('nan')]*6, [True]*6):
            cfg=config();cfg['operations']['work']['target']=target
            with self.assertRaises(ValueError):validate(cfg)
    def test_sdk_rejection_is_failure(self):
        self.robot.MoveJ.return_value=14
        with self.assertRaisesRegex(RuntimeError,'14'):self.engine.execute('work')
        self.assertIsNone(self.engine.target)
    def test_stop_does_not_mark_safe_pose_complete(self):
        self.engine.execute('safe_pose');self.engine.call('stop')
        self.assertEqual(self.engine.status()['status'],'CANCELED')
    def test_measured_safe_pose_gate_and_error(self):
        app=SimpleNamespace(studio_config={'arm':self.cfg},studio_arm_safe=False)
        FairinoUIMixin._fr5_receive(app,self.engine.status());self.assertTrue(app.studio_arm_safe)
        self.robot.emergency=1;FairinoUIMixin._fr5_receive(app,self.engine.status());self.assertFalse(app.studio_arm_safe)
    def test_parent_timeout_closes_worker(self):
        client=FairinoClient();client.connected=True;client.process=Mock();client.responses=__import__('queue').Queue()
        client.process.poll.return_value=None
        with self.assertRaises(TimeoutError):client._request('status',timeout=.02)
        self.assertFalse(client.connected);self.assertIsNone(client.process)
    def mission_console(self):
        c=SimpleNamespace(real=True,connected=True,control_enabled=True,last_state=time.monotonic(),
            current_state=lambda:dict(x=0,y=0,theta=0,speed=0,emergency=False,stopped=False),
            studio_config={'arm':self.cfg},studio_results={},studio_arm_safe=False,log=Mock(),
            studio_bridge=Mock(),send_command=Mock(),_fr5_priority_stop=Mock(),
            sim=SimpleNamespace(auto_charge=dict(phase='IDLE',enabled=False)),_jog_lock=__import__('threading').Lock(),_jog_wakeup=None)
        c._arm_call=lambda cfg,kind,operation=None:self.engine.call(kind,operation)
        c._fr5_receive=lambda result:FairinoUIMixin._fr5_receive(c,result)
        def submit(fn,callback=None):
            token=len(c.studio_results)+1
            try:result=(True,fn())
            except Exception as e:result=(False,str(e))
            c.studio_results[token]=result;return token
        c._studio_submit=submit;c.studio_runner=MissionRunner(ConsoleAdapter(c))
        return c
    def test_real_mission_waits_until_measured_arrival(self):
        c=self.mission_console();runner=c.studio_runner;now=time.monotonic()
        runner.start([dict(type='Arm Action',operation='work')]);runner.tick(now,.1)
        self.assertEqual(runner.status,'RUNNING');self.assertFalse(c.studio_arm_safe)
        self.robot.joints=self.cfg['operations']['work']['target'];self.engine.command_time-=1
        runner.tick(now+1,.1);self.assertEqual(runner.status,'COMPLETED');self.assertFalse(c.studio_arm_safe)
    def test_safe_pose_arrival_allows_next_amr_action(self):
        c=self.mission_console();runner=c.studio_runner;now=time.monotonic()
        runner.start([dict(type='Arm Action',operation='safe_pose')]);runner.tick(now,.1)
        self.engine.command_time-=1;runner.tick(now+1,.1)
        self.assertEqual(runner.status,'COMPLETED');self.assertTrue(c.studio_arm_safe)
    def test_sdk_rejection_fails_mission_and_requests_stop(self):
        c=self.mission_console();self.robot.MoveJ.return_value=19
        c.studio_runner.start([dict(type='Arm Action',operation='work')]);c.studio_runner.tick(time.monotonic(),.1)
        self.assertEqual(c.studio_runner.status,'FAILED');c._fr5_priority_stop.assert_called_once()
    def test_stop_channel_not_queued_on_sdk(self):
        client=FairinoClient();client.config=self.cfg;client.process=Mock();client.connected=True
        client.process.poll.return_value=None;process=client.process
        with patch('seer_control.fairino_client.xmlrpc.client.ServerProxy') as proxy:
            rpc=proxy.return_value.__enter__.return_value;rpc.StopMotion.return_value=0
            self.assertEqual(client.stop()['status'],'STOPPED');rpc.StopMotion.assert_called_once()
        process.terminate.assert_called_once();self.assertFalse(client.connected)
    def test_isolated_worker_protocol_with_fake_sdk(self):
        # Synthetic SDK is confined to a temp directory; no hardware connections.
        source='''class RPC:
    def __init__(self,ip): print("SDK diagnostic")
    def GetSDKComState(self): return 0,0
    def GetActualJointPosDegree(self): return 0,[0]*6
    def GetActualTCPPose(self): return 0,[0]*6
    def GetRobotMotionDone(self): return 0,1
    def GetRobotEmergencyStopState(self): return 0,0
    def GetSafetyStopState(self): return 0,[0,0]
    def GetRobotErrorCode(self): return 0,[0,0]
    def MoveJ(self,*a,**kw): return 0
'''
        with tempfile.TemporaryDirectory() as folder:
            sdk=Path(folder)/'fairino';sdk.mkdir();(sdk/'Robot.py').write_text(source,encoding='utf-8')
            cfg=config();cfg['sdk_path']=folder;client=FairinoClient()
            try:
                self.assertEqual(client.connect(cfg)['joints_deg'],[0.]*6)
                self.assertTrue(client.call(cfg,'execute','work')['accepted'])
                self.assertEqual(client.call(cfg,'status')['status'],'RUNNING')
            finally:
                process=client.process;client.close()
                if process:process.wait(timeout=5)


if __name__=='__main__':unittest.main()
