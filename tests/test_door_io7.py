import ast
import importlib.util
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from seer_control.door_feedback import SurfaceDoorFollower
from seer_control.door_io7 import DoorIO7
from seer_control.geometry3d import transform,point
from seer_control.runtime_feedback import RuntimeArmFeedback

class DoorIO7Tests(unittest.TestCase):
    def follower(self):
        faces=[((.1,0,0),(1,0,0),(1,0,1)),((.1,0,0),(1,0,1),(.1,0,1))]
        return SurfaceDoorFollower([dict(name='left:1',pivot=(0,0,0),sign=1,handle=(9,9,9),surface_faces=faces,hook_faces=faces,hook_open_only=True,roller_open_angle=True,roller_after_hook_release=True,roller_capture_gap=.04,roller_close=True)])
    def test_off_holds_and_on_does_not_force_open_without_contact(self):
        f=self.follower();gate=DoorIO7();far=(9,9,9)
        gate.update(f,far,far,transform(),0,False,'hook')
        gate.update(f,far,far,transform(rpy=(0,0,.3)),.1,True,'hook')
        self.assertEqual(f.angles['left:1'],0)
        gate.update(f,far,far,transform(rpy=(0,0,.4)),.2,None,None)
        self.assertEqual(f.angles['left:1'],0)

    def test_hook_open_handoff_and_roller_pulse(self):
        f=self.follower();gate=DoorIO7();far=(9,9,9)
        for i in range(13):
            r=transform(rpy=(0,0,math.radians(5*i)))
            gate.update(f,far,far,r,i*.1,True,'hook')
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),60)
        gate.update(f,far,far,r,1.3,False,None)
        self.assertEqual(gate.phase,'handoff')
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),60)
        gate.update(f,far,far,r,1.4,True,None)
        self.assertEqual(gate.phase,'roller')
        gate.update(f,far,far,transform(rpy=(0,0,1.5)),1.5,True,'roller')
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),60)
        r=transform(rpy=(0,0,math.radians(60)))
        gate.update(f,far,point(r,(.7,-.06,.5)),r,1.6,True,'roller')
        gate.update(f,far,point(r,(.7,-.026,.5)),r,1.7,True,'roller')
        self.assertTrue(gate.engaged)
        for i in range(1,21):
            r=transform(rpy=(0,0,math.radians(60+5*i)))
            gate.update(f,far,point(r,(.7,-.026,.5)),r,1.7+i*.1,True,'roller')
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),120)
        gate.update(f,far,far,r,3.8,False,None)
        self.assertEqual(gate.phase,'open_hold')
        gate.update(f,far,far,r,3.9,True,None)
        self.assertEqual(gate.phase,'closing')
        gate.update(f,far,far,r,4.0,False,None)
        for i in range(40):gate.update(f,far,far,None,4.1+i*.1,False,None)
        self.assertEqual(f.angles['left:1'],0)

    def test_receiving_gap_preserves_opening_reference_without_recapture(self):
        f=self.follower();gate=DoorIO7();far=(9,9,9)
        gate.update(f,far,far,transform(),0,True,None)
        gate.update(f,far,far,transform(rpy=(0,0,math.radians(20))),.1,True,None)
        gate.update(f,None,None,None,1.0,None,None)
        gate.update(f,far,far,transform(rpy=(0,0,math.radians(60))),1.2,True,None)
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),60)
        self.assertEqual(gate.phase,'hook')

    def test_small_reverse_rotation_does_not_release_or_inflate_angle(self):
        f=self.follower();gate=DoorIO7()
        for i,deg in enumerate((0,10,9,10)):
            r=transform(rpy=(0,0,math.radians(deg)))
            gate.update(f,point(r,(.7,0,.5)),None,r,i*.1,True,'hook')
        self.assertTrue(gate.engaged)
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),10)

    def test_closing_off_smoothly_finishes_partial_door_only_after_third_pulse(self):
        f=self.follower();f.angles['left:1']=math.radians(80);gate=DoorIO7();gate.phase='open_hold'
        far=(9,9,9);rot=transform()
        gate.update(f,far,far,rot,0.,False,'roller')
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),80)
        gate.update(f,far,far,rot,.1,True,'roller')
        gate.update(f,far,far,rot,.2,False,'roller')
        self.assertEqual(gate.phase,'auto_close')
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),80)
        previous=f.angles['left:1']
        for i in range(1,50):
            gate.update(f,far,far,None,.2+i*.05,False,None)
            self.assertLessEqual(f.angles['left:1'],previous+1e-12)
            self.assertLessEqual(previous-f.angles['left:1'],math.radians(3.1))
            previous=f.angles['left:1']
        self.assertEqual(gate.phase,'waiting');self.assertEqual(previous,0.)

    def test_sixty_degree_hook_stage_for_each_hinge_direction(self):
        for sign in (-1,1):
            f=self.follower();f.doors['left:1']['sign']=sign;gate=DoorIO7()
            for i in range(13):
                r=transform(rpy=(0,0,math.radians(5*i)*sign))
                gate.update(f,point(r,(.7,0,.5)),None,r,i*.1,True,'hook')
            self.assertAlmostEqual(math.degrees(f.angles['left:1']),60)

    def test_auto_close_pauses_on_missing_signal_without_resuming_with_jump(self):
        f=self.follower();f.angles['left:1']=math.radians(80);gate=DoorIO7();gate.phase='open_hold'
        far=(9,9,9)
        gate.update(f,far,far,transform(),0,True,'roller')
        gate.update(f,far,far,transform(),.1,False,'roller')
        gate.update(f,None,None,None,.2,False,None)
        held=f.angles['left:1']
        gate.update(f,None,None,None,10,None,None)
        self.assertEqual(f.angles['left:1'],held)
        gate.update(f,None,None,None,10.1,False,None)
        self.assertGreater(math.degrees(f.angles['left:1']),60)

    def test_runtime_rejects_stale_or_missing_do7(self):
        r=RuntimeArmFeedback('http://localhost:8000');r._sample=(time.monotonic(),[]);r._io_changed=time.monotonic()
        r._payload={'robot':{'vacuum_io':{'valid':True,'outputs':{'7':True}},'tool':{'x':-12.6,'y':-141.265,'z':48.}}}
        self.assertEqual(r.opening_feedback(),(True,'hook'))
        r._io_changed-=2;self.assertEqual(r.opening_feedback(),(None,None))

    def test_remote_patch_only_reads_and_does_not_break_vacuum_on_do7_error(self):
        path=Path(__file__).resolve().parents[1]/'tools/runtime_viewer_patches/enable_do7_feedback.py'
        spec=importlib.util.spec_from_file_location('do7patch',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as temp:
            target=Path(temp)/'viewer.py';target.write_text('def read_vacuum_io(moma):\n    return {}\n')
            module.install(target);tree=ast.parse(target.read_text());fn=tree.body[0]
            calls=[n for n in ast.walk(fn) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='OutputSignal']
            self.assertTrue(all(len(n.args)==2 for n in calls))
            def getter(mode,i):
                if i==7:raise RuntimeError('unsupported')
                return False
            env={'time':time};exec(compile(ast.Module(body=[fn],type_ignores=[]),'test','exec'),env)
            data=env['read_vacuum_io'](NS(OutputSignal=getter,CNTMOD=NS(CNTROB=0)))
            self.assertTrue(data['valid']);self.assertEqual(set(data['outputs']),{'2','3','4','5'})

class DirectFeedbackTests(unittest.TestCase):
    def test_existing_sdk_do7_is_preferred_without_connecting(self):
        from unittest.mock import Mock
        from seer_control.door_io7 import read_opening_feedback
        now=time.monotonic();runtime=NS(opening_feedback=Mock(return_value=(False,'hook')),tool_kind=lambda:'roller')
        client=NS(connected=True,connect=Mock())
        app=NS(fr5_client=client,fr5_rx=now,fr5_feedback={'driver':'fairino','io':{'DO':{'7':True}}},runtime_arm_feedback=runtime)
        self.assertEqual(read_opening_feedback(app,now),(True,'roller'));client.connect.assert_not_called();runtime.opening_feedback.assert_not_called()
        app.fr5_feedback['tool_index']=1;app.runtime_arm_feedback=None
        self.assertEqual(read_opening_feedback(app,now),(True,'hook'))
        app.runtime_arm_feedback=runtime
        app.fr5_rx-=2
        self.assertEqual(read_opening_feedback(app,now),(False,'hook'))

if __name__=='__main__':unittest.main()
