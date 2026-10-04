import unittest,json
from types import SimpleNamespace
from seer_control.hardware_capabilities import controller_identity,real_state_event,FEATURES
from seer_control.studio_ui import StudioMixin,ConsoleAdapter
from seer_control.studio_core import MissionRunner
class HardwareReportingTests(unittest.TestCase):
 def test_only_received_identity_is_used(self):
  self.assertEqual(controller_identity({}),{})
  self.assertEqual(controller_identity({'api_1000':{'robot_model':'SRC','robokit_version':'test','password':'secret'}}),{'model':'SRC','version':'test'})
 def test_state_events_exclude_continuous_pose(self):
  a=dict(x=1,y=2,speed=.1,blocked=True,task='RUNNING')
  b=dict(a,x=3,speed=.2)
  self.assertEqual(real_state_event(a),real_state_event(b))
  self.assertNotEqual(real_state_event(a),real_state_event(dict(a,blocked=False)))
 def test_real_report_records_transitions_without_commands_or_sim_data(self):
  state=dict(task='RUNNING',blocked=False,emergency=False)
  c=SimpleNamespace(real=True,raw={'api_1000':{'version':'test'}},current_state=lambda:state)
  r=c.studio_runner=MissionRunner(ConsoleAdapter(c));r.start([dict(type='Path Nav',goal='B')])
  self.assertEqual(r.report.metadata['backend'],'REAL');self.assertFalse(r.report.metadata['hardware_verified'])
  StudioMixin._studio_report_update(c);StudioMixin._studio_report_update(c)
  events=[e for e in r.report.records if e['kind']=='실기 제어기 상태'];self.assertEqual(len(events),1)
  state['blocked']=True;StudioMixin._studio_report_update(c)
  state['blocked']=False;StudioMixin._studio_report_update(c)
  events=[e for e in r.report.records if e['kind']=='실기 제어기 상태'];self.assertEqual(len(events),3)
  self.assertEqual([json.loads(e['detail'])['blocked'] for e in events],[False,True,False])
  r.cycle=1;StudioMixin._studio_report_update(c);self.assertEqual(r.report.records[-1]['loop'],2)
 def test_audit_explicitly_identifies_unconnected_real_planner(self):
  rows=[row for row in FEATURES if '자유 공간' in row[0]]
  self.assertEqual(len(rows),1);self.assertIn('SIM 전용',rows[0][2])
