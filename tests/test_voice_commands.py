import unittest,json
from seer_control.voice_commands import parse_exact,validate_command,ground_command,command_schema
from seer_control.voice_runtime import local_url
class VoiceTests(unittest.TestCase):
 nodes=['LM1','LM2','LM3','LM7','LM8','CP1'];ops=['safe_pose','pick_and_place']
 def parse(self,s):return parse_exact(s,self.nodes,self.ops)
 def test_from_to_keeps_start_check(self):self.assertEqual(self.parse('lm1에서 lm3로 이동해'),dict(action='goto',nodes=['LM3'],start='LM1'))
 def test_spoken_korean_node_names(self):self.assertEqual(self.parse('엘엠 삼으로 이동해'),dict(action='goto',nodes=['LM3']))
 def test_ordered_destinations(self):self.assertEqual(self.parse('LM2, LM7, LM8 순서로 이동해')['nodes'],['LM2','LM7','LM8'])
 def test_loop_count(self):self.assertEqual(self.parse('LM1, LM3 세 바퀴 돌아'),dict(action='loop',nodes=['LM1','LM3'],repeats=3))
 def test_unknown_node_rejected(self):
  with self.assertRaises(ValueError):self.parse('LM999로 이동해')
 def test_negation_never_executes(self):
  for s in ['LM3로 이동하지마','정지하지 말고 LM3로 이동해','LM3 말고 LM1로 이동해']:
   with self.assertRaises(ValueError):self.parse(s)
 def test_stop_and_status_shortcuts(self):self.assertEqual(self.parse('멈춰')['action'],'stop');self.assertEqual(self.parse('현재 위치 알려줘')['action'],'status')
 def test_arm_only_registered_operation(self):self.assertEqual(self.parse('로봇팔 안전 자세로 이동해'),dict(action='arm_action',operation='safe_pose'))
 def test_mission_controls(self):
  for phrase,action in [('미션 취소해','cancel'),('미션 일시정지','pause'),('미션 재개','resume')]:self.assertEqual(self.parse(phrase)['action'],action)
 def test_llm_cannot_inject_payload_or_api(self):
  for cmd in [{'action':'execute','code':'motor()'},{'action':'goto','nodes':['LM3'],'vx':1},{'action':'arm_action','operation':'unregistered'},{'action':'goto','nodes':['LM99']}]:
   with self.assertRaises(ValueError):validate_command(cmd,self.nodes,self.ops)
 def test_repeats_must_be_bounded_integer(self):
  for n in (True,0,101,-1,2.5):
   with self.assertRaises(ValueError):validate_command(dict(action='loop',nodes=['LM1','LM3'],repeats=n),self.nodes,self.ops)
 def test_llm_must_ground_destination_in_input(self):
  with self.assertRaises(ValueError):ground_command(dict(action='goto',nodes=['LM3']),'배터리 알려줘')
  self.assertEqual(ground_command(dict(action='goto',nodes=['LM3']),'목적지를 LM3로 잡아줘')['action'],'goto')
 def test_llm_cannot_invent_stop_or_arm_work(self):
  for value,text in [({'action':'stop'},'LM3에 도착해서 기다려'),({'action':'arm_action','operation':'safe_pose'},'팔을 움직여봐')]:
   with self.assertRaises(ValueError):ground_command(value,text)
 def test_localhost_only(self):
  self.assertEqual(local_url('http://127.0.0.1:12634/'),'http://127.0.0.1:12634')
  for u in ['https://127.0.0.1','http://example.com','http://user:pass@127.0.0.1','file:///tmp','http://127.0.0.1?key=x']:
   with self.assertRaises(ValueError):local_url(u)
 def test_schema_has_per_action_fields(self):
  branches=command_schema(self.nodes,self.ops)['oneOf'];status=next(x for x in branches if x['properties']['action']['enum']==['status']);self.assertEqual(set(status['properties']),{'action'})

 def test_spoken_charging_node(self):self.assertEqual(self.parse("씨 피 일로 이동해"),dict(action="goto",nodes=["CP1"]))
