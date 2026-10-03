import math,unittest
from types import SimpleNamespace
from seer_control.model import MapModel
from seer_control.dynamic_obstacles import update_actors,compile_actor_path,actor_can_reverse,validate_actor
from seer_control.smap import make_path_record

class EncounterTests(unittest.TestCase):
 def actor(self,key,kind,start,end):
  return dict(id=key,dynamic=True,kind=kind,x=start[0],y=start[1],radius=.3,motion_path=[list(start),list(end)],speed_mps=.5,dwell_s=0,encounter_wait_s=.3)
 def model(self,actors):return MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=4,y=0)],edges=[['A','B']],walls=[],obstacles=actors))
 def test_people_meet_and_reverse_without_overlap(self):
  a=self.actor('P1','person',(0,0),(4,0));b=self.actor('P2','person',(4,0),(0,0));m=self.model([a,b]);reversed=False;separated=False
  for _ in range(250):
   update_actors(m,.1,SimpleNamespace(x=20,y=20),.3)
   self.assertGreater(math.dist((a['x'],a['y']),(b['x'],b['y'])),.6-1e-6)
   reversed|=a.get('_direction')==-1 or b.get('_direction')==-1
   if reversed and abs(a['x']-b['x'])>2:separated=True
  self.assertTrue(reversed and separated)
 def test_amr_and_person_keep_moving(self):
  a=self.actor('R','amr',(0,0),(4,0));a['motion_nodes']=['A','B'];b=self.actor('P','person',(4,0),(0,0));m=self.model([a,b]);reverse=False
  for _ in range(250):
   update_actors(m,.1,SimpleNamespace(x=20,y=20),.3);reverse|=a.get('_direction')==-1
   self.assertGreater(math.dist((a['x'],a['y']),(b['x'],b['y'])),.6-1e-6)
  self.assertTrue(reverse)
 def test_wait_policy_is_preserved(self):
  a=self.actor('P1','person',(0,0),(4,0));b=self.actor('P2','person',(4,0),(0,0));a['encounter_policy']=b['encounter_policy']='wait';m=self.model([a,b])
  for _ in range(100):update_actors(m,.1,SimpleNamespace(x=20,y=20),.3)
  self.assertEqual(a.get('_motion'),'통행 대기');self.assertNotEqual(a.get('_direction'),-1)
 def test_one_way_amr_reverse_is_rejected(self):
  a=self.actor('R','amr',(0,0),(4,0));a['motion_nodes']=['A','B'];m=self.model([a]);m.path_records=[dict(a='A',b='B',raw=make_path_record(m.nodes['A'],m.nodes['B']),properties={})]
  a['x']=2
  self.assertFalse(actor_can_reverse(m,a,[(0,0),(4,0),(0,0)],1,1))
 def test_manual_pause_is_preserved(self):
  a=self.actor('P','person',(0,0),(4,0));a['paused']=True;m=self.model([a])
  for _ in range(100):update_actors(m,.1,SimpleNamespace(x=20,y=20),.3)
  self.assertEqual(a['x'],0);self.assertEqual(a['_motion'],'일시정지')
 def test_invalid_encounter_wait_rejected(self):
  a=self.actor('P','person',(0,0),(4,0));a['encounter_wait_s']=float('nan')
  with self.assertRaises(ValueError):validate_actor(a)
