import unittest,math
from seer_control.model import MapModel,Simulator
from seer_control.manual_safety import manual_motion_reason,controller_manual_reason
from seer_control.studio_core import collision_reason

def scene():
 return MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=4,y=0)],edges=[['A','B']],walls=[],obstacles=[]))
class ManualSafetyTests(unittest.TestCase):
 def test_held_forward_stops_before_wall_in_every_policy(self):
  for policy in ('wait','stop','avoid','adaptive','auto','reroute'):
   m=scene();m.walls=[[1,-2,1,2]];s=Simulator(m);s.obstacle_policy=policy
   for i in range(100):
    try:s.drive(.3,0)
    except ValueError:pass
    s.tick(.1);self.assertFalse(collision_reason(m,s.state.x,s.state.y,.23))
   self.assertTrue(s.state.blocked);self.assertLess(s.state.x,.7);self.assertEqual((s.v,s.w,s.lease,s.state.speed),(0,0,0,0))
 def test_backing_away_and_turning_are_possible_after_forward_block(self):
  m=scene();m.obstacles=[dict(x=.7,y=0,radius=.2)];s=Simulator(m);s.state.x=.15
  with self.assertRaises(ValueError):s.drive(.3,0)
  s.drive(-.2,0);s.tick(.1);self.assertLess(s.state.x,.15);self.assertFalse(s.state.blocked)
  old=(s.state.x,s.state.y);s.drive(0,.5);s.tick(.1)
  self.assertEqual((s.state.x,s.state.y),old);self.assertGreater(s.state.theta,0)
 def test_reverse_obstacle_blocks_backing_up(self):
  m=scene();m.obstacles=[dict(x=-.6,y=0,radius=.2)];s=Simulator(m)
  with self.assertRaises(ValueError):s.drive(-.3,0)
  self.assertEqual(s.state.x,0)
 def test_new_dynamic_obstacle_stops_existing_command(self):
  m=scene();s=Simulator(m);s.drive(.3,0)
  m.obstacles=[dict(id='P',x=.6,y=0,radius=.2,dynamic=True,kind='person',paused=True,motion_path=[[.6,0],[2,0]],speed_mps=.5)]
  s.tick(.1);self.assertEqual(s.state.x,0);self.assertTrue(s.state.blocked);self.assertEqual(s.lease,0)
  m.obstacles=[];s.tick(.1);self.assertEqual(s.state.x,0);self.assertTrue(s.state.blocked)
  s.drive(.2,0);s.tick(.1);self.assertGreater(s.state.x,0)
 def test_mixed_arc_checks_side_obstacle(self):
  m=scene();m.obstacles=[dict(x=.6,y=.15,radius=.2)]
  self.assertTrue(manual_motion_reason(m,(.1,0,0),.3,.6,.23))
 def test_full_amr_radius_stops_earlier(self):
  m=scene();m.walls=[[1,-2,1,2]];s=Simulator(m);s.collision_radius=.6
  for i in range(50):
   try:s.drive(.3,0)
   except ValueError:pass
   s.tick(.1)
  self.assertLess(s.state.x,.4);self.assertFalse(collision_reason(m,s.state.x,s.state.y,.6))
 def test_virtual_wall_and_forbidden_area_stop_manual_motion(self):
  for virtual in (True,False):
   m=scene()
   if virtual:m.virtual_walls=[[.6,-2,.6,2]]
   else:m.area_records=[dict(id='NO',points=[(.6,-2),(2,-2),(2,2),(.6,2)],properties={'forbidden':True})]
   self.assertTrue(manual_motion_reason(m,(.15,0,0),.3,0,.23))
 def test_real_controller_interlocks_and_stale_status(self):
  for state in ({'blocked':True},{'emergency':True},{'stopped':True},{'motor':False}):
   self.assertTrue(controller_manual_reason(state,10,11))
  self.assertTrue(controller_manual_reason({},10,14))
  self.assertEqual(controller_manual_reason({'blocked':False,'emergency':False},10,11),'')
