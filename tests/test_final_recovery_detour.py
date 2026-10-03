import unittest,time
from types import SimpleNamespace
from unittest.mock import Mock
from seer_control.model import MapModel,Simulator
from seer_control.studio_core import MissionRunner,collision_reason
from seer_control.studio_ui import ConsoleAdapter

def scene(dynamic=False,sealed=False):
    walls=[[-1,-2,7,-2],[-1,2,7,2],[-1,-2,-1,2],[7,-2,7,2]]
    if sealed:walls.append([3,-2,3,2])
    obs=dict(x=3,y=0,radius=.45,map_fixed=False)
    if dynamic:obs.update(id='AMR1',dynamic=True,kind='amr',paused=True,motion_path=[[3,0],[4,0]],speed_mps=.5)
    return MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=6,y=0)],edges=[['A','B']],walls=walls,obstacles=[obs]))
class FinalDetourTests(unittest.TestCase):
    def run_scene(self,dynamic=False,sealed=False):
        m=scene(dynamic,sealed);s=Simulator(m);s.obstacle_policy='reroute';s.reroute_wait_s=.1;s.reroute_attempt_limit=1;s.auto_static_s=.3;s.navigate('B');seen=False;sideways=False
        for i in range(1200):
            s.tick(.1);seen|='패스 전 자율 우회' in s.avoidance_status;sideways|=abs(s.state.y)>.5
            self.assertFalse(collision_reason(m,s.state.x,s.state.y,.23))
            if not s.route:break
        return s,seen,sideways
    def test_only_graph_lane_blocked_but_free_space_reaches_goal(self):
        s,seen,sideways=self.run_scene();self.assertTrue(seen and sideways)
        self.assertEqual(s.state.last_node,'B');self.assertFalse(s.skipped_goals)
    def test_stopped_dynamic_amr_is_bypassed_without_collision(self):
        s,seen,sideways=self.run_scene(True);self.assertTrue(seen and sideways)
        self.assertEqual(s.state.last_node,'B');self.assertFalse(s.skipped_goals)
    def test_sealed_room_still_skips_no_wall_crossing(self):
        s,seen,sideways=self.run_scene(sealed=True);self.assertIn('B',s.skipped_goals)
        self.assertLess(s.state.x,3);self.assertIn('자율 우회',s.skip_result['reason'])
    def test_timeout_recovers_then_completes_dependent_action(self):
        m=scene();s=Simulator(m);s.obstacle_policy='auto';s.auto_static_s=30;s.auto_wait_s=20;s.auto_scenarios['unknown']='wait_reroute'
        c=SimpleNamespace(sim=s,map=m,real=False,connected=True,sim_powered=True,studio_arm_safe=True,current_state=s.status,log=Mock(),studio_config={'arm':{}},studio_bridge=Mock(),studio_results={})
        r=c.studio_runner=MissionRunner(ConsoleAdapter(c));r.start([dict(type='Path Nav',goal='B'),dict(type='Set DO',channel=0,value=True)])
        now=time.monotonic();r.tick(now,.1);r.actions[0]['timeout_s']=2;recovered=False
        for i in range(1,1200):
            s.tick(.1);r.tick(now+i*.1,.1)
            recovered|='패스 전 자율 우회' in s.avoidance_status
            if not r.active:break
        self.assertTrue(recovered);self.assertEqual(r.status,'COMPLETED',r.error)
        self.assertFalse(r.skipped);self.assertTrue(s.do[0]);self.assertEqual(s.state.last_node,'B')
    def test_explicit_wait_and_stop_are_respected(self):
        s=Simulator(scene());s.obstacle_policy='auto';s.navigate('B')
        for scenario in ('wait','stop'):
            s._auto_block=('AMR1','static',scenario)
            self.assertFalse(s.try_recovery_detour())
    def test_footprint_already_overlapping_never_escapes_by_teleport(self):
        m=scene();s=Simulator(m);s.obstacle_policy='reroute';s.navigate('B');s.state.x=3
        self.assertFalse(s.try_recovery_detour());self.assertEqual(s.state.x,3)

    def test_full_amr_footprint_in_close_demo_aisle(self):
        from pathlib import Path
        m=MapModel.load(Path(__file__).resolve().parents[1]/'maps/demo.json')
        m.obstacles=[dict(id='AMR1',dynamic=True,kind='amr',x=4.23,y=2,radius=.61,paused=True,motion_path=[[4.23,2],[2,2]],speed_mps=.5)]
        s=Simulator(m);s.collision_radius=.6045;s.obstacle_policy='reroute';s.state.x=5.48;s.state.y=2;s.state.last_node='LM2'
        s.navigate('LM1');s.route=['LM1'];s._segment_start='LM2';s._waypoints=[(2,2)];s._reference_waypoints=[(7,2),(2,2)]
        self.assertTrue(s.try_recovery_detour());departed=False
        for i in range(1400):
            s.tick(.1);departed|=s.state.y<1
            self.assertFalse(collision_reason(m,s.state.x,s.state.y,.6045))
            if not s.route:break
        self.assertTrue(departed);self.assertEqual(s.state.last_node,'LM1');self.assertFalse(s.skipped_goals)

    def test_new_obstacle_on_bypass_replans_instead_of_canceling(self):
        m=scene();s=Simulator(m);s.obstacle_policy='reroute';s.reroute_wait_s=.1;s.reroute_attempt_limit=1;s.navigate('B')
        added=False;rerouted=False
        for i in range(1400):
            s.tick(.1)
            if not added and '패스 전 자율 우회' in s.avoidance_status and s._waypoints:
                side=-1 if min(y for x,y in s._waypoints)<-.5 else 1
                m.obstacles.append(dict(id='NEW',dynamic=True,kind='amr',x=3,y=side*.8,radius=.45,paused=True,motion_path=[[3,side*.8],[4,side*.8]],speed_mps=.5))
                added=True
            elif added and s._waypoints:
                rerouted|=any(y*side<-.5 for x,y in s._waypoints)
            self.assertFalse(collision_reason(m,s.state.x,s.state.y,.23))
            if not s.route:break
        self.assertTrue(added and rerouted);self.assertEqual(s.state.last_node,'B');self.assertFalse(s.skipped_goals)
