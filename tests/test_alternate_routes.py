import unittest,time
from types import SimpleNamespace
from unittest.mock import Mock
from seer_control.model import MapModel,Simulator
from seer_control.studio_ui import ConsoleAdapter
from seer_control.studio_core import MissionRunner,collision_reason
from seer_control.alternate_routes import alternate_route
from seer_control.smap import make_path_record


def scene(obstacles=None):
    return MapModel(dict(format='amr-console-map-v1',name='alternate',nodes=[dict(id=k,x=x,y=y) for k,x,y in
        [('A',0,0),('B',3,0),('C',0,2),('D',3,2),('E',6,0),('F',0,4),('G',3,4)]],
        edges=[['A','B'],['A','C'],['C','D'],['D','B'],['B','E'],['D','E'],['C','F'],['F','G'],['G','D']],walls=[],obstacles=obstacles or []))


def robot(model):
    s=Simulator(model);s.obstacle_policy='reroute';s.reroute_wait_s=.5;s.reroute_attempt_limit=2
    return s


class AlternateTests(unittest.TestCase):
    def test_wait_then_follow_other_connected_lanes(self):
        m=scene([dict(x=1.5,y=0,radius=.3)]);s=robot(m);s.navigate('B');wait=False;alternate=False
        for _ in range(1000):
            s.tick(.1);wait|='정적 장애물 대기' in s.avoidance_status;alternate|='다른 연결' in s.avoidance_status
            self.assertFalse(collision_reason(m,s.state.x,s.state.y,.23))
        self.assertTrue(wait and alternate);self.assertEqual(s.state.last_node,'B');self.assertFalse(s.skipped_goals)
    def test_new_blockage_replans_again(self):
        m=scene([dict(x=1.5,y=0,radius=.3)]);s=robot(m);s.navigate('B');added=False;upper=False
        for _ in range(1700):
            s.tick(.1)
            if not added and '다른 연결' in s.avoidance_status:
                m.obstacles.append(dict(x=1.5,y=2,radius=.3));added=True
            upper|=s.state.y>3.5
        self.assertTrue(added and upper);self.assertEqual(s.state.last_node,'B')
    def test_unreachable_goal_is_marked_without_teleport(self):
        m=scene([dict(x=3,y=0,radius=.4)]);s=robot(m);s.navigate('B')
        for _ in range(500):s.tick(.1)
        self.assertIn('B',s.skipped_goals);self.assertEqual(s.skip_result['goal'],'B')
        self.assertNotEqual(s.state.last_node,'B');self.assertFalse(s.route)
        self.assertFalse(collision_reason(m,s.state.x,s.state.y,.23))
    def test_mission_skips_dependent_work_and_reaches_next_goal(self):
        m=scene([dict(x=3,y=0,radius=.4)]);s=robot(m)
        c=SimpleNamespace(sim=s,map=m,real=False,connected=True,control_enabled=False,sim_powered=True,
            studio_arm_safe=True,current_state=s.status,log=Mock(),studio_bridge=Mock(),studio_config={'arm':{}},studio_results={})
        runner=c.studio_runner=MissionRunner(ConsoleAdapter(c));runner.start([
            dict(type='Path Nav',goal='B',route_nodes=['A','B'],delay_ms=1000),
            dict(type='Arm Action',operation='work'),dict(type='Set DO',channel=0,value=True),
            dict(type='Path Nav',goal='E',route_nodes=['B','E'])])
        now=time.monotonic()
        for i in range(1700):s.tick(.1);runner.tick(now+i*.1,.1)
        self.assertEqual(runner.status,'COMPLETED',runner.error);self.assertEqual(s.state.last_node,'E')
        self.assertEqual(runner.actions[0]['status'],'패스');self.assertEqual(runner.actions[1]['status'],'목적지 미도착으로 생략')
        self.assertEqual(runner.skipped[0]['goal'],'B');self.assertFalse(s.do[0]);self.assertEqual(s.arm['status'],'IDLE')
    def test_stopped_actor_becomes_static_and_reroutes(self):
        m=scene([dict(id='P',dynamic=True,kind='person',x=1.5,y=0,radius=.3,motion_path=[[1.5,0],[1.5,2]],speed_mps=.5,paused=True)])
        s=robot(m);s.navigate('B')
        for _ in range(200):s.tick(.1)
        self.assertEqual(m.obstacles[0]['_classification'],'static');self.assertFalse(s.skipped_goals)
        for _ in range(800):s.tick(.1)
        self.assertEqual(s.state.last_node,'B')
    def test_one_way_lane_cannot_be_backtracked(self):
        m=scene([dict(x=1.5,y=0,radius=.3)])
        m.path_records=[dict(a='A',b='B',raw=make_path_record(m.nodes['A'],m.nodes['B']),properties={}),
            dict(a='A',b='C',raw=make_path_record(m.nodes['A'],m.nodes['C']),properties={})]
        self.assertIsNone(alternate_route(m,(.5,0),'B','A','B',[(0,0),(3,0)],.23))
    def test_switch_policy_cancels_disconnected_placeholder(self):
        m=scene();m.nodes['Z']=dict(id='Z',x=8,y=8);s=robot(m);s.navigate('Z')
        s.obstacle_policy='wait';s.tick(.1)
        self.assertFalse(s.route);self.assertNotEqual(s.state.last_node,'Z')
        self.assertAlmostEqual(s.state.x,0)

    def test_disconnected_goal_is_skipped_after_wait(self):
        m=scene();m.nodes['Z']=dict(id='Z',x=8,y=8);s=robot(m);s.navigate('Z')
        for _ in range(50):s.tick(.1)
        self.assertIn('Z',s.skipped_goals);self.assertAlmostEqual(s.state.x,0)


if __name__=='__main__':unittest.main()
