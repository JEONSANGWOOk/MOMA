import unittest
from seer_control.model import MapModel,Simulator
from seer_control.avoidance import detour,clear_segment,rejoin_detour
from seer_control.studio_core import collision_reason


def fixture(walls=None,obstacles=None):
    return MapModel(dict(format='amr-console-map-v1',name='avoidance',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=3,y=0)],edges=[['A','B']],walls=walls or [],obstacles=obstacles or []))


class AvoidanceTests(unittest.TestCase):
    def test_rejoins_near_obstacle_before_far_destination(self):
        m=fixture(obstacles=[dict(x=1.5,y=0,radius=.3)])
        reference=[(0,0),(10,0)]
        path=rejoin_detour(m,(.4,0),reference,.23)
        self.assertTrue(path)
        peak=max(range(len(path)),key=lambda i:abs(path[i][1]))
        rejoin=next(p for p in path[peak+1:] if abs(p[1])<1e-8)
        self.assertLess(rejoin[0],2.4)
        self.assertEqual(path[-1],(10,0))
        self.assertTrue(all(clear_segment(m,a,b,.23) for a,b in zip(path,path[1:])))
    def test_rejoin_preserves_bent_reference_suffix(self):
        m=fixture(obstacles=[dict(x=1,y=0,radius=.2)])
        reference=[(0,0),(2,0),(4,2),(7,2)]
        path=rejoin_detour(m,(.1,0),reference,.23)
        self.assertTrue(path);self.assertIn((4.,2.),path);self.assertEqual(path[-1],(7.,2.))
    def test_sim_rejoins_original_line_early(self):
        m=fixture(obstacles=[dict(x=1.5,y=0,radius=.3)]);m.nodes['B']['x']=8
        s=Simulator(m);s.obstacle_policy='avoid';s.navigate('B');passed=False
        for _ in range(1500):
            s.tick(.1)
            if s.state.x>2.4:
                passed=True;self.assertAlmostEqual(s.state.y,0,places=5)
        self.assertTrue(passed);self.assertEqual(s.state.last_node,'B')
    def test_local_obstacle_detour_is_clear(self):
        m=fixture(obstacles=[dict(x=1.5,y=0,radius=.3)])
        path=detour(m,(0,0),(3,0),.23)
        self.assertTrue(path);self.assertGreater(max(abs(p[1]) for p in path),.5)
        self.assertTrue(all(clear_segment(m,a,b,.23) for a,b in zip(path,path[1:])))
    def test_wall_detour(self):
        m=fixture(walls=[[1.5,-.4,1.5,.4]])
        path=detour(m,(0,0),(3,0),.23)
        self.assertTrue(path);self.assertTrue(all(clear_segment(m,a,b,.23) for a,b in zip(path,path[1:])))
    def test_closed_room_has_no_detour(self):
        m=fixture(walls=[[-1,-1,4,-1],[4,-1,4,1],[4,1,-1,1],[-1,1,-1,-1],[1.5,-1,1.5,1]])
        self.assertIsNone(detour(m,(0,0),(3,0),.23))
    def test_blocked_goal_waits(self):
        m=fixture(obstacles=[dict(x=3,y=0,radius=.3)])
        self.assertIsNone(detour(m,(0,0),(3,0),.23))
    def test_forbidden_area_avoided(self):
        m=fixture();m.area_records=[dict(id='no',points=[[1,-.3],[2,-.3],[2,.3],[1,.3]],properties={'forbidden':True})]
        path=detour(m,(0,0),(3,0),.23)
        self.assertTrue(path);self.assertTrue(all(clear_segment(m,a,b,.23) for a,b in zip(path,path[1:])))
    def test_sim_avoids_and_reaches_same_goal(self):
        m=fixture(obstacles=[dict(x=1.5,y=0,radius=.3)]);s=Simulator(m);s.obstacle_policy='avoid';s.navigate('B');ys=[]
        for _ in range(1000):
            s.tick(.1);ys.append(abs(s.state.y));self.assertFalse(collision_reason(m,s.state.x,s.state.y,.23))
        self.assertEqual(s.state.last_node,'B');self.assertGreater(max(ys),.5)
    def test_wait_resumes_after_obstacle_removed(self):
        m=fixture(obstacles=[dict(x=1.5,y=0,radius=.3)]);s=Simulator(m);s.navigate('B')
        for _ in range(200):s.tick(.1)
        self.assertTrue(s.state.blocked);self.assertLess(s.state.x,1)
        m.obstacles=[]
        for _ in range(300):s.tick(.1)
        self.assertEqual(s.state.last_node,'B')
    def test_wall_blocks_manual_motion_and_large_footprint(self):
        m=fixture(walls=[[1,-2,1,2]]);s=Simulator(m);s.collision_radius=.6
        for _ in range(100):
            if not s.state.blocked:s.drive(.3,0)
            s.tick(.1)
        self.assertLessEqual(s.state.x,.4);self.assertEqual(s.state.speed,0)
    def test_unreachable_avoid_waits_without_crossing_wall(self):
        m=fixture(walls=[[-1,-1,4,-1],[4,-1,4,1],[4,1,-1,1],[-1,1,-1,-1],[1.5,-1,1.5,1]])
        s=Simulator(m);s.obstacle_policy='avoid';s.navigate('B')
        for _ in range(200):s.tick(.1)
        self.assertTrue(s.state.blocked);self.assertLess(s.state.x,1.3)
        self.assertIn('대기',s.avoidance_status)

    def test_blocked_transit_node_can_be_bypassed(self):
        m=fixture(obstacles=[dict(x=3,y=0,radius=.3)])
        m.nodes['C']=dict(id='C',x=6.,y=0.,kind='station');m.edges.append(['B','C'])
        s=Simulator(m);s.obstacle_policy='avoid';s.navigate('C')
        for _ in range(1500):
            s.tick(.1);self.assertFalse(collision_reason(m,s.state.x,s.state.y,.23))
        self.assertEqual(s.state.last_node,'C');self.assertEqual(s.state.task,'완료')
