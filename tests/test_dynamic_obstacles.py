import unittest
import math
from seer_control.model import MapModel,Simulator
from seer_control.dynamic_obstacles import update_actors,validate_actor
from seer_control.studio_core import collision_reason
from seer_control.obstacle_ui import ObstacleUIMixin
from types import SimpleNamespace


def actor(kind='person',start=(1.5,0),end=(1.5,2),speed=.5,paused=False):
    return dict(id='P1',dynamic=True,kind=kind,x=start[0],y=start[1],radius=.3 if kind=='person' else .61,
                motion_path=[list(start),list(end)],speed_mps=speed,dwell_s=1,paused=paused)


def scene(obstacles=None,walls=None):
    return MapModel(dict(format='amr-console-map-v1',name='actors',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=4,y=0)],
        edges=[['A','B']],walls=walls or [],obstacles=obstacles or []))


class ActorTests(unittest.TestCase):
    def test_person_and_amr_move_and_return(self):
        for kind in ('person','amr'):
            with self.subTest(kind=kind):
                m=scene([actor(kind,start=(3,2),end=(3,3),speed=1)]);s=Simulator(m)
                for _ in range(10):s.tick(.1)
                self.assertAlmostEqual(m.obstacles[0]['y'],3)
                for _ in range(23):s.tick(.1)
                self.assertLess(m.obstacles[0]['y'],2.5)
    def test_pause_keeps_actor_position(self):
        m=scene([actor(start=(3,2),end=(3,3))]);m.dynamic_paused=True;s=Simulator(m)
        for _ in range(20):s.tick(.1)
        self.assertEqual(m.obstacles[0]['y'],2)
    def test_wall_and_robot_stop_actor_sweep(self):
        m=scene([actor(start=(2,1),end=(2,-1),speed=2)],walls=[[1,0,3,0]]);s=Simulator(m)
        for _ in range(50):s.tick(.1)
        self.assertGreater(m.obstacles[0]['y'],.3)
        self.assertEqual(m.obstacles[0]['_motion'],'통행 대기')
        m=scene([actor(start=(-2,0),end=(2,0),speed=2)]);s=Simulator(m)
        for _ in range(50):
            s.tick(.1);self.assertGreater(math.hypot(m.obstacles[0]['x'],m.obstacles[0]['y']),.53)
    def test_save_load_returns_to_taught_start_without_runtime_state(self):
        m=scene([actor(start=(3,2),end=(3,3))]);s=Simulator(m)
        for _ in range(5):s.tick(.1)
        data=m.data();saved=data['obstacles'][0]
        self.assertEqual(saved['y'],2);self.assertFalse(any(k.startswith('_') for k in saved))
        self.assertAlmostEqual(MapModel(data).obstacles[0]['y'],2)
    def test_invalid_motion_parameters(self):
        for key,value in [('speed_mps',0),('speed_mps',float('nan')),('motion_path',[[1,2],[1,2]]),('kind','other')]:
            obs=actor();obs[key]=value
            with self.assertRaises(ValueError):validate_actor(obs)
    def test_dynamic_lidar_hit_moves_with_actor(self):
        m=scene([actor(start=(2,0),end=(2,2))]);s=Simulator(m)
        self.assertAlmostEqual(s.scan()[0][0],1.7)
        for _ in range(30):s.tick(.1)
        self.assertGreater(s.scan()[0][0],2)
    def test_adaptive_waits_for_dynamic_then_keeps_original_path(self):
        m=scene([actor(paused=True)]);s=Simulator(m);s.obstacle_policy='adaptive';s.navigate('B')
        for _ in range(150):s.tick(.1)
        self.assertTrue(s.state.blocked);self.assertIn('동적',s.avoidance_status)
        m.obstacles[0]['paused']=False;m.obstacles[0]['dwell_s']=60
        for _ in range(400):s.tick(.1);self.assertAlmostEqual(s.state.y,0)
        self.assertEqual(s.state.last_node,'B')
    def test_adaptive_detours_static_obstacle(self):
        m=scene([dict(x=2,y=0,radius=.3)]);s=Simulator(m);s.obstacle_policy='adaptive';s.navigate('B');ys=[]
        for _ in range(700):
            s.tick(.1);ys.append(abs(s.state.y));self.assertFalse(collision_reason(m,s.state.x,s.state.y,.23))
        self.assertGreater(max(ys),.5);self.assertEqual(s.state.last_node,'B')
    def test_stop_policy_stays_latched_until_manual_release(self):
        m=scene([dict(x=2,y=0,radius=.3)]);s=Simulator(m);s.obstacle_policy='stop';s.navigate('B')
        for _ in range(150):s.tick(.1)
        self.assertTrue(s._obstacle_latched);x=s.state.x;m.obstacles=[]
        for _ in range(100):s.tick(.1)
        self.assertEqual(s.state.x,x)
        app=SimpleNamespace(sim=s,sim_required=lambda:None)
        ObstacleUIMixin._obstacles_resume(app)
        for _ in range(400):s.tick(.1)
        self.assertEqual(s.state.last_node,'B')
    def test_avoid_policy_detours_dynamic(self):
        m=scene([actor(paused=True)]);s=Simulator(m);s.obstacle_policy='avoid';s.navigate('B');ys=[]
        for _ in range(700):s.tick(.1);ys.append(abs(s.state.y))
        self.assertGreater(max(ys),.5);self.assertEqual(s.state.last_node,'B')


if __name__=='__main__':unittest.main()
