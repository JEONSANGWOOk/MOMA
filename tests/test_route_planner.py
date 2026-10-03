import unittest
from seer_control.model import MapModel, Simulator
from seer_control.smap import make_path_record
from seer_control.route_planner import plan_stops, plan_actions, upgrade_loop_chain
from seer_control.studio_core import validate_actions


def map_with_paths():
    model=MapModel(dict(format='amr-console-map-v1',name='routes',nodes=[dict(id=k,x=x,y=y) for k,x,y in
                   [('A',0,0),('B',1,0),('C',1,1),('D',2,0)]],edges=[['A','B'],['B','D'],['A','C'],['C','D']],walls=[]))
    model.path_records=[]
    for a,b in model.edges:
        for src,dst in ((a,b),(b,a)):
            raw=make_path_record(model.nodes[src],model.nodes[dst])
            model.path_records.append(dict(a=src,b=dst,raw=raw,properties=dict(maxspeed=.05 if 'B' in (a,b) else .45)))
    return model


class RoutePlannerTests(unittest.TestCase):
    def test_non_adjacent_stops_expand_and_fast_route_differs(self):
        model=map_with_paths()
        shortest=plan_stops(model,['A','D'])
        fastest=plan_stops(model,['A','D'],'time')
        self.assertEqual(shortest['nodes'],['A','B','D'])
        self.assertEqual(fastest['nodes'],['A','C','D'])
        self.assertGreater(fastest['distance'],shortest['distance'])
        self.assertLess(fastest['seconds'],shortest['seconds'])

    def test_detour_excludes_node_and_requires_reachable_path(self):
        model=map_with_paths()
        self.assertEqual(plan_stops(model,['A','D'],excluded=['B'])['nodes'],['A','C','D'])
        self.assertTrue(plan_stops(model,['A','D'],excluded=['B','C'])['errors'])
        self.assertTrue(plan_stops(model,['A','D'],excluded=['A'])['errors'])
        self.assertTrue(plan_stops(model,['A','D'],excluded=['Unknown'])['errors'])

    def test_duplicate_stop_is_no_motion_and_wait_only_at_requested_stops(self):
        plan=plan_stops(map_with_paths(),['A','D','D','A'])
        self.assertFalse(plan['errors'])
        self.assertEqual(plan['nodes'],['A','B','D','B','A'])
        actions=validate_actions(plan_actions(plan,500))
        self.assertEqual([a['goal'] for a in actions],['D','A'])
        self.assertEqual([a['delay_ms'] for a in actions],[500,500])
        self.assertEqual(actions[0]['route_nodes'],['A','B','D'])

    def test_existing_generated_loop_is_upgraded(self):
        chain=dict(loop_stops=['A','D','A'],loop_route=['A','B','D','B','A'],routing_policy='최단 거리',dwell_ms=300,
                   tasks=[dict(groups=[dict(actions=[dict(type='Path Nav',goal=b,route_nodes=[a,b],delay_ms=300 if b in ('D','A') else 0) for a,b in zip(['A','B','D','B'],['B','D','B','A'])])])])
        self.assertTrue(upgrade_loop_chain(map_with_paths(),chain))
        self.assertEqual([a['goal'] for a in chain['tasks'][0]['groups'][0]['actions']],['D','A'])
        self.assertFalse(upgrade_loop_chain(map_with_paths(),chain))

    def test_transit_node_spin_is_ignored_and_speed_stays_positive(self):
        for corner in (False,True):
            model=MapModel(dict(format='amr-console-map-v1',name='transit',nodes=[
                dict(id='A',x=0,y=0),dict(id='B',x=1,y=0,spin=True,r=3.14),
                dict(id='C',x=1 if corner else 2,y=1 if corner else 0,spin=True,r=1.57)],edges=[['A','B'],['B','C']],walls=[]))
            sim=Simulator(model);sim.navigate('C')
            passage=[]
            for _ in range(500):
                sim.tick(.1)
                if sim.state.last_node=='B' and sim.state.mode=='RUNNING' and ((sim.state.y<.2) if corner else sim.state.x<1.2):
                    passage.append(sim.state.speed)
                if not sim.route:break
            self.assertTrue(passage)
            self.assertTrue(all(speed>0 for speed in passage),passage)
            self.assertEqual(sim.state.last_node,'C')
            self.assertEqual(sim.state.speed,0)

    def test_one_way_paths_are_respected(self):
        model=map_with_paths();model.path_records=[r for r in model.path_records if r['a']=='A']
        plan=plan_stops(model,['B','A'])
        self.assertTrue(plan['errors'])
        self.assertEqual(plan['legs'][0]['nodes'],[])
        with self.assertRaises(ValueError):plan_actions(plan)

    def test_sim_follows_explicit_curve_instead_of_shorter_alternative(self):
        model=map_with_paths()
        raw=make_path_record(model.nodes['A'],model.nodes['D'])
        raw['controlPos1']['y']=3;raw['controlPos2']['y']=3
        model.path_records.append(dict(a='A',b='D',raw=raw,properties=dict(maxspeed=.45)))
        self.assertEqual(model.route('A','D'),['A','B','D'])
        sim=Simulator(model);sim.navigate('D',route_nodes=['A','D'])
        heights=[]
        for _ in range(800):sim.tick(.1);heights.append(sim.state.y)
        self.assertGreater(max(heights),1.)
        self.assertEqual(sim.state.last_node,'D')

    def test_invalid_explicit_route_rejected_before_moving(self):
        sim=Simulator(map_with_paths())
        with self.assertRaises(ValueError):sim.navigate('D',route_nodes=['B','D'])
        with self.assertRaises(ValueError):sim.navigate('D',route_nodes=['A','D'])
        self.assertEqual(sim.route,[])
