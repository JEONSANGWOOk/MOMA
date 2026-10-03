import unittest
from seer_control.location import LocationTracker


class LocationTests(unittest.TestCase):
    def setUp(self):
        self.tracker=LocationTracker()
        self.nodes={'A':dict(x=0,y=0),'B':dict(x=1,y=0),'C':dict(x=2,y=0)}

    def update(self, state, now=0, **kwargs):
        return self.tracker.update(self.nodes,state,context='map1',now=now,**kwargs)

    def test_sim_holds_passed_node_until_arrival(self):
        state=dict(x=.8,y=0,last_node='A',target='C')
        result=self.update(state,route=['B','C'])
        self.assertEqual((result['last'],result['next']),('A','B'))
        self.assertAlmostEqual(result['distance'],.2)
        state.update(x=1,last_node='B')
        result=self.update(state,route=['C'])
        self.assertEqual((result['last'],result['next'],result['at']),('B','C','B'))
        state.update(x=2,last_node='C',target='')
        result=self.update(state)
        self.assertEqual((result['last'],result['next'],result['at']),('C','','C'))

    def test_real_nearest_is_not_arrival_and_requires_dwell(self):
        self.assertEqual(self.update(dict(x=.6,y=0),real=True)['last'],'')
        state=dict(x=1,y=0)
        self.assertEqual(self.update(state,now=1,real=True)['last'],'')
        self.assertEqual(self.update(state,now=1.4,real=True)['last'],'B')
        self.assertEqual(self.update(dict(x=1.6,y=0),now=2,real=True)['last'],'B')

    def test_reported_arrival_overrides_estimation_and_remaining_route(self):
        result=self.update(dict(x=1,y=0,target_id='C',unfinished_path=['B','C']),real=True,sources=[dict(last_station='A')])
        self.assertEqual(result['last'],'A')
        self.assertFalse(result['estimated'])
        self.assertEqual(result['next'],'B')
        result=self.update(dict(x=1,y=0),now=1,real=True,sources=[dict(last_station='A')])
        self.assertEqual(result['last'],'A')

    def test_stale_data_and_context_switch(self):
        self.update(dict(x=0,y=0,last_node='A'))
        result=self.update(dict(x=1,y=0,last_node='B'),valid=False)
        self.assertEqual(result['last'],'A')
        self.assertFalse(result['valid'])
        result=self.tracker.update(self.nodes,{},context='map2',now=1,valid=False)
        self.assertEqual(result['last'],'')

    def test_low_confidence_and_invalid_coordinates_do_not_create_arrival(self):
        for now in (0,1):
            self.assertEqual(self.update(dict(x=0,y=0,localization=.1),now=now,real=True)['last'],'')
            self.assertEqual(self.update(dict(x=float('nan'),y=0),now=now,real=True)['last'],'')

    def test_goal_fallback_is_explicit(self):
        result=self.update(dict(x=.5,y=0,target_id='C'),real=True)
        self.assertEqual(result['next'],'C')
        self.assertTrue(result['next_is_goal'])

    def test_close_nodes_do_not_flap(self):
        self.nodes['B']['x']=.2
        self.update(dict(x=0,y=0),real=True)
        self.update(dict(x=0,y=0),now=.4,real=True)
        for now in (1,2,3):
            self.assertEqual(self.update(dict(x=.2,y=0),now=now,real=True)['last'],'A')
