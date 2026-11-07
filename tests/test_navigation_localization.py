import unittest
from unittest.mock import Mock
from seer_control.navigation_localization import prepare_navigation

class NavigationLocalizationTests(unittest.TestCase):
    def setup_client(self):
        state=dict(x=1.,y=2.,angle=.1,vx=0.,vy=0.,w=0.,task_status=4,confidence=.95,blocked=False)
        client=Mock()
        def query(api):
            if api==1300:return dict(current_map='map',ret_code=0)
            if api==1021:return dict(reloc_status=1,ret_code=0)
            return dict(state)
        client.query.side_effect=query;client.command.return_value={'ret_code':0}
        ticks=[0.]
        def sleep(dt):ticks[0]+=dt
        return client,state,lambda:ticks[0],sleep
    def test_relocalizes_current_pose_then_confirms_without_navigation(self):
        c,s,clock,sleep=self.setup_client();guard=Mock()
        result=prepare_navigation(c,guard,clock=clock,sleep=sleep)
        self.assertEqual(result['map_name'],'map')
        self.assertEqual(c.command.call_args_list[0].args,('relocate',dict(x=1.,y=2.,angle=.1)))
        self.assertEqual(c.command.call_args_list[1].args,('confirm_loc',{}))
        self.assertEqual(c.command.call_count,2)
    def test_low_confidence_never_confirms(self):
        c,s,clock,sleep=self.setup_client();s['confidence']=.5
        with self.assertRaises(TimeoutError):prepare_navigation(c,Mock(),timeout=2,clock=clock,sleep=sleep)
        self.assertEqual(c.command.call_count,1)
    def test_moving_robot_rejected_before_relocalization(self):
        c,s,clock,sleep=self.setup_client();s['vx']=.1
        with self.assertRaises(ValueError):prepare_navigation(c,Mock(),clock=clock,sleep=sleep)
        c.command.assert_not_called()
    def test_connection_or_cancel_change_aborts_without_confirm(self):
        c,s,clock,sleep=self.setup_client();guard=Mock(side_effect=[None,None,ValueError('cancelled')])
        with self.assertRaises(ValueError):prepare_navigation(c,guard,clock=clock,sleep=sleep)
        self.assertEqual(c.command.call_count,1)
    def test_large_correction_blocks_confirmation(self):
        c,s,clock,sleep=self.setup_client();original=c.query.side_effect
        def command(name,payload):
            s['x']=2.;return {'ret_code':0}
        c.command.side_effect=command
        with self.assertRaises(ValueError):prepare_navigation(c,Mock(),clock=clock,sleep=sleep)
        self.assertEqual(c.command.call_count,1)
    def test_failed_relocalization_blocks_confirmation(self):
        c,s,clock,sleep=self.setup_client();original=c.query.side_effect
        c.query.side_effect=lambda api:dict(reloc_status=0) if api==1021 else original(api)
        with self.assertRaises(ValueError):prepare_navigation(c,Mock(),clock=clock,sleep=sleep)
        self.assertEqual(c.command.call_count,1)
    def test_map_change_blocks_confirmation(self):
        c,s,clock,sleep=self.setup_client();original=c.query.side_effect;maps=[0]
        def query(api):
            if api==1300:
                maps[0]+=1;return dict(current_map='map' if maps[0]==1 else 'other')
            return original(api)
        c.query.side_effect=query
        with self.assertRaises(ValueError):prepare_navigation(c,Mock(),clock=clock,sleep=sleep)
        self.assertEqual(c.command.call_count,1)

    def test_fast_success_retains_three_checks_without_old_one_second_wait(self):
        c,s,clock,sleep=self.setup_client()
        prepare_navigation(c,Mock(),clock=clock,sleep=sleep)
        self.assertGreaterEqual(clock(),.8-1e-9)
        self.assertLess(clock(),1.1)
        self.assertGreaterEqual(sum(call.args==(1021,) for call in c.query.call_args_list),5)
    def test_observed_relocating_transition_uses_fast_path(self):
        c,s,clock,sleep=self.setup_client();original=c.query.side_effect;count=[0]
        def query(api):
            if api==1021:
                count[0]+=1
                if count[0]==2:return dict(reloc_status=2)
            return original(api)
        c.query.side_effect=query
        prepare_navigation(c,Mock(),clock=clock,sleep=sleep)
        self.assertGreaterEqual(clock(),.5-1e-9)
        self.assertLess(clock(),.8)
    def test_continued_relocating_never_confirms_even_with_high_confidence(self):
        c,s,clock,sleep=self.setup_client();original=c.query.side_effect;count=[0]
        def query(api):
            if api==1021:
                count[0]+=1
                if count[0]>1:return dict(reloc_status=2)
            return original(api)
        c.query.side_effect=query
        with self.assertRaises(TimeoutError):prepare_navigation(c,Mock(),timeout=2.,clock=clock,sleep=sleep)
        self.assertEqual(c.command.call_count,1)
