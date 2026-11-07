import unittest
from unittest.mock import Mock,patch
from concurrent.futures import Future
from seer_control.departure_link import DepartureState,DepartureLink

def ready():
    f=Future();f.set_result({'status':'success'});return f

class DepartureLinkTests(unittest.TestCase):
    def test_start_only_when_speed_rises_then_complete_once(self):
        f=DepartureState()
        self.assertEqual(f.update('LM2',0,'LM1',4,0),[])
        self.assertEqual(f.update('LM2',0,'LM1',6,.1),[])
        self.assertEqual(f.update('LM2',0,'LM1',2,.2),[])
        self.assertEqual(f.update('',.1,'LM1',2,.3),['main2'])
        self.assertEqual(f.update('',.2,'LM1',2,.4),[])
        self.assertEqual(f.update('LM1',0,'LM1',4,.5),['main3'])
        self.assertEqual(f.update('LM1',0,'LM1',4,.6),[])
    def test_midroute_zero_does_not_run_main3(self):
        f=DepartureState();f.update('LM2',0,'LM1',2,0)
        f.update('',.1,'LM1',2,.1)
        self.assertEqual(f.update('',0,'LM1',3,1),[])
        self.assertEqual(f.update('LM1',0,'LM1',2,2),[])
        self.assertEqual(f.update('LM1',0,'LM1',2,2.4),['main3'])
    def test_no_movement_no_scenarios(self):
        f=DepartureState();f.update('LM2',0,'LM1',2,0)
        self.assertEqual(f.update('LM1',0,'LM1',4,1),[])
    def test_target_change_and_failure_abort(self):
        for target,task in [('LM3',2),('LM1',5),('LM1',6)]:
            f=DepartureState();f.update('LM2',0,'LM1',2,0)
            with self.assertRaises(ValueError):f.update('',.1,target,task,.1)
    def test_worker_runs_main2_then_main3_without_output_calls(self):
        import io
        scenario=Mock();clock=[0.];calls=[];arrival=[None]
        def monotonic():clock[0]+=.1;return clock[0]
        def run_scenario(url,name):calls.append((name,clock[0]));return ready()
        scenario.start.side_effect=run_scenario
        link=DepartureLink('localhost','http://localhost:8000',scenario)
        link.nav_sent.set();link.opener=Mock();link.opener.open.return_value=io.BytesIO(b'{"connected":true}')
        link.stop.wait=Mock(return_value=False)
        states=iter([('LM2',0),('LM2',0),('',.1)]+[('LM1',0)]*30)
        tasks=iter([6,2,2]+[4]*30)
        session=Mock()
        def request(api):
            if api==1100:
                station,speed=next(states);return {'current_station':station,'vx':speed,'vy':0}
            task=next(tasks)
            if task==4 and arrival[0] is None:arrival[0]=clock[0]
            return {'target_id':'LM1','task_status':task}
        session.request.side_effect=request
        with patch('seer_control.departure_link.PersistentSeerSession',return_value=session), patch('seer_control.departure_link.time.monotonic',side_effect=monotonic):link.run()
        self.assertEqual([c.args[1] for c in scenario.start.call_args_list],['main2','main3'])
        self.assertGreaterEqual(calls[1][1]-arrival[0],2.)
        self.assertEqual(link.opener.open.call_count,1)
        events=list(link.events.queue)
        self.assertFalse([e for e in events if e[0]=='error'],events)
