import unittest,tempfile,json
from pathlib import Path
from seer_control.studio_core import MissionRunner
class Adapter:
    def __init__(self):self.calls=0
    def begin(self,a):self.calls+=1
    def poll(self,a,dt,elapsed):
        if a.get('goal')=='B' and self.calls==1:return dict(skip=True,goal='B',reason='차단')
        return True
    def cancel(self):pass
class ReportTests(unittest.TestCase):
    def test_loop_audit_survives_action_reset_and_skipped_followups(self):
        r=MissionRunner(Adapter());r.start([dict(type='Path Nav',goal='B'),dict(type='Arm Action',operation='work'),dict(type='Path Nav',goal='C')],2)
        for i in range(8):r.tick(i,.1)
        d=r.report.data()
        self.assertEqual(r.status,'COMPLETED');self.assertEqual(d['completed_loops'],2)
        self.assertEqual((d['success'],d['failure'],d['skipped'],d['omitted']),(4,0,1,1))
        self.assertEqual([x['loop'] for x in d['records'] if x['kind']=='패스'],[1])
        self.assertIn('2번째 루프',r.report.text())
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'audit.json';r.report.save(p)
            self.assertEqual(json.loads(p.read_text(encoding='utf-8'))['success'],4)
            self.assertTrue(p.with_suffix('.txt').exists())
    def test_failure_and_cancel_are_separate_results(self):
        class Fail(Adapter):
            def begin(self,a):raise ValueError('연결 오류')
        r=MissionRunner(Fail());r.start([dict(type='Path Nav',goal='B')]);r.tick(1,.1)
        self.assertEqual(r.report.data()['failure'],1);self.assertIsNotNone(r.report.ended)
        r.start([dict(type='Path Nav',goal='C')]);r.cancel()
        self.assertEqual(r.report.data()['failure'],0);self.assertEqual(r.report.data()['canceled'],1)
