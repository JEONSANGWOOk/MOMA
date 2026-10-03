import tempfile, unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
from seer_control.app import Console
from seer_control.pose_recovery import normalized_pose_record,save_pose,load_pose,map_matches,confidence_ok,nearest_node,nearest_reachable_node
from seer_control.model import MapModel, Simulator

class PoseRecoveryTests(unittest.TestCase):
    def test_roundtrip(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'pose.json'; rec=normalized_pose_record('M1',1.2,3.4,.5,.91,'SIMULATION','2026-01-01T00:00:00')
            save_pose(p,rec); got=load_pose(p)
            self.assertEqual(got['map_name'],'M1'); self.assertAlmostEqual(got['x'],1.2); self.assertAlmostEqual(got['theta'],.5)
    def test_map_guard_and_confidence(self):
        self.assertTrue(map_matches('A','A')); self.assertFalse(map_matches('A','B')); self.assertTrue(confidence_ok(.9,.8)); self.assertFalse(confidence_ok(.7,.8))
    def test_nearest(self):
        nodes={'A':{'x':0,'y':0},'B':{'x':2,'y':0}}
        self.assertEqual(nearest_node(nodes,1.8,.1)[0],'B')
    def test_nearest_reachable(self):
        m=MapModel({'format':'amr-console-map-v1','name':'M','nodes':[{'id':'A','x':0,'y':0},{'id':'B','x':1,'y':0},{'id':'C','x':.1,'y':.1}], 'edges':[['A','B']], 'walls':[]})
        self.assertEqual(nearest_reachable_node(m,.12,.12)[0],'A')
    def test_sim_restart_pose_application(self):
        m=MapModel({'format':'amr-console-map-v1','name':'M','nodes':[{'id':'A','x':0,'y':0},{'id':'B','x':2,'y':0}], 'edges':[['A','B']], 'walls':[]})
        s=Simulator(m); s.state.x=1.25; s.state.y=.2; s.state.theta=.4
        rec=normalized_pose_record('M',s.state.x,s.state.y,s.state.theta,s.state.localization,'SIMULATION')
        s2=Simulator(m); s2.state.x=rec['x']; s2.state.y=rec['y']; s2.state.theta=rec['theta']
        self.assertAlmostEqual(s2.state.x,1.25); self.assertAlmostEqual(s2.state.theta,.4)

    def test_console_startup_restores_saved_pose_before_autosave(self):
        m=MapModel({'format':'amr-console-map-v1','name':'M','nodes':[{'id':'A','x':0,'y':0},{'id':'B','x':2,'y':0}], 'edges':[['A','B']], 'walls':[]})
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'pose.json'
            save_pose(p,normalized_pose_record('M',1.25,.2,.4,.91,'SIMULATION'))
            c=SimpleNamespace(real=False,map=m,sim=Simulator(m),sim_powered=True,
                pose_autorecover=Mock(get=Mock(return_value=False)),
                pose_resume_loop=Mock(get=Mock(return_value=False)),
                _load_last_pose=lambda:load_pose(p),_current_map_name=lambda:'M',
                _update_pose_status=Mock(),log=Mock(),draw_map=Mock(),after=Mock(),
                connection_text=Mock())
            c.recover_last_pose=lambda auto=False:Console.recover_last_pose(c,auto)
            c._startup_pose_recovery_check=lambda:Console._startup_pose_recovery_check(c)
            c._startup_pose_recovery_check()
            self.assertAlmostEqual(c.sim.state.x,1.25)
            self.assertAlmostEqual(c.sim.state.y,.2)
            self.assertAlmostEqual(c.sim.state.theta,.4)
            c.after.assert_not_called()
            Console.sim_reboot(c)
            self.assertAlmostEqual(c.sim.state.x,1.25)
            self.assertAlmostEqual(c.sim.state.theta,.4)
            for rec in (normalized_pose_record('OTHER',9,9,1,source='SIMULATION'),
                        normalized_pose_record('M',9,9,1,source='REAL')):
                save_pose(p,rec)
                Console.sim_reboot(c)
                self.assertEqual(c.sim.state.x,0)

if __name__=='__main__': unittest.main()
