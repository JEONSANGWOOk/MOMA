import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock
from seer_control.geometry3d import transform
from seer_control.vacuum_panel_display import VacuumPanelDisplay,arm_vacuum_signal
from seer_control.fairino_api import read_output_io
from seer_control.runtime_feedback import RuntimeArmFeedback

class VacuumDisplayTests(unittest.TestCase):
    def test_attach_follow_release_and_unknown(self):
        panel=VacuumPanelDisplay();size=(.16,.09,.012)
        panel.update(True,transform((1,2,1)),transform(),size,.02)
        self.assertTrue(panel.attached)
        panel.update(None,transform((2,3,1)),transform(),size,.02)
        self.assertTrue(panel.attached)
        self.assertEqual(panel.matrix[0][3],2)
        panel.update(False,transform((8,8,1)),transform(),size,.02)
        self.assertFalse(panel.attached)
        self.assertEqual(panel.matrix[0][3],2)
        self.assertLess(panel.matrix[2][3],1.006)
        for _ in range(100):panel.update(False,None,transform(),size,.05)
        self.assertAlmostEqual(panel.matrix[2][3],.006)
        self.assertFalse(panel.falling)

    def test_place_near_equipment_remains_on_shelf(self):
        panel=VacuumPanelDisplay();size=(.16,.09,.012)
        initial=transform((1,2,.8))
        panel.update(True,transform((1.03,2.02,.82)),initial,size,.02)
        panel.update(False,None,initial,size,.02)
        self.assertFalse(panel.attached);self.assertFalse(panel.falling)
        self.assertAlmostEqual(panel.matrix[2][3],.8)
        placed=[list(r) for r in panel.matrix]
        panel.update(False,transform((9,9,9)),initial,size,.1)
        self.assertEqual(panel.matrix,placed)

    def test_masks_and_independent_bank_failure(self):
        robot=NS(GetDO=Mock(return_value=(0,[2,1])),GetToolDO=Mock(return_value=(0,4)))
        io,error=read_output_io(robot)
        self.assertFalse(error)
        self.assertTrue(io['DO']['0']);self.assertTrue(io['DO']['9'])
        self.assertFalse(io['ToolDO']['0']);self.assertTrue(io['ToolDO']['1'])
        robot.GetToolDO.side_effect=TimeoutError('late')
        io,error=read_output_io(robot)
        self.assertTrue(io['DO']['0']);self.assertNotIn('ToolDO',io)
        self.assertIn('late',error)

    def test_unconfigured_stale_and_inverted_signal(self):
        now=time.monotonic();app=NS(fr5_rx=now,fr5_feedback={'io':{'DO':{'0':True}}})
        self.assertIsNone(arm_vacuum_signal(app,{},now))
        cfg={'source':'FR5 SDK','channel':0,'bank':'DO','mode':'single'}
        self.assertIs(arm_vacuum_signal(app,cfg,now),True)
        self.assertIs(arm_vacuum_signal(app,dict(cfg,active_high=False),now),False)
        self.assertIsNone(arm_vacuum_signal(app,cfg,now+2))

    def test_paired_output_levels_release_without_blow(self):
        now=time.monotonic();bits={str(i):False for i in (2,3,4,5)}
        app=NS(fr5_rx=now,fr5_feedback={'io':{'DO':bits}});cfg={'source':'FR5 SDK'}
        self.assertIs(arm_vacuum_signal(app,cfg,now),False)
        bits['2']=True;bits['4']=True
        self.assertIs(arm_vacuum_signal(app,cfg,now),True)
        bits['3']=True
        self.assertIs(arm_vacuum_signal(app,cfg,now),False)
        bits.update({str(i):False for i in (2,3,4,5)})
        self.assertIs(arm_vacuum_signal(app,cfg,now),False)
        self.assertIsNone(arm_vacuum_signal(app,cfg,now+2))

    def test_runtime_io_token_must_keep_advancing(self):
        runtime=RuntimeArmFeedback('http://localhost:8000')
        data={'robot':{'joint':dict(zip(('J1','J2','J3','J4','J5','J6'),[0]*6)),
              'vacuum_io':{'valid':True,'sample_time':17.,'outputs':{'2':True,'3':False,'4':True,'5':False}}}}
        runtime._get=Mock(side_effect=[{'connected':True},data])
        runtime.poll()
        self.assertIs(runtime.vacuum_state(),True)
        runtime._io_changed=time.monotonic()-3
        self.assertIsNone(runtime.vacuum_state())
        runtime._get=Mock(side_effect=[{'connected':True},data]);runtime.poll()
        self.assertIsNone(runtime.vacuum_state())
        data['robot']['vacuum_io']['sample_time']=18.
        runtime._get=Mock(side_effect=[{'connected':True},data]);runtime.poll()
        self.assertIs(runtime.vacuum_state(),True)

    def test_runtime_missing_is_unknown_not_off(self):
        runtime=RuntimeArmFeedback('http://localhost:8000')
        runtime._sample=(time.monotonic(),(0,)*6)
        runtime._payload={'robot':{'io':{'vacuum':0}}}
        self.assertIs(runtime.signal('robot.io.vacuum'),False)
        self.assertIsNone(runtime.signal('robot.io.missing'))
        runtime._sample=(time.monotonic()-3,(0,)*6)
        self.assertIsNone(runtime.signal('robot.io.vacuum'))

if __name__=='__main__':unittest.main()
