import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
from seer_control.gamepad import Sample
from seer_control.gamepad_missions import MissionButtons,execute

class MissionButtonsTests(unittest.TestCase):
    def sample(self,buttons=0,axis=0,t=1):return Sample({'X':axis,'Y':0},buttons,t,('test',))
    def test_each_button_fires_only_once_and_hotplug_held_is_ignored(self):
        gate=MissionButtons()
        self.assertEqual(gate.update(self.sample(2),1,True,True)[0],None)
        gate.update(self.sample(0,t=1.1),1.1,True,True)
        self.assertEqual(gate.update(self.sample(2,t=1.2),1.2,True,True)[0],'link')
        self.assertEqual(gate.update(self.sample(2,t=1.3),1.3,True,True)[0],None)
        gate.update(self.sample(0,t=1.4),1.4,True,True)
        self.assertEqual(gate.update(self.sample(1,t=1.5),1.5,True,True)[0],'return')
    def test_neutral_enable_freshness_and_single_button_required(self):
        for enabled,axis,buttons in ((False,0,2),(True,.8,2),(True,0,3)):
            gate=MissionButtons();gate.update(self.sample(),1,True,True)
            self.assertIsNone(gate.update(self.sample(buttons,axis,1.1),1.1,True,enabled)[0])
        gate=MissionButtons();gate.update(self.sample(),1,True,True)
        self.assertIsNone(gate.update(self.sample(2),2,True,True)[0])
    def app(self,station):
        return NS(real=False,connected=True,control_enabled=True,current_state=lambda:dict(speed=0,last_node=station),task_running=False,studio_runner=NS(active=False),studio_arm_safe=True,_pad_stop=Mock(),release_drive=Mock(),pad_enabled=Mock(),pad_gate=Mock(),target=Mock(),navigate=Mock(),log=Mock())
    def test_link_reuses_existing_scenario_sequence_without_running_robot(self):
        app=self.app('LM2')
        with patch('seer_control.departure_link.show_departure_link') as start:
            execute(app,'link');start.assert_called_once_with(app,autostart=True)
        app.pad_enabled.set.assert_called_once_with(False);self.assertTrue(app._pad_mission_stop)
    def test_return_only_navigates_to_lm2(self):
        app=self.app('LM1');execute(app,'return')
        app.target.set.assert_called_once_with('LM2');app.navigate.assert_called_once()
    def test_wrong_station_or_motion_does_not_dispatch(self):
        app=self.app('LM2')
        with self.assertRaises(ValueError):execute(app,'return')
        app.navigate.assert_not_called();app._pad_stop.assert_not_called()
        app=self.app('LM1');app.current_state=lambda:dict(speed=.2,last_node='LM1')
        with self.assertRaises(ValueError):execute(app,'return')
        app.navigate.assert_not_called()

class RuntimePreflightTests(unittest.TestCase):
    def test_fault_or_missing_status_blocks_before_navigation(self):
        from seer_control.departure_link import validate_runtime_status
        for payload in ({},{'servo':{'error':[4294967300,3101]}},{'servo':{'error':[0]}},{'servo':{'error':[False,0]}}):
            with self.assertRaises(ValueError):validate_runtime_status(payload)
        self.assertTrue(validate_runtime_status({'servo':{'error':[0,0]}}))
