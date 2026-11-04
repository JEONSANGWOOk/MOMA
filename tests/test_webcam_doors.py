import json
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from seer_control.webcam_doors import Observer,classify,read_live
from seer_control.door_feedback import DoorFollower
from seer_control.world3d import WorldView3D


class WebcamDoorTests(unittest.TestCase):
    def frames(self):
        rng=np.random.default_rng(9)
        closed=rng.integers(0,256,(96,192,3),dtype=np.uint8)
        opened=closed.copy();opened[:,96:]=rng.integers(0,256,(96,96,3),dtype=np.uint8)
        return closed,opened

    def observer(self):
        o=Observer();closed,opened=self.frames();o.shape=list(closed.shape[:2])
        o.profiles={'anchor':{'roi':[0,0,96,96],'reference':o.patch(closed,[0,0,96,96])},
                    'left':{'roi':[96,0,96,96],'closed':o.patch(closed,[96,0,96,96]),
                            'open':o.patch(opened,[96,0,96,96]),'angle':80}}
        return o,closed,opened

    def test_state_requires_stability_and_occlusion_is_unknown(self):
        o,closed,opened=self.observer()
        self.assertEqual(o.observe(closed,0)[0]['left']['state'],'UNKNOWN')
        self.assertEqual(o.observe(closed,.7)[0]['left']['state'],'CLOSED')
        self.assertEqual(o.observe(opened,.8)[0]['left']['state'],'UNKNOWN')
        state=o.observe(opened,1.5)[0]['left']
        self.assertEqual(state,{'state':'OPEN','display_angle_deg':80})
        hidden=opened.copy();hidden[:,96:]=0
        self.assertEqual(o.observe(hidden,1.6)[0]['left']['state'],'UNKNOWN')

    def test_camera_reference_mismatch_invalidates_detection(self):
        o,closed,opened=self.observer();o.observe(closed,0);o.observe(closed,.7)
        moved=closed.copy();moved[:,:96]=np.flip(moved[:,:96],axis=0)
        states,error=o.observe(moved,.8)
        self.assertEqual(states['left']['state'],'UNKNOWN');self.assertTrue(error)

    def test_ambiguous_templates_and_stale_publication_are_unknown(self):
        image=np.zeros((96,96),dtype=np.uint8)
        self.assertEqual(classify(image,image,image),'UNKNOWN')
        data=json.dumps({'timestamp':0,'doors':{'left':{'state':'OPEN','display_angle_deg':90}}})
        with patch('seer_control.webcam_doors.LIVE') as path:
            path.read_text.return_value=data
            self.assertEqual(read_live()['left']['state'],'UNKNOWN')

    def test_camera_observation_does_not_override_door_display(self):
        from seer_control.door_feedback import ContactDoorFollower
        f=ContactDoorFollower([dict(name='door:1',pivot=(0,0,0),handle=(1,0,1),sign=1)])
        view=NS(app=NS(real=True),placement_confirmed=lambda:True,cad_asset=None,cad_origin=[0]*6,
                scales={'cad':1},arm_asset=None,tool_asset=None,mount=[0]*6,
                _real_door_key=None,_real_door_follower=f,webcam_doors_enabled=NS(get=lambda:True),
                real_doors_follow=NS(get=lambda:False),arm_follow=NS(get=lambda:False))
        key=(id(None),tuple(view.cad_origin),1,id(None),id(None),tuple(view.mount));view._real_door_key=key
        f.angles['door:1']=.4
        with patch('seer_control.webcam_doors.read_live') as observation:
            WorldView3D._update_real_doors(view,None,{}, {})
            observation.assert_not_called()
        self.assertEqual(f.angles['door:1'],.4)



if __name__=='__main__':unittest.main()
