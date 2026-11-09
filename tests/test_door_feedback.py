import math
import unittest
from unittest.mock import patch
from types import SimpleNamespace as NS
from seer_control.door_feedback import DoorFollower
from seer_control.world3d import WorldView3D
from seer_control.geometry3d import RobotDescription, transform, point


class DoorFeedbackTests(unittest.TestCase):
    def follower(self,sign=1):
        return DoorFollower([dict(name='left',pivot=(0,0,0),handle=(1,0,1),sign=sign),
                             dict(name='right',pivot=(3,0,0),handle=(4,0,1),sign=-1)])

    def test_contact_then_measured_arc_opens_only_contacted_door(self):
        for sign in (-1,1):
            f=self.follower(sign)
            f.update((1,0,1),0)
            for i in range(1,21):
                a=math.radians(i*3)*sign
                f.update((math.cos(a),math.sin(a),1),i*.1)
            self.assertAlmostEqual(f.angles['left'],math.radians(60))
            self.assertEqual(f.angles['right'],0)
            f.update(None,2.1)
            self.assertIsNone(f.active)
            self.assertAlmostEqual(f.angles['left'],math.radians(60))

    def test_no_contact_no_opening(self):
        f=self.follower()
        for i in range(20):
            a=i*.05
            f.update((2*math.cos(a),2*math.sin(a),1),i*.1)
        self.assertEqual(f.angles,{'left':0.,'right':0.})

    def test_gap_jump_and_radial_release_hold_last_angle(self):
        for tip,stamp in [((.99,.1,1),1.),((0,1,1),.1),((1.2,0,1),.1),((1,0,1.2),.1)]:
            f=self.follower();f.update((1,0,1),0);f.update(tip,stamp)
            self.assertIsNone(f.active)
            self.assertEqual(f.angles['left'],0.)

    def test_recontact_open_door_can_close(self):
        f=self.follower();f.angles['left']=math.radians(30)
        f.update(f.handle('left'),0)
        for i in range(1,11):
            a=math.radians(30-i*3)
            f.update((math.cos(a),math.sin(a),1),i*.1)
        self.assertAlmostEqual(f.angles['left'],0)

    def test_real_display_ignores_sim_auto_animation_and_hold_angles(self):
        f=self.follower();f.angles['left']=.4
        view=NS(app=NS(real=True),_real_door_follower=f,
                cad_door_hold_angles={'left':2.},cad_door_hold_angle=2.)
        self.assertEqual(WorldView3D.cad_door_angle(view,{'name':'left'}),.4)

    def test_measured_fk_in_rotated_cad_frame_drives_door(self):
        names=[f'joint{i}' for i in range(6)]
        arm=RobotDescription('arm',{},[dict(name=n,type='revolute') for n in names],[])
        origin=[3,4,0,0,0,math.pi/2]
        cadworld=transform(origin[:3],origin[3:])
        tip=[1,0,1]
        def fk(values,world):
            target=point(cadworld,tip)
            from seer_control.door_teaching import hook_contact_offset
            offset=hook_contact_offset()
            return {'J6':transform(tuple(target[i]-offset[i] for i in range(3)))}
        arm.link_transforms=fk
        mesh=NS(faces=[((.99,0,.99),(1.01,0,.99),(1,0,1.01))],face_materials=[36])
        cad=NS(door_parts=[dict(name='left',pivot=(0,0,0),sign=1,asset=mesh)])
        view=NS(app=NS(real=True,connected=True,last_state=100),cad_asset=cad,
                placement_confirmed=lambda:True,
                cad_origin=origin,scales={'cad':1},arm_asset=arm,
                tool_asset=NS(name='EndEffector V3_EndOnly_Change1-org'),mount=[0]*6,
                real_doors_follow=NS(get=lambda:True),arm_follow=NS(get=lambda:True),
                _real_door_follower=None,_real_door_key=None)
        measured=dict.fromkeys(names,0.)
        with patch('seer_control.world3d.time.monotonic',return_value=100):
            WorldView3D._update_real_doors(view,transform(),measured,measured)
        tip[:]=[math.cos(.05),math.sin(.05),1]
        with patch('seer_control.world3d.time.monotonic',return_value=100.1):
            WorldView3D._update_real_doors(view,transform(),measured,measured)
        self.assertAlmostEqual(view._real_door_follower.angles['left'],.05)
        with patch('seer_control.world3d.time.monotonic',return_value=104):
            WorldView3D._update_real_doors(view,transform(),measured,measured)
        self.assertIsNone(view._real_door_follower.active)
        self.assertAlmostEqual(view._real_door_follower.angles['left'],.05)


class ContactDoorTests(unittest.TestCase):
    def follower(self):
        from seer_control.door_feedback import ContactDoorFollower
        return ContactDoorFollower([dict(name='left',pivot=(0,0,0),handle=(1,0,1),push=(1,0,.3),sign=1),
                                    dict(name='right',pivot=(3,0,0),handle=(4,0,1),push=(4,0,.3),sign=-1)])
    def test_roller_opens_but_cannot_pull_closed(self):
        f=self.follower();f.update_contacts(None,(1,0,.3),0)
        for i in range(1,11):
            a=i*.03;f.update_contacts(None,(math.cos(a),math.sin(a),.3),i*.1)
        self.assertAlmostEqual(f.angles['left'],.3)
        self.assertEqual(f.angles['right'],0)
        f.update_contacts(None,(math.cos(.27),math.sin(.27),.3),1.1)
        self.assertAlmostEqual(f.angles['left'],.3)
        self.assertIsNone(f.active)
    def test_hook_pull_and_missing_feedback(self):
        f=self.follower();f.update_contacts((1,0,1),None,0)
        f.update_contacts((math.cos(.05),math.sin(.05),1),None,.1)
        self.assertAlmostEqual(f.angles['left'],.05)
        f.update_contacts(None,None,.2)
        self.assertIsNone(f.active)
        self.assertAlmostEqual(f.angles['left'],.05)
    def test_no_contact_no_motion(self):
        f=self.follower()
        f.update_contacts((8,8,8),(7,7,7),0)
        self.assertEqual(f.angles,{'left':0,'right':0})
        self.assertIsNone(f.active)


class SurfaceContactTests(unittest.TestCase):
    def follower(self):
        from seer_control.door_feedback import SurfaceDoorFollower
        faces=[((.2,0,0),(1,0,0),(1,0,1)),((.2,0,0),(1,0,1),(.2,0,1))]
        return SurfaceDoorFollower([dict(name='left',pivot=(0,0,0),handle=(1,0,.9),sign=1,surface_faces=faces),
            dict(name='right',pivot=(3,0,0),handle=(4,0,.9),sign=-1,surface_faces=[tuple((p[0]+3,p[1],p[2]) for p in face) for face in faces])],roller_radius=.025)
    def rotate(self,p,a):
        return (math.cos(a)*p[0]-math.sin(a)*p[1],math.sin(a)*p[0]+math.cos(a)*p[1],p[2])
    def test_hook_to_roller_handoff_keeps_angle_and_pushes_further(self):
        f=self.follower();f.update_contacts((1,0,.9),None,0)
        for i in range(1,8):f.update_contacts(self.rotate((1,0,.9),i*.05),None,i*.1)
        self.assertAlmostEqual(f.angles['left'],.35)
        # Contact a different point and height, not the old fixed frame target.
        f.update_contacts(None,self.rotate((.7,-.025,.5),.35),.8)
        self.assertEqual(f.contact_mode,'push')
        f.update_contacts(None,self.rotate((.7,-.025,.5),.40),.9)
        self.assertGreater(f.angles['left'],.39)
        self.assertEqual(f.angles['right'],0)
        self.assertGreater(f.angular_velocity,0)
        self.assertLessEqual(f.penetration,.0011)
    def test_swept_contact_catches_crossing_between_samples(self):
        f=self.follower();f.update_contacts(None,(.7,-.045,.6),0)
        f.update_contacts(None,(.7,.045,.6),.1)
        self.assertGreater(f.angles['left'],0)
        self.assertEqual(f.contact_mode,'push')
    def test_retreat_and_radial_slide_do_not_pull_or_open(self):
        f=self.follower();f.update_contacts(None,(.7,-.025,.5),0)
        f.update_contacts(None,(.6,-.025,.5),.1)
        self.assertAlmostEqual(f.angles['left'],0)
        f.update_contacts(None,(.6,-.1,.5),.2)
        self.assertAlmostEqual(f.angles['left'],0)
        self.assertIsNone(f.active)
    def test_no_surface_contact_and_telemetry_jump_do_not_open(self):
        f=self.follower();f.update_contacts(None,(.7,-.1,2),0)
        f.update_contacts(None,(.7,.1,2),.1)
        self.assertEqual(f.angles['left'],0)
        f.update_contacts(None,(.7,-.025,.5),1)
        f.update_contacts(None,(.7,.025,.5),2)
        self.assertEqual(f.angles['left'],0)
    def test_mirrored_hinge_pushes_in_opposite_world_direction(self):
        f=self.follower()
        f.update_contacts(None,(3.7,.025,.5),0)
        tip=self.rotate((.7,.025,.5),-.05)
        f.update_contacts(None,(tip[0]+3,tip[1],tip[2]),.1)
        self.assertGreater(f.angles['right'],.04)
        self.assertEqual(f.angles['left'],0)

    def test_opening_bound_is_preserved(self):
        f=self.follower();f.angles['left']=f.max_angle
        f.update_contacts(None,self.rotate((.7,-.025,.5),f.max_angle),0)
        f.update_contacts(None,self.rotate((.7,-.025,.5),f.max_angle+.03),.1)
        self.assertEqual(f.angles['left'],f.max_angle)


if __name__=='__main__':unittest.main()
