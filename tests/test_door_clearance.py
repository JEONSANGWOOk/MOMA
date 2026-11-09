import math
import unittest
from types import SimpleNamespace as NS
from seer_control.door_clearance import DoorClearance,triangles_hit_box
from seer_control.geometry3d import transform

class DoorClearanceTests(unittest.TestCase):
    def test_triangle_crossing_box_and_separated_surface(self):
        self.assertTrue(triangles_hit_box([[(-2,0,0),(2,0,0),(0,2,0)]],(.1,.1,.1)))
        self.assertFalse(triangles_hit_box([[(-2,0,.2),(2,0,.2),(0,2,.2)]],(.1,.1,.1)))
        # AABB overlaps but the triangle itself misses the box.
        self.assertFalse(triangles_hit_box([[(.09,2,0),(2,.09,0),(2,2,0)]],(.1,.1,.1)))
    def door(self):
        return DoorClearance(dict(asset=NS(faces=[((0,0,0),(1,0,0),(1,0,1)),((0,0,0),(1,0,1),(0,0,1))]),pivot=(0,0,0),sign=1))
    def test_resolve_opens_door_without_moving_robot_box(self):
        door=self.door();boxes=[('tool',transform((.7,0,.5)),(.1,.1,.1))]
        original=repr(boxes)
        angle,hits=door.resolve(0,boxes)
        self.assertGreater(angle,0);self.assertFalse(hits)
        self.assertFalse(door.collisions(angle,boxes));self.assertEqual(repr(boxes),original)
    def test_no_solution_reports_collision_and_preserves_angle(self):
        door=self.door();boxes=[('arm',transform((0,0,.5)),(4,4,4))]
        angle,hits=door.resolve(.2,boxes,maximum=.3)
        self.assertEqual(angle,.2);self.assertEqual(hits,['arm'])
    def test_clear_pose_never_moves_door(self):
        door=self.door();angle,hits=door.resolve(.2,[('tool',transform((5,0,0)),(.1,.1,.1))])
        self.assertEqual(angle,.2);self.assertFalse(hits)

    def test_roller_target_160_retains_clearance_extension_and_closes(self):
        from seer_control.door_io7 import DoorIO7
        from seer_control.door_feedback import SurfaceDoorFollower
        f=SurfaceDoorFollower([dict(name='left:1',pivot=(0,0,0),sign=1,handle=(9,9,9))])
        gate=DoorIO7(roller_target=160);gate.phase='roller';gate.engaged=True
        gate.rotation=transform();gate.stamp=0.;gate.start=math.radians(60)
        f.angles['left:1']=math.radians(140)
        gate.update(f,None,None,transform(rpy=(0,0,.1)),.1,True,'roller')
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),140)
        gate.update(f,None,None,transform(rpy=(0,0,math.radians(100))),.2,True,'roller')
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),160)
        gate.update(f,None,None,transform(),.3,False,'roller')
        gate.update(f,None,None,transform(),.4,True,'roller')
        gate.update(f,None,None,transform(),.5,False,'roller')
        for i in range(60):gate.update(f,None,None,None,.6+i*.1,False,None)
        self.assertEqual(f.angles['left:1'],0.)

    def test_closing_tracks_each_rotation_without_contact_until_off(self):
        from seer_control.door_io7 import DoorIO7
        from seer_control.door_feedback import SurfaceDoorFollower
        for sign in (-1,1):
            f=SurfaceDoorFollower([dict(name='left:1',pivot=(0,0,0),sign=sign,handle=(9,9,9))])
            f.angles['left:1']=math.radians(160)
            gate=DoorIO7(roller_target=160,closing_motion=150);gate.phase='open_hold'
            for i,rotation in enumerate((0,30,60,90,120,150)):
                gate.update(f,(9,9,9),(9,9,9),transform(rpy=(0,0,-sign*math.radians(rotation))),i*.1,True,'roller')
                self.assertAlmostEqual(math.degrees(f.angles['left:1']),160*(1-rotation/150),places=7)
            self.assertEqual(gate.phase,'closing')
            self.assertAlmostEqual(f.angles['left:1'],0.)

    def test_closing_missing_feedback_and_reverse_do_not_jump_or_reopen(self):
        from seer_control.door_io7 import DoorIO7
        from seer_control.door_feedback import SurfaceDoorFollower
        f=SurfaceDoorFollower([dict(name='left:1',pivot=(0,0,0),sign=1,handle=(9,9,9))])
        f.angles['left:1']=math.radians(160)
        gate=DoorIO7(roller_target=160);gate.phase='open_hold'
        def step(deg,t,on=True):gate.update(f,None,None,transform(rpy=(0,0,math.radians(deg))),t,on,None)
        step(0,0);step(-30,.1);before=f.angles['left:1']
        step(-45,.2,None);self.assertEqual(f.angles['left:1'],before)
        step(-20,.3);self.assertEqual(f.angles['left:1'],before)
        step(-60,.4);self.assertAlmostEqual(math.degrees(f.angles['left:1']),96)
        step(-60,.5,False);self.assertEqual(gate.phase,'auto_close')
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),96)

    def test_measured_rotation_does_not_scale_to_target_or_snap_on_off(self):
        from seer_control.door_io7 import DoorIO7
        from seer_control.door_feedback import SurfaceDoorFollower
        f=SurfaceDoorFollower([dict(name='left:1',pivot=(0,0,0),sign=1,handle=(9,9,9))])
        gate=DoorIO7(hook_motion=59.6,roller_motion=76.3,roller_target=135.9,measured_rotation=True)
        gate.update(f,None,None,transform(),0,True,None)
        gate.update(f,None,None,transform(rpy=(0,0,math.radians(30))),.1,True,None)
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),30)
        gate.update(f,None,None,transform(rpy=(0,0,math.radians(59))),.2,True,None)
        gate.update(f,None,None,transform(),.3,False,None)
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),59)
        gate.phase='roller';gate.engaged=True;gate.start=f.angles['left:1'];gate.progress=0.;gate.rotation=transform()
        gate.update(f,None,None,transform(rpy=(0,0,math.radians(20))),.4,True,None)
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),79)
        gate.update(f,None,None,transform(rpy=(0,0,math.radians(90))),.5,True,None)
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),135.9)

    def test_measured_closing_speed_is_not_scaled_and_off_finish_is_bounded(self):
        from seer_control.door_io7 import DoorIO7
        from seer_control.door_feedback import SurfaceDoorFollower
        f=SurfaceDoorFollower([dict(name='left:1',pivot=(0,0,0),sign=1,handle=(9,9,9))])
        f.angles['left:1']=math.radians(135.9)
        gate=DoorIO7(measured_rotation=True,closing_speed=30);gate.phase='open_hold'
        gate.update(f,None,None,transform(),0,True,None)
        gate.update(f,None,None,transform(rpy=(0,0,-math.radians(30))),.1,True,None)
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),105.9)
        gate.update(f,None,None,transform(),.2,False,None)
        # Smoothstep peak speed = 1.5 * remaining angle / duration.
        self.assertLessEqual(1.5*math.degrees(gate.close_from)/gate.close_duration,30+1e-8)

    def test_surface_closing_stays_at_roller_radius_without_wrist_rotation(self):
        from seer_control.door_io7 import DoorIO7
        from seer_control.door_feedback import SurfaceDoorFollower
        from seer_control.geometry3d import point
        faces=[((0,0,0),(1,0,0),(1,0,1)),((0,0,0),(1,0,1),(0,0,1))]
        f=SurfaceDoorFollower([dict(name='left:1',pivot=(0,0,0),sign=1,handle=(9,9,9),surface_faces=faces)])
        f.angles['left:1']=math.radians(90)
        gate=DoorIO7(measured_rotation=True);gate.phase='open_hold'
        for i,deg in enumerate(range(90,29,-3)):
            roller=point(transform(rpy=(0,0,math.radians(deg))),(.7,-f.roller_radius,.5))
            gate.update(f,None,roller,transform(),i*.1,True,'roller')
            self.assertAlmostEqual(math.degrees(f.angles['left:1']),deg,places=5)
            near=f._nearest('left:1',roller,f.angles['left:1'],.1)
            self.assertAlmostEqual(near[0],f.roller_radius,places=6)

    def test_surface_opening_does_not_jump_with_wrist_rotation_or_bad_position(self):
        from seer_control.door_io7 import DoorIO7
        from seer_control.door_feedback import SurfaceDoorFollower
        f=SurfaceDoorFollower([dict(name='left:1',pivot=(0,0,0),sign=1,handle=(9,9,9))])
        f.angles['left:1']=math.radians(60)
        gate=DoorIO7(measured_rotation=True);gate.phase='roller';gate.engaged=True
        gate.roller_position=(.5,.5,.5);gate.roller_time=0
        gate.update(f,None,(.5,.5,.5),transform(rpy=(0,0,2)),.1,True,'roller')
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),60)
        gate.update(f,None,(-.5,.5,.5),transform(),.2,True,'roller')
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),60)

    def test_absolute_closing_plane_waits_for_contact_and_tracks_fast_motion(self):
        from seer_control.door_io7 import DoorIO7
        from seer_control.door_feedback import SurfaceDoorFollower
        from seer_control.geometry3d import point
        for sign in (-1,1):
            f=SurfaceDoorFollower([dict(name='left:1',pivot=(0,0,0),sign=sign,handle=(9,9,9),roller_plane_y=0.)])
            f.angles['left:1']=math.radians(120)
            gate=DoorIO7(measured_rotation=True,closing_speed=20);gate.phase='open_hold'
            # Roller starts outside the door, so the leaf holds until reached.
            for i,deg in enumerate(range(140,19,-10)):
                roller=point(transform(rpy=(0,0,sign*math.radians(deg))),(.7,-f.roller_radius,.5))
                gate.update(f,None,roller,transform(),i*.1,True,'roller')
                self.assertAlmostEqual(math.degrees(f.angles['left:1']),min(120,deg),places=5)

    def test_hook_uses_hook_travel_instead_of_wrist_rotation(self):
        from seer_control.door_io7 import DoorIO7
        from seer_control.door_feedback import SurfaceDoorFollower
        from seer_control.geometry3d import point
        f=SurfaceDoorFollower([dict(name='left:1',pivot=(0,0,0),sign=1,handle=(9,9,9))]);g=DoorIO7(measured_rotation=True)
        g.update(f,(.3,0,.4),None,transform(),0,True,'hook')
        hook=point(transform(rpy=(0,0,math.radians(10))),(.3,0,.4))
        g.update(f,hook,None,transform(rpy=(0,0,math.radians(40))),.1,True,'hook')
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),10)

    def test_second_pulse_acquires_plane_when_finite_mesh_contact_is_missing(self):
        from seer_control.door_io7 import DoorIO7
        from seer_control.door_feedback import SurfaceDoorFollower
        from seer_control.geometry3d import point
        f=SurfaceDoorFollower([dict(name='left:1',pivot=(0,0,0),sign=1,handle=(9,9,9),roller_plane_y=0.)]);f.angles['left:1']=math.radians(58)
        g=DoorIO7(measured_rotation=True,roller_target=135.9);g.phase='handoff'
        for i,deg in enumerate((35,45,58,60,70,80,90,100)):
            roller=point(transform(rpy=(0,0,math.radians(deg))),(.35,f.roller_radius,.4))
            g.update(f,None,roller,transform(),i*.1,True,'roller')
            if deg<58:self.assertAlmostEqual(math.degrees(f.angles['left:1']),58)
        self.assertTrue(g.engaged)
        self.assertAlmostEqual(math.degrees(f.angles['left:1']),100,places=5)
