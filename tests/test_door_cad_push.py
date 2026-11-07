"""Verify both actual CAD leaves open under mirrored roller pushes."""
import math
import unittest
from pathlib import Path
from seer_control.geometry3d import load_glb
from seer_control.door_feedback import SurfaceDoorFollower, _sub, _dot

class CADRollerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.asset=load_glb(Path(__file__).resolve().parents[1]/'SIMPLE_WHOLE.glb',max_faces=12000)
    def test_both_leaf_hinges_open_from_closed_and_partly_open(self):
        for suffix in (':1',':2'):
            door=next(p for p in self.asset.door_parts if p['name'].endswith(suffix))
            faces=door['asset'].faces;pivot=door['pivot'];sign=door['sign'];contact=None
            for face in faces:
                c=tuple(sum(p[i] for p in face)/3 for i in range(3));r=_sub(c,pivot);radius=math.hypot(r[0],r[1])
                if radius<.15:continue
                u=_sub(face[1],face[0]);v=_sub(face[2],face[0])
                n=(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]);length=math.sqrt(_dot(n,n))
                if length<1e-12:continue
                n=tuple(x/length for x in n);tangent=(-r[1]*sign/radius,r[0]*sign/radius,0)
                if abs(_dot(n,tangent))<.9:continue
                if _dot(n,tangent)<0:n=tuple(-x for x in n)
                contact=c,n;break
            self.assertIsNotNone(contact)
            for initial in (0,35,80):
                with self.subTest(door=suffix,initial=initial):
                    c,n=contact
                    f=SurfaceDoorFollower([dict(name=door['name'],pivot=pivot,sign=sign,handle=c,surface_faces=faces)],roller_radius=.02245)
                    f.angles[door['name']]=math.radians(initial)
                    start=tuple(c[i]-n[i]*f.roller_radius for i in range(3));end=tuple(start[i]+n[i]*.012 for i in range(3))
                    rotation=math.radians(initial)*sign
                    start=f._rotate(door['name'],start,rotation);end=f._rotate(door['name'],end,rotation)
                    f.update_contacts(None,start,0);f.update_contacts(None,end,.1)
                    self.assertGreater(f.angles[door['name']],math.radians(initial))
                    self.assertEqual(f.contact_mode,'push')
                    self.assertLess(f.penetration,.0011)

class CADFrameHookTests(CADRollerTests):
    def test_right_frame_hook_tracks_arc_and_holds_on_release(self):
        door=next(p for p in self.asset.door_parts if p['name'].endswith(':2'))
        frame=[f for f,m in zip(door['asset'].faces,door['asset'].face_materials) if m==33]
        pivot=door['pivot'];name=door['name']
        contact=max((tuple(sum(p[i] for p in face)/3 for i in range(3)) for face in frame),
                    key=lambda p:math.hypot(p[0]-pivot[0],p[1]-pivot[1]))
        for initial in (0,35,80):
            with self.subTest(initial=initial):
                f=SurfaceDoorFollower([dict(name=name,pivot=pivot,sign=door['sign'],handle=(99,99,99),
                    surface_faces=door['asset'].faces,hook_faces=frame,hook_only=True)])
                f.angles[name]=math.radians(initial)
                tip=f._rotate(name,contact,math.radians(initial)*door['sign'])
                f.update_contacts(None,tip,0)
                self.assertEqual(f.angles[name],math.radians(initial))
                self.assertIsNone(f.active)
                f.update_contacts(tip,None,.1)
                self.assertEqual(f.active,name)
                for i in range(1,11):
                    tip=f._rotate(name,contact,math.radians(initial+i*2)*door['sign'])
                    f.update_contacts(tip,None,.1+i*.1)
                self.assertAlmostEqual(f.angles[name],math.radians(initial+20))
                self.assertEqual(f.contact_mode,'hook')
                # Retracting the pushing hook holds the door open.
                reverse=f._rotate(name,contact,math.radians(initial+18)*door['sign'])
                f.update_contacts(reverse,None,1.2)
                self.assertIsNone(f.active)
                self.assertAlmostEqual(f.angles[name],math.radians(initial+20))
                f.update_contacts(None,None,1.3)
                self.assertIsNone(f.active)
                self.assertAlmostEqual(f.angles[name],math.radians(initial+20))

class BothLeafTrajectoryTests(CADRollerTests):
    def test_either_leaf_tracks_hook_trajectory_without_moving_other_leaf(self):
        doors=[]
        for d in self.asset.door_parts:
            faces=[f for f,m in zip(d['asset'].faces,d['asset'].face_materials) if m in (33,36,37)]
            doors.append(dict(name=d['name'],pivot=d['pivot'],sign=d['sign'],handle=(99,99,99),
                surface_faces=d['asset'].faces,hook_faces=faces,hook_open_only=True))
        for selected in doors:
            for initial in (0,35,80):
                with self.subTest(door=selected['name'],initial=initial):
                    f=SurfaceDoorFollower(doors);name=selected['name'];pivot=selected['pivot']
                    f.angles[name]=math.radians(initial)
                    contact=max((tuple(sum(p[i] for p in face)/3 for i in range(3)) for face in selected['hook_faces']),
                        key=lambda p:math.hypot(p[0]-pivot[0],p[1]-pivot[1]))
                    f.update_contacts((99,99,99),None,0)
                    self.assertIsNone(f.active)
                    f.update_contacts(f._rotate(name,contact,math.radians(initial)*selected['sign']),None,.1)
                    self.assertEqual(f.active,name)
                    for i in range(1,11):
                        tip=f._rotate(name,contact,math.radians(initial+2*i)*selected['sign'])
                        f.update_contacts(tip,None,.1+i*.1)
                    self.assertAlmostEqual(f.angles[name],math.radians(initial+20))
                    for other in doors:
                        if other['name']!=name:self.assertEqual(f.angles[other['name']],0)
                    f.update_contacts(None,None,1.2)
                    self.assertIsNone(f.active)
                    self.assertAlmostEqual(f.angles[name],math.radians(initial+20))

class StopThenOpenTests(CADRollerTests):
    def test_right_hook_waits_for_stop_then_tracks_next_movement(self):
        d=next(p for p in self.asset.door_parts if p['name'].endswith(':2'))
        frame=[f for f,m in zip(d['asset'].faces,d['asset'].face_materials) if m in (33,36,37)]
        name=d['name'];pivot=d['pivot']
        tip=max((tuple(sum(p[i] for p in face)/3 for i in range(3)) for face in frame),
            key=lambda p:math.hypot(p[0]-pivot[0],p[1]-pivot[1]))
        f=SurfaceDoorFollower([dict(name=name,pivot=pivot,sign=d['sign'],handle=(99,99,99),
            surface_faces=d['asset'].faces,hook_faces=frame,hook_open_only=True,hook_dwell=.4)])
        for t in (0,.1,.2,.3):
            f.update_contacts(tip,None,t)
            self.assertIsNone(f.active)
            self.assertEqual(f.angles[name],0)
        f.update_contacts(tip,None,.41)
        self.assertEqual(f.active,name)
        self.assertEqual(f.angles[name],0)
        for i in range(1,11):
            f.update_contacts(f._rotate(name,tip,math.radians(i*2)*d['sign']),None,.41+i*.1)
        self.assertAlmostEqual(f.angles[name],math.radians(20))
        f.update_contacts(None,None,1.5)
        self.assertIsNone(f.active)
        self.assertAlmostEqual(f.angles[name],math.radians(20))
    def test_approach_motion_does_not_arm_right_hook(self):
        from seer_control.door_feedback import SurfaceDoorFollower
        face=((1,-.2,0),(1,.2,0),(1,0,2))
        f=SurfaceDoorFollower([dict(name='right',pivot=(0,0,0),sign=1,handle=(99,99,99),
            surface_faces=[face],hook_faces=[face],hook_open_only=True,hook_dwell=.4)])
        for i in range(10):
            f.update_contacts((1,-.04+i*.008,1),None,i*.1)
            self.assertIsNone(f.active)
            self.assertEqual(f.angles['right'],0)

if __name__=='__main__':unittest.main()

class RollerCloseTests(unittest.TestCase):
    def test_outer_push_closes_and_retreat_does_not_pull(self):
        faces=[((.1,0,0),(1,0,0),(1,0,1)),((.1,0,0),(1,0,1),(.1,0,1))]
        f=SurfaceDoorFollower([dict(name='left',pivot=(0,0,0),sign=1,handle=(9,9,9),surface_faces=faces,roller_close=True)],roller_radius=.025)
        angle=math.radians(45);f.angles['left']=angle
        start=f._rotate('left',(.7,.025,.5),angle)
        end=f._rotate('left',(.7,.013,.5),angle)
        f.update_contacts(None,start,0)
        f.update_contacts(None,end,.1)
        self.assertLess(f.angles['left'],angle)
        self.assertGreaterEqual(f.angles['left'],0)
        closed=f.angles['left']
        f.update_contacts(None,start,.2)
        self.assertAlmostEqual(f.angles['left'],closed)

    def test_opening_push_reverse_does_not_close(self):
        faces=[((.1,0,0),(1,0,0),(1,0,1)),((.1,0,0),(1,0,1),(.1,0,1))]
        f=SurfaceDoorFollower([dict(name='left',pivot=(0,0,0),sign=1,handle=(9,9,9),surface_faces=faces,roller_close=True)],roller_radius=.025)
        angle=math.radians(45);f.angles['left']=angle
        start=f._rotate('left',(.7,-.025,.5),angle)
        end=f._rotate('left',(.7,-.013,.5),angle)
        f.update_contacts(None,start,0);f.update_contacts(None,end,.1)
        opened=f.angles['left'];self.assertGreater(opened,angle)
        f.update_contacts(None,start,.2)
        self.assertAlmostEqual(f.angles['left'],opened)

    def test_roller_reaches_120_and_holds_when_retracted(self):
        faces=[((.1,0,0),(1,0,0),(1,0,1)),((.1,0,0),(1,0,1),(.1,0,1))]
        f=SurfaceDoorFollower([dict(name='left',pivot=(0,0,0),sign=1,handle=(9,9,9),surface_faces=faces,roller_close=True)],roller_radius=.025)
        f.angles['left']=math.radians(35)
        for i in range(86):
            p=f._rotate('left',(.7,-.025,.5),math.radians(35+i))
            f.update_contacts(None,p,i*.05)
        self.assertAlmostEqual(math.degrees(f.angles['left']),120.,places=5)
        for i in range(1,6):
            p=f._rotate('left',(.7,-.025-i*.01,.5),math.radians(120))
            f.update_contacts(None,p,4.25+i*.05)
        self.assertAlmostEqual(math.degrees(f.angles['left']),120.,places=5)

    def test_moving_roller_wins_over_nearby_hook_reacquisition(self):
        faces=[((.1,0,0),(1,0,0),(1,0,1)),((.1,0,0),(1,0,1),(.1,0,1))]
        f=SurfaceDoorFollower([dict(name='left',pivot=(0,0,0),sign=1,handle=(.2,0,.5),surface_faces=faces,hook_faces=faces,hook_open_only=True)],roller_radius=.025)
        f._roller_previous=(.7,-.025,.5);f._roller_stamp=0.
        f.update_contacts((.2,0,.5),(.7,-.013,.5),.1)
        self.assertGreater(f.angles['left'],0.)
        self.assertEqual(f.contact_mode,'push')

    def test_closing_follows_angle_after_surface_contact_is_lost(self):
        faces=[((.1,0,0),(1,0,0),(1,0,1)),((.1,0,0),(1,0,1),(.1,0,1))]
        f=SurfaceDoorFollower([dict(name='left',pivot=(0,0,0),sign=1,handle=(9,9,9),surface_faces=faces,roller_close=True)],roller_radius=.025)
        angle=math.radians(80);f.angles['left']=angle
        start=f._rotate('left',(.7,.025,.5),angle)
        end=f._rotate('left',(.7,.013,.5),angle)
        f.update_contacts(None,start,0);f.update_contacts(None,end,.1)
        self.assertEqual(f.contact_mode,'close-angle')
        # Deliberately move away radially; angular closing must still continue.
        for i in range(1,43):
            p=f._rotate('left',(.7+i*.002,.013,.5),angle-math.radians(2*i))
            f.update_contacts(None,p,.1+i*.05)
        self.assertAlmostEqual(f.angles['left'],0.,places=6)

    def test_closing_angle_stops_on_missing_sample(self):
        faces=[((.1,0,0),(1,0,0),(1,0,1))]
        f=SurfaceDoorFollower([dict(name='left',pivot=(0,0,0),sign=1,handle=(9,9,9),surface_faces=faces,roller_close=True)])
        f.angles['left']=1.;f.active='left';f.contact_mode='close-angle';f._closing_track={'name':'left','height':.5}
        f.update_contacts(None,None,1.)
        self.assertIsNone(f._closing_track)
        self.assertEqual(f.angles['left'],1.)

    def test_hook_alignment_tolerance_tracks_both_hinge_directions(self):
        faces=[((.1,0,0),(1,0,0),(1,0,1)),((.1,0,0),(1,0,1),(.1,0,1))]
        for sign in (-1,1):
            f=SurfaceDoorFollower([dict(name='door',pivot=(0,0,0),sign=sign,handle=(9,9,9),surface_faces=faces,hook_faces=faces,hook_open_only=True,hook_capture=.065,hook_release=.12)])
            f.update_contacts((.7,.055,.5),None,0.)
            self.assertEqual(f.active,'door')
            for i in range(1,16):
                tip=f._rotate('door',(.7,.055,.5),math.radians(i*2)*sign)
                f.update_contacts(tip,None,i*.05)
            self.assertAlmostEqual(math.degrees(f.angles['door']),30.)
            f.detach();f.angles['door']=0.
            f.update_contacts((.7,.25,.5),None,2.)
            self.assertIsNone(f.active)

    def test_hook_regrip_then_roller_opening_follows_with_gap(self):
        faces=[((.1,0,0),(1,0,0),(1,0,1)),((.1,0,0),(1,0,1),(.1,0,1))]
        f=SurfaceDoorFollower([dict(name='left',pivot=(0,0,0),sign=1,handle=(9,9,9),surface_faces=faces,hook_faces=faces,hook_open_only=True,roller_open_angle=True,roller_capture_gap=.04)],roller_radius=.025)
        # Hook opens to 35 degrees, then releases while the arm changes posture.
        f.update_contacts((.7,0,.5),None,0.)
        for i in range(1,8):f.update_contacts(f._rotate('left',(.7,0,.5),math.radians(5*i)),None,i*.1)
        held=f.angles['left'];f.update_contacts(None,None,.8)
        self.assertAlmostEqual(math.degrees(held),35.)
        f.update_contacts(None,f._rotate('left',(.7,-.055,.5),held),.9)
        f.update_contacts(None,f._rotate('left',(.7,-.045,.5),held),1.)
        self.assertEqual(f.contact_mode,'open-angle')
        for i in range(1,45):
            p=f._rotate('left',(.7+i*.002,-.045,.5),held+math.radians(2*i))
            f.update_contacts(None,p,1.+i*.05)
        self.assertAlmostEqual(math.degrees(f.angles['left']),120.)
        f.update_contacts(None,f._rotate('left',(.788,-.045,.5),math.radians(115)),3.25)
        self.assertAlmostEqual(math.degrees(f.angles['left']),120.)

    def test_reposition_cannot_open_until_roller_separates_and_reapproaches(self):
        faces=[((.1,0,0),(1,0,0),(1,0,1)),((.1,0,0),(1,0,1),(.1,0,1))]
        f=SurfaceDoorFollower([dict(name='left',pivot=(0,0,0),sign=1,handle=(9,9,9),surface_faces=faces,hook_faces=faces,hook_open_only=True,roller_open_angle=True,roller_capture_gap=.04,roller_after_hook_release=True)],roller_radius=.025)
        f.update_contacts((.7,0,.5),None,0.)
        for i in range(1,8):f.update_contacts(f._rotate('left',(.7,0,.5),math.radians(5*i)),None,i*.1)
        opened=f.angles['left']
        position=lambda y:f._rotate('left',(.7,y,.5),opened)
        # Roller moving near the leaf during regrip must not drag it.
        f.update_contacts(None,position(-.035),.8)
        f.update_contacts(None,position(-.026),.9)
        self.assertAlmostEqual(f.angles['left'],opened)
        self.assertEqual(f._roller_gate['left'],'clear')
        # Only a separate approach after visible clearance starts roller opening.
        f.update_contacts(None,position(-.12),1.)
        self.assertEqual(f._roller_gate['left'],'ready')
        self.assertAlmostEqual(f.angles['left'],opened)
        f.update_contacts(None,position(-.026),1.1)
        self.assertGreater(f.angles['left'],opened)
        self.assertEqual(f.contact_mode,'open-angle')
