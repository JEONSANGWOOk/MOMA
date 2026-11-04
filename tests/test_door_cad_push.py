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
