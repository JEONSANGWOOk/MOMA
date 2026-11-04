import json
import math
import unittest
from types import SimpleNamespace as NS
from seer_control.equipment_placement import section,placed,quality,refine,signature,confirmed
from seer_control.world3d import WorldView3D


class PlacementTests(unittest.TestCase):
    def test_section_uses_scan_height_not_whole_mesh_bounds(self):
        side=((0,0,0),(2,0,0),(2,0,1),(0,0,1))
        other=((0,1,0),(2,1,0),(2,1,1),(0,1,1))
        roof=((50,50,2),(51,50,2),(50,51,2))
        hull=section(NS(faces=[side,other,roof]),[0]*6,1.)
        self.assertEqual(set(hull),{(0.,0.),(2.,0.),(2.,1.),(0.,1.)})

    def test_local_fit_improves_known_outline_pose(self):
        outline=[(-1,-.5),(1,-.5),(1,.5),(-1,.5)]
        dense=[]
        for a,b in zip(outline,outline[1:]+outline[:1]):
            dense.extend((a[0]+(b[0]-a[0])*i/30,a[1]+(b[1]-a[1])*i/30) for i in range(30))
        actual=(3.,2.,.2);cloud=placed(dense,actual)
        initial=(3.1,1.9,.25)
        before=quality(outline,cloud,initial)
        fit=refine(outline,cloud,initial);after=quality(outline,cloud,fit)
        self.assertLess(after['error'],before['error'])
        self.assertTrue(after['ok'])
        self.assertLess(math.dist(fit[:2],actual[:2]),.05)
        self.assertFalse(quality(outline,[],fit)['ok'])
        self.assertFalse(quality(outline,[(100,100)]*20,fit)['ok'])

    def test_confirmation_survives_save_and_invalidates_on_pose_or_map_changes(self):
        app=NS(host=NS(get=lambda:'192.168.30.13'),current_robot_map='map1',map=NS(name='map1'),
               lidar_map_alignment=(0,0,0),lidar_mount_overrides=[(.1,.2,.3)],spatial_assets={})
        view=NS(cad_origin=[0]*6,scales={'cad':1.},cad_asset=NS(name='machine',faces=[1]),cad_placement_confirmation=None)
        self.assertFalse(confirmed(app,view))
        view.cad_placement_confirmation=json.loads(json.dumps(dict(signature=signature(app,view),source='real_lidar_operator_confirmed')))
        self.assertTrue(confirmed(app,view))
        view.cad_origin[0]=.01
        self.assertFalse(confirmed(app,view))
        view.cad_origin[0]=0;app.current_robot_map='map2'
        self.assertFalse(confirmed(app,view))

    def test_unconfirmed_placement_blocks_passive_door_tracking(self):
        follower=NS(detach=lambda:None)
        view=NS(app=NS(real=True),placement_confirmed=lambda:False,_real_door_follower=follower)
        WorldView3D._update_real_doors(view,None,{}, {})
        self.assertIn('LiDAR',view.real_door_status)


if __name__=='__main__':unittest.main()
