import math
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from seer_control.door_teaching import hook_contact_offset,handle_center,place_handle,reference,handle_contact_anchor
from seer_control.geometry3d import transform,multiply,point,load_glb_robot

class HookTeachingTests(unittest.TestCase):
    def test_tool_tcp_matches_measured_sdk_feedback(self):
        path=Path(__file__).resolve().parents[1]/'models/extctrl_linux/FR5_Assembly.glb'
        arm=load_glb_robot(path,max_faces=150)
        q=[-18.113685129864024,-73.81495933703061,-106.61398295413682,-89.36176813808734,-89.96302902998536,-25.00079461400641]
        joints={j['name']:math.radians(v) for j,v in zip(arm.movable(),q)}
        tcp=point(arm.link_transforms(joints)['J6'],hook_contact_offset())
        expected=(.4220186557769778,-.3202983983229773,.705972521047797)
        self.assertLess(math.dist(tcp,expected),1e-6)

    def test_grasp_target_registers_handle_in_xyz_at_any_heading(self):
        face=((-0.045,-.844,.363),(-.006,-.832,.469),(-.045,-.832,.469))
        part={'asset':NS(faces=[face],face_materials=[36])}
        center=handle_center(part)
        origin=[12,-20,.79,0,0,0]
        marker_frame=((1,0,0,.4),(0,0,1,-.2),(0,-1,0,1.2),(0,0,0,1))
        for yaw in (0,.4,-1.2):
            marker=multiply(transform((1,2,0),(0,0,yaw)),marker_frame)
            for scale in (1.,2.):
                result=place_handle(marker,part,origin,scale)
                cad_point=point(transform(result[:3],result[3:]),tuple(v*scale for v in handle_contact_anchor(part,scale)))
                target=point(multiply(marker,reference()['marker_grasp_tcp']),(0,0,0))
                self.assertLess(math.dist(cad_point,target),1e-12)
        self.assertEqual(origin,[12,-20,.79,0,0,0])

    def test_controller_export_records_early_return_and_native_rotation(self):
        data=reference()
        self.assertTrue(data['main5_returns_before_open'])
        # SDK EulerZYX(180,0,0) is Rz(180), not Rx(180).
        self.assertAlmostEqual(data['marker_grasp_tcp'][2][2],1.)
        self.assertAlmostEqual(data['marker_grasp_tcp'][0][0],-1.)
        self.assertEqual(set(data['source_sha256']),{'scenario.py','scenario_data.py','camera.py'})

class TaughtHingeTests(unittest.TestCase):
    def test_hinge_uses_teaching_without_moving_equipment(self):
        from seer_control.door_teaching import apply_taught_hinge
        face=((-0.045,-.844,.363),(-.006,-.832,.469),(-.045,-.832,.469))
        part={'name':'door:1','asset':NS(faces=[face],face_materials=[36]),'pivot':(-.3259,-.854,0)}
        lod={'name':'door:1','pivot':part['pivot']}
        view=NS(cad_asset=NS(name='SIMPLE_WHOLE.glb',door_parts=[part],lod_door_parts=[lod]),scales={'cad':1},cad_origin=[1,2,3,0,0,.2])
        original=list(view.cad_origin);self.assertTrue(apply_taught_hinge(view))
        center=handle_contact_anchor(part);delta=reference()['cad_left_hinge_from_handle']
        self.assertAlmostEqual(part['pivot'][0]-center[0],delta[0]);self.assertAlmostEqual(part['pivot'][1]-center[1],delta[1])
        self.assertEqual(lod['pivot'],part['pivot'])
        self.assertFalse(apply_taught_hinge(view));self.assertEqual(view.cad_origin,original)

class PendingProfileTests(unittest.TestCase):
    def test_pending_candidate_survives_old_profile_and_is_consumed_once(self):
        import tempfile,json
        from seer_control.door_teaching import apply_pending_profile
        config=dict(mount=[0]*6,assets={'cad':'same'},cad_origin=[99]*6)
        patch=dict(cad_origin=[1,2,3,0,0,.1],marker_placement={'mode':'hook_teaching','teaching_sha256':reference()['source_sha256']})
        data=dict(format='moma-pending-hook-alignment-v1',expected_mount=config['mount'],expected_assets=config['assets'],patch=patch)
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'pending_hook_alignment.json';path.write_text(json.dumps(data),encoding='utf-8')
            result=apply_pending_profile(config,temp)
            self.assertEqual(result['cad_origin'],[1,2,3,0,0,.1]);self.assertEqual(config['cad_origin'],[99]*6)
            self.assertFalse(path.exists());self.assertTrue(path.with_name('applied_hook_alignment.json').exists())
            self.assertEqual(apply_pending_profile(config,temp),config)

    def test_model_change_does_not_consume_pending_alignment(self):
        import tempfile,json
        from seer_control.door_teaching import apply_pending_profile
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'pending_hook_alignment.json';path.write_text(json.dumps(dict(format='moma-pending-hook-alignment-v1',expected_mount=[0]*6,expected_assets={})),encoding='utf-8')
            with self.assertRaises(ValueError):apply_pending_profile(dict(mount=[1]*6,assets={}),temp)
            self.assertTrue(path.exists())

if __name__=='__main__':unittest.main()

class HandleContactDepthTests(unittest.TestCase):
    def test_anchor_is_ahead_of_handle_surface_with_physical_scale_invariant_gap(self):
        part={'asset':NS(faces=[((-.04,-.844,.36),(-.006,-.832,.47),(-.04,-.832,.47))],face_materials=[36])}
        c=handle_center(part)
        for scale in (1.,2.):
            anchor=handle_contact_anchor(part,scale)
            self.assertEqual((anchor[0],anchor[2]),(c[0],c[2]))
            self.assertAlmostEqual((-.844-anchor[1])*scale,.007955389183358362)
    def test_depth_change_moves_equipment_away_from_hook_at_rotated_heading(self):
        part={'asset':NS(faces=[((-.04,-.844,.36),(-.006,-.832,.47),(-.04,-.832,.47))],face_materials=[36])}
        marker=((1,0,0,.4),(0,0,1,-.2),(0,-1,0,1.2),(0,0,0,1))
        for yaw in (0,.4,-1.2):
            m=multiply(transform((1,2,0),(0,0,yaw)),marker)
            new=place_handle(m,part,[0,0,0,0,0,0]);center=handle_center(part)
            target=point(multiply(m,reference()['marker_grasp_tcp']),(0,0,0))
            old=[target[0]-math.cos(yaw)*center[0]+math.sin(yaw)*center[1],target[1]-math.sin(yaw)*center[0]-math.cos(yaw)*center[1]]
            delta=(new[0]-old[0],new[1]-old[1])
            depth=(center[1]-handle_contact_anchor(part)[1])
            self.assertAlmostEqual(delta[0],-math.sin(yaw)*depth)
            self.assertAlmostEqual(delta[1],math.cos(yaw)*depth)
