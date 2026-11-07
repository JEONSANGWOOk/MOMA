import asyncio
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import AsyncMock, patch
from fastapi.testclient import TestClient
from web_control import server
from web_control.server import Control, Command, Doors, create_app

class BackendTests(unittest.IsolatedAsyncioTestCase):
    async def test_stale_and_read_only_commands_never_send(self):
        c=Control();c.mode='read';c.connected=True;c.rx=time.monotonic()
        c.command_request=AsyncMock()
        with self.assertRaises(ValueError):await c.send(Command(name='lock_control'))
        c.mode='control';c.rx=0
        with self.assertRaises(ValueError):await c.send(Command(name='navigate',goal='LM1'))
        c.command_request.assert_not_awaited()
    async def test_uncertain_command_is_never_replayed(self):
        c=Control();c.mode='control';c.connected=True;c.rx=time.monotonic()
        c.state={'emergency':False,'blocked':False}
        c.command_request=AsyncMock(side_effect=TimeoutError('late ACK'))
        with self.assertRaises(TimeoutError):await c.send(Command(name='pause'))
        c.command_request.assert_awaited_once()
        self.assertEqual(c.command_state['status'],'UNCERTAIN')
    async def test_stop_does_not_wait_for_status_or_command_lock(self):
        c=Control();c.mode='control';c.host='127.0.0.1';c.rx=0
        c.command_request=AsyncMock(return_value={'ack':True})
        async with c.command_lock:await c.stop()
        self.assertEqual(c.command_request.await_count,2)
    async def test_real_navigation_requires_actual_map_and_arm_safe_pose(self):
        c=Control();c.mode='control';c.connected=True;c.rx=time.monotonic()
        c.state={'emergency':False,'blocked':False};c.command_request=AsyncMock()
        with self.assertRaisesRegex(ValueError,'지도'):await c.send(Command(name='navigate',goal='LM1'))
        c.map.smap_source={}
        with self.assertRaisesRegex(ValueError,'FR5'):await c.send(Command(name='navigate',goal='LM1'))
        c.command_request.assert_not_awaited()
    async def test_sim_navigation_and_do(self):
        c=Control();await c.send(Command(name='navigate',goal='LM2'))
        self.assertTrue(c.sim.route)
        await c.send(Command(name='set_do',channel=4,value=True))
        self.assertTrue(c.sim.do[4])
    async def test_mission_completion_needs_new_target_feedback(self):
        c=Control();c.mode='control';c.connected=True;c.rx=time.monotonic()
        c.mission={'status':'RUNNING','goals':['LM2'],'index':0,'awaiting':True,'sent_at':time.monotonic(),'rx_at_send':c.rx}
        c.map.nodes['LM2'].update(x=1.,y=2.,r=.5)
        c.state={'task':'COMPLETED','target':'LM2','x':1.,'y':2.,'theta':.5}
        await c.mission_tick();self.assertEqual(c.mission['status'],'RUNNING')
        c.rx+=.1;await c.mission_tick();self.assertEqual(c.mission['status'],'COMPLETED')
    async def test_completed_task_with_wrong_theta_does_not_advance(self):
        c=Control();c.mode='control';c.connected=True;c.rx=time.monotonic()
        c.map.nodes['LM2'].update(x=1.,y=2.,r=.5)
        c.mission={'status':'RUNNING','goals':['LM2'],'index':0,'awaiting':True,'sent_at':time.monotonic(),'rx_at_send':c.rx-.1}
        c.state={'task':'COMPLETED','target':'LM2','x':1.,'y':2.,'theta':.6}
        await c.mission_tick();self.assertEqual(c.mission['status'],'RUNNING')
        c.state['theta']=.5;c.rx+=.1
        await c.mission_tick();self.assertEqual(c.mission['status'],'COMPLETED')

    async def test_slow_command_does_not_starve_telemetry(self):
        c=Control();c.mode='control';c.connected=True;c.rx=time.monotonic()
        c.state={'emergency':False,'blocked':False}
        class StateClient:
            def snapshot(self):return ({'emergency':False,'blocked':False,'x':1,'y':2}, {})
        c.client=StateClient()
        async def slow(*args):await asyncio.sleep(.45);return {'ack':True}
        c.command_request=slow
        poll=asyncio.create_task(c.poll())
        before=c.rx
        try:
            command=asyncio.create_task(c.send(Command(name='pause')))
            await asyncio.sleep(.3)
            self.assertFalse(command.done())
            self.assertGreater(c.rx,before)
            await command
        finally:
            poll.cancel();await asyncio.gather(poll,return_exceptions=True)

    async def test_read_only_cannot_stop_physical_robot(self):
        c=Control();c.mode='read';c.command_request=AsyncMock()
        with self.assertRaises(ValueError):await c.stop()
        c.command_request.assert_not_awaited()

class EquipmentPlacementTests(unittest.TestCase):
    def test_confirmed_origin_overrides_unconfirmed_edit_and_survives_restart(self):
        import json
        origin=[-.254,2.117,.79,0,0,.0872664626]
        with tempfile.TemporaryDirectory() as directory:
            desktop=Path(directory)/'desktop.json'
            desktop.write_text(json.dumps({'viewer3d':{'cad_origin':[99,99,0,0,0,0],
                'cad_placement_confirmation':{'source':'real_lidar_operator_confirmed','signature':{'origin':origin}}}}),encoding='utf-8')
            with patch.object(server,'DESKTOP',desktop):
                first=Control();second=Control()
            self.assertEqual(first.viewer['cad_origin'],origin)
            self.assertEqual(second.viewer['cad_origin'],origin)
            self.assertEqual(json.loads(desktop.read_text())['viewer3d']['cad_origin'][0],99)
    def test_invalid_or_unconfirmed_record_does_not_supply_pose(self):
        pose=[1,2,3,0,0,.5]
        for record in ({'source':'draft','signature':{'origin':[9]*6}},
                       {'source':'real_lidar_operator_confirmed','signature':{'origin':[float('nan')]*6}}):
            restored=server.restore_equipment_placement({'cad_origin':pose,'cad_placement_confirmation':record})
            self.assertEqual(restored['cad_origin'],pose)


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.patch=patch.object(server,'SETTINGS',Path(self.tmp.name)/'settings.json');self.patch.start()
        self.control=Control();self.client=TestClient(create_app(self.control));self.h={'X-Moma-Session':'test-operator-session-001'}
    def tearDown(self):self.client.close();self.patch.stop();self.tmp.cleanup()
    def acquire(self):self.assertEqual(self.client.post('/api/control/acquire',headers=self.h).status_code,200)
    def test_mutation_needs_lease_and_lease_is_exclusive(self):
        self.assertEqual(self.client.post('/api/doors',json={'left':90,'right':0},headers=self.h).status_code,409)
        self.acquire()
        self.assertEqual(self.client.post('/api/control/acquire',headers={'X-Moma-Session':'another-operator-session'}).status_code,409)
    def test_independent_initial_doors_saved_and_validated(self):
        self.acquire();self.assertEqual(self.client.post('/api/doors',json={'left':90,'right':0,'follow':True},headers=self.h).status_code,200)
        self.assertEqual(self.control.doors,{'left':90,'right':0})
        self.assertEqual(server.read_json(server.SETTINGS,{})['doors']['left'],90)
        self.assertEqual(self.client.post('/api/doors',json={'left':121,'right':0},headers=self.h).status_code,422)
    def test_foreign_origin_is_rejected(self):
        r=self.client.post('/api/control/acquire',headers={**self.h,'Origin':'https://unexpected.example'})
        self.assertEqual(r.status_code,403)
    def test_expired_lease_cannot_write(self):
        self.acquire();self.control.owner_until=0
        self.assertEqual(self.client.post('/api/doors',json={'left':0,'right':0},headers=self.h).status_code,409)
    def test_raw_model_route_uses_original_glb_and_rejects_unknown_roles(self):
        self.assertEqual(self.client.get('/api/models/unknown.glb').status_code,404)
        response=self.client.get('/api/models/amr.glb',headers={'Range':'bytes=0-19'})
        self.assertEqual(response.status_code,206)
        self.assertEqual(response.content[:4],b'glTF')
        meta=self.control.scene_data['assets']['amr']
        self.assertEqual(meta['detail'],'full-source')
        self.assertNotIn('vertices',meta)
    def test_realsense_default_and_explicit_disable(self):
        self.assertEqual(self.control.camera_source,'realsense_remote')
        self.assertTrue(self.control.camera_url.endswith('/video_feed'))
        self.acquire()
        response=self.client.post('/api/camera',json={'source':'off'},headers=self.h)
        self.assertEqual(response.status_code,200)
        self.assertFalse(self.client.get('/api/status').json()['camera_enabled'])

    def test_webcam_selection_persistence_and_validation(self):
        self.acquire()
        response=self.client.post('/api/camera',json={'source':'webcam','device':1},headers=self.h)
        self.assertEqual(response.status_code,200)
        self.assertEqual(self.control.camera_url,'webcam:1')
        self.assertTrue(self.client.get('/api/status').json()['camera_enabled'])
        self.assertEqual(Control().camera_url,'webcam:1')
        self.assertEqual(self.client.post('/api/camera',json={'source':'webcam','device':-1},headers=self.h).status_code,422)
        self.client.post('/api/camera',json={'source':'off'},headers=self.h)
        self.assertEqual(self.control.camera.url,'')

    def test_realsense_stays_available_while_webcam_is_selected(self):
        self.acquire()
        hub=self.control.realsense
        hub.publish(b'realsense-jpeg','image/jpeg',hub.generation)
        self.client.post('/api/camera',json={'source':'webcam','device':0},headers=self.h)
        webcam=self.control.camera
        webcam.publish(b'webcam-jpeg','image/jpeg',webcam.generation)
        self.assertEqual(self.client.get('/api/camera/frame').content,b'webcam-jpeg')
        self.assertEqual(self.client.get('/api/camera/frame?source=realsense').content,b'realsense-jpeg')
        self.assertTrue(self.client.get('/api/status').json()['realsense']['connected'])
        self.client.post('/api/camera',json={'source':'off'},headers=self.h)
        self.assertEqual(self.client.get('/api/camera/frame?source=realsense').content,b'realsense-jpeg')
        self.client.post('/api/camera',json={'source':'realsense_remote'},headers=self.h)
        self.assertEqual(self.client.get('/api/camera/frame').content,b'realsense-jpeg')
        self.assertEqual(self.control.camera.url,'')
        self.assertEqual(self.client.get('/api/camera/frame?source=unknown').status_code,400)

    def test_status_does_not_mark_disconnected_sim_online(self):
        self.control.connected=False;self.assertFalse(self.client.get('/api/status').json()['connected'])

class WebcamCaptureTests(unittest.TestCase):
    def test_shared_capture_publishes_jpeg_and_releases_device(self):
        from unittest.mock import MagicMock
        import sys
        from web_control.camera import CameraHub
        hub=CameraHub('webcam:0')
        cv=MagicMock();capture=cv.VideoCapture.return_value
        capture.isOpened.return_value=True
        capture.read.return_value=(True,object())
        cv.imencode.return_value=(True,MagicMock(tobytes=lambda:b'jpeg-frame'))
        original=hub.publish
        def publish(*args):
            original(*args);hub.stop_event.set()
        with patch.dict(sys.modules,{'cv2':cv}),patch.object(hub,'publish',side_effect=publish):
            hub.capture_webcam('webcam:0',0)
        self.assertEqual(hub.latest()[0],b'jpeg-frame')
        capture.release.assert_called_once()

if __name__=='__main__':unittest.main()
