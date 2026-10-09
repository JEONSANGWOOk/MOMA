"""Protocol and independent simulated controller regressions; no hardware."""
import copy
import json
import math
import socket
import time
import unittest

from seer_control.live import SeerClient
from seer_control.model import MapModel
from seer_control.robot_emulator import VirtualRobot,EmulatorServer,PORT_APIS
from seer_control.smap import load_smap_data
from seer_control.transport import HEADER,PersistentSeerSession,receive_exact


def model():
    return MapModel(dict(format='amr-console-map-v1',nodes=[dict(id=k,x=x,y=y) for k,x,y in
        [('A',0,0),('B',2,0),('C',0,2),('D',2,2)]],
        edges=[['A','B'],['A','C'],['C','D'],['D','B']],walls=[[-2,-2,4,-2],[4,-2,4,4],[4,4,-2,4],[-2,4,-2,-2]]))


class VirtualRobotTests(unittest.TestCase):
    def setUp(self):self.robot=VirtualRobot(model());self.robot.request(4005,dict(nick_name='test'))
    def tearDown(self):self.robot.close()
    def steps(self,count=500):
        for _ in range(count):self.robot.step(.1)
    def test_moves_then_reports_completed_position_and_speed(self):
        self.assertEqual(self.robot.request(3051,dict(id='B'))['ret_code'],0)
        self.steps(5);self.assertGreater(self.robot.status()['x'],0);self.assertEqual(self.robot.task_status,2)
        self.steps();s=self.robot.status()
        self.assertEqual(s['task_status'],4);self.assertAlmostEqual(s['x'],2,places=2);self.assertEqual(s['vx'],0)
    def test_pause_resume_cancel(self):
        self.robot.request(3051,dict(id='B'));self.steps(10)
        self.assertEqual(self.robot.request(3001)['ret_code'],0)
        pose=self.robot.status()['x'];self.steps(20);self.assertEqual(self.robot.status()['x'],pose)
        self.assertEqual(self.robot.task_status,3)
        self.assertEqual(self.robot.request(3002)['ret_code'],0);self.steps(5)
        self.assertGreater(self.robot.status()['x'],pose)
        self.robot.request(3003);self.steps(20)
        self.assertEqual(self.robot.task_status,6);self.assertFalse(self.robot.sim.route)
    def test_invalid_navigation_does_not_replace_active_task(self):
        self.robot.request(3051,dict(id='B'))
        self.assertNotEqual(self.robot.request(3051,dict(id='D'))['ret_code'],0)
        self.assertEqual(self.robot.sim.state.target,'B')
    def test_no_unsupported_api_success_or_wrong_port(self):
        self.assertNotEqual(self.robot.request(6100)['ret_code'],0)
        self.assertNotEqual(self.robot.request(3051,dict(id='B'),19204)['ret_code'],0)
        self.assertNotEqual(self.robot.request(3051,dict(id='missing'))['ret_code'],0)
    def test_control_ownership_and_emergency(self):
        self.assertNotEqual(self.robot.request(4005,dict(nick_name='other'))['ret_code'],0)
        self.assertNotEqual(self.robot.request(4006,dict(nick_name='other'))['ret_code'],0)
        self.robot.inject('emergency');self.assertTrue(self.robot.status()['emergency'])
        self.assertNotEqual(self.robot.request(3051,dict(id='B'))['ret_code'],0)
        self.robot.inject('emergency');self.robot.request(4006,dict(nick_name='test'))
        self.assertNotEqual(self.robot.request(3051,dict(id='B'))['ret_code'],0)
    def test_blocked_clear_and_fail_injection(self):
        self.robot.request(3051,dict(id='B'));self.steps(4);self.robot.inject('blocked')
        pose=self.robot.status()['x'];self.steps(10)
        self.assertEqual(self.robot.status()['x'],pose);self.assertIn('block_x',self.robot.status())
        self.robot.inject('blocked');self.steps();self.assertEqual(self.robot.task_status,4)
        self.robot.inject('failure');self.steps();self.assertEqual(self.robot.task_status,5)
        self.assertTrue(self.robot.status()['errors'])
    def test_physical_obstacle_wait_then_clear(self):
        self.robot.inject('obstacle',(1,0));self.robot.request(3051,dict(id='B'));self.steps(80)
        s=self.robot.status();self.assertTrue(s['blocked']);self.assertLess(s['x'],1);self.assertEqual(s['task_status'],2)
        self.robot.inject('clear_obstacles');self.steps();self.assertEqual(self.robot.task_status,4)
    def test_free_navigation_reroutes_after_cancel(self):
        self.robot.inject('obstacle',(1,0));self.robot.request(3051,dict(id='B'));self.steps(40)
        self.assertTrue(self.robot.status()['blocked'])
        self.robot.request(3003);self.assertEqual(self.robot.request(3050,dict(id='B'))['ret_code'],0)
        self.steps(1000);self.assertEqual(self.robot.task_status,4);self.assertAlmostEqual(self.robot.status()['x'],2,places=2)
    def test_jog_manual_mode_and_lease_expiration(self):
        self.assertNotEqual(self.robot.request(2010,dict(vx=.2))['ret_code'],0)
        self.robot.request(4000,dict(mode=0));self.robot.request(2010,dict(vx=.2,w=0))
        self.robot.step(.1);self.assertGreater(self.robot.status()['x'],0)
        self.steps(20);pose=self.robot.status()['x'];self.steps(20)
        self.assertEqual(self.robot.status()['x'],pose);self.assertEqual(self.robot.status()['vx'],0)
    def test_map_download_and_upload_roundtrip(self):
        data=self.robot.request(4011,dict(map_name='emulator_demo'))
        imported=load_smap_data(data)
        self.assertEqual(imported.route('A','B'),['A','B']);self.assertTrue(imported.cloud)
        data['advancedPointList'][1]['pos']['x']=2.5
        self.assertEqual(self.robot.request(4010,data)['ret_code'],0)
        self.assertEqual(self.robot.request(1301)['stations'][1]['x'],2.5)
        self.assertNotEqual(self.robot.request(4011,dict(map_name='missing'))['ret_code'],0)
    def test_relocate_atomic_validation(self):
        before=self.robot.status();response=self.robot.request(2002,dict(x=1,y=1,angle=float('nan')))
        self.assertNotEqual(response['ret_code'],0);self.assertEqual(self.robot.status()['x'],before['x'])
        self.assertEqual(self.robot.request(2002,dict(x=0,y=2,angle=.2))['ret_code'],0)
        self.assertEqual(self.robot.status()['last_station'],'C')


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.robot=VirtualRobot(model())
        self.server=EmulatorServer(self.robot,ports={p:0 for p in PORT_APIS}).start(physics=False)
    def tearDown(self):self.server.close();self.robot.close()
    def client(self,port):return PersistentSeerSession('127.0.0.1',self.server.bound_ports[port],timeout=.5)
    def test_persistent_session_multiple_requests(self):
        client=self.client(19204)
        try:
            for api in (1100,1101,1000,1002,1007,1009,1020,1021,1022,1300,1301):
                r=client.request(api);self.assertEqual(r['_response_type'],api+10000)
            self.assertEqual(client.request(1100)['robot_model'],'LAN Virtual AMR')
        finally:client.close()
    def test_fragmented_and_back_to_back_frames(self):
        with socket.create_connection(('127.0.0.1',self.server.bound_ports[19204])) as sock:
            sock.settimeout(1)
            for seq in (1,2):
                packet=HEADER.pack(0x5a,1,seq,0,1100,b'\0'*6)
                sock.sendall(packet[:3]);sock.sendall(packet[3:])
            for seq in (1,2):
                h=HEADER.unpack(receive_exact(sock,16));data=json.loads(receive_exact(sock,h[3]))
                self.assertEqual(h[2],seq);self.assertEqual(h[4],11100);self.assertTrue(data['is_emulator'])
    def test_readonly_client_profile_snapshot(self):
        profile=json.loads((__import__('pathlib').Path(__file__).resolve().parents[1]/'config/api_profile.json').read_text(encoding='utf-8'))
        profile['status_queries'][0]['port']=self.server.bound_ports[19204]
        client=SeerClient('127.0.0.1',profile);state,_=client.snapshot()
        self.assertEqual(state['task'],'NONE');self.assertEqual(state['mode'],'AUTO');self.assertEqual(state['safety'],'CLEAR')
    def test_navigation_over_separate_control_socket(self):
        control=self.client(19207);nav=self.client(19206)
        try:
            control.request(4005,dict(nick_name='network-test'))
            nav.request(3051,dict(id='B'))
            for _ in range(500):self.robot.step(.1)
            state=self.client(19204)
            try:self.assertEqual(state.request(1100)['task_status'],4)
            finally:state.close()
        finally:control.close();nav.close()
    def test_offline_closes_connection(self):
        client=self.client(19204)
        try:
            client.request(1100);self.robot.inject('offline',1)
            with self.assertRaises(ConnectionError):client.request(1100)
        finally:client.close()
    def test_shutdown_with_idle_socket_and_partial_bind_failure(self):
        sock=socket.create_connection(('127.0.0.1',self.server.bound_ports[19204]))
        self.server.close();sock.close()
        occupied=socket.socket();occupied.bind(('127.0.0.1',0));occupied.listen()
        failed=EmulatorServer(self.robot,ports={19204:0,19205:occupied.getsockname()[1]})
        try:
            with self.assertRaises(OSError):failed.start()
            failed.close()
        finally:occupied.close()


if __name__=='__main__':unittest.main()
