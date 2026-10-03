import json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from seer_control.live import SeerClient
from seer_control.smap import load_smap
from seer_control.transport import HEADER
from test_core import FragmentedSocket
ROOT=Path(__file__).resolve().parents[1]
class LiveTests(unittest.TestCase):
    def setUp(self):self.client=SeerClient('localhost',json.loads((ROOT/'config/api_profile.json').read_text()))
    def response(self,api,body):
        data=json.dumps(body).encode();return FragmentedSocket(HEADER.pack(0x5a,1,1,len(data),api+10000,b'\0'*6)+data)
    def test_empty_request_and_units(self):
        sock=self.response(1100,dict(x=1,y=2,angle=.3,battery_level=.75,emergency=False,blocked=False,task_status=2))
        with patch('socket.create_connection',return_value=sock):state,raw=self.client.snapshot()
        self.assertEqual(len(sock.sent),16);self.assertEqual(state['battery'],75)
        self.assertEqual(state['task'],'RUNNING');self.assertEqual(state['safety'],'CLEAR')
    def test_controller_error(self):
        with patch('socket.create_connection',return_value=self.response(3051,dict(ret_code=40000))):
            with self.assertRaises(ValueError):self.client.command('navigate',{'id':'LM1'})
    def test_control_port_payload(self):
        sock=self.response(3051,{})
        with patch('socket.create_connection',return_value=sock) as call:self.client.command('navigate',{'id':'LM1'})
        self.assertEqual(call.call_args.args[0],('localhost',19206));self.assertEqual(json.loads(sock.sent[16:]),{'id':'LM1'})
    def test_missing_pose(self):
        with patch('socket.create_connection',return_value=self.response(1100,{})):
            with self.assertRaises(ValueError):self.client.snapshot()
    def test_empty_smap(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'map.smap';p.write_text(json.dumps(dict(header={'mapName':'test'},normalPosList=[{'x':0,'y':1}])))
            m=load_smap(p);self.assertEqual(m.nodes,{});self.assertEqual(m.cloud,[(0.,1.)])
    def test_tcp_fragmented_roundtrip(self):
        from tools.mock_robot import start
        servers=start((0,))
        try:
            r=self.client.request(dict(port=servers[0].server_address[1],request_type=1301,response_type=11301))
            self.assertEqual(r['stations'][0]['id'],'LM1')
        finally:
            for s in servers:s.shutdown();s.server_close()
    def test_download_map_4011(self):
        body={'header':{'mapName':'fixture'},'normalPosList':[{'x':1,'y':2}]}
        sock=self.response(4011,body)
        with patch('socket.create_connection',return_value=sock) as call:
            result=self.client.download_map('fixture')
        self.assertEqual(call.call_args.args[0],('localhost',19207))
        self.assertEqual(json.loads(sock.sent[16:]),{'map_name':'fixture'})
        self.assertEqual(result['header']['mapName'],'fixture')

    def test_downloaded_smap_data(self):
        from seer_control.smap import load_smap_data
        m=load_smap_data({'header':{'mapName':'robot'},'normalPosList':[{'x':1,'y':2}],
                          'advancedPointList':[{'className':'LandMark','instanceName':'P1','pos':{'x':3,'y':4}}]})
        self.assertEqual(m.name,'robot');self.assertEqual(m.cloud,[(1.,2.)]);self.assertIn('P1',m.nodes)

    def test_map_load_port_and_name(self):
        sock=self.response(2022,{'ret_code':0})
        with patch('socket.create_connection',return_value=sock) as call:self.client.command('load_map',{'map_name':'floor2'})
        self.assertEqual(call.call_args.args[0],('localhost',19205))
        self.assertEqual(json.loads(sock.sent[16:]),{'map_name':'floor2'})
    def test_invalid_map_name_rejected_before_network(self):
        with patch('socket.create_connection') as call:
            with self.assertRaises(ValueError):self.client.command('load_map',{'map_name':'../map'})
            call.assert_not_called()

class ManualMotionTests(unittest.TestCase):
    def setUp(self):
        self.client=SeerClient('localhost',json.loads((ROOT/'config/api_profile.json').read_text()))
    def response(self,api,body):
        data=json.dumps(body).encode();return FragmentedSocket(HEADER.pack(0x5a,1,1,len(data),api+10000,b'\0'*6)+data)
    def test_open_loop_motion_2010(self):
        sock=self.response(2010,{})
        with patch('socket.create_connection',return_value=sock) as call:
            self.client.command('motion',{'vx':0.1,'vy':0.0,'w':-0.2})
        self.assertEqual(call.call_args.args[0],('localhost',19205))
        self.assertEqual(json.loads(sock.sent[16:]),{'vx':0.1,'vy':0.0,'w':-0.2})
    def test_motion_limit(self):
        with patch('socket.create_connection') as call:
            with self.assertRaises(ValueError):
                self.client.command('motion',{'vx':0.8,'vy':0.0,'w':0.0})
            call.assert_not_called()
    def test_stop_open_loop_2000(self):
        sock=self.response(2000,{})
        with patch('socket.create_connection',return_value=sock) as call:
            self.client.command('stop_motion')
        self.assertEqual(call.call_args.args[0],('localhost',19205))



class ExtendedApiTests(unittest.TestCase):
    def test_laser_points_ranges_to_world(self):
        from seer_control.live import laser_points
        data={'angle_min':0.0,'angle_increment':1.5707963267948966,'ranges':[1.0,2.0], 'range_min':0.01,'range_max':10.0}
        pts=laser_points(data,(10.0,20.0,0.0))
        self.assertEqual(len(pts),2)
        self.assertAlmostEqual(pts[0][0],11.0,places=6)
        self.assertAlmostEqual(pts[0][1],20.0,places=6)
        self.assertAlmostEqual(pts[1][0],10.0,places=6)
        self.assertAlmostEqual(pts[1][1],22.0,places=6)

    def test_laser_points_polar_dicts(self):
        from seer_control.live import laser_points
        pts=laser_points({'laser_beams':[{'angle':0.0,'distance':1.5},{'angle':1.5707963267948966,'dist':2.0}]})
        self.assertEqual(len(pts),2)
        self.assertAlmostEqual(pts[0][0],1.5,places=6)
        self.assertAlmostEqual(pts[1][1],2.0,places=6)

    def test_control_lock_commands_present(self):
        from seer_control.live import COMMANDS
        self.assertEqual(COMMANDS['lock_control'],(19207,4005))
        self.assertEqual(COMMANDS['unlock_control'],(19207,4006))

class ControlNicknameValidationTests(unittest.TestCase):
    def _client(self):
        profile={'verified':True,'transport':'seer-16-byte','status_queries':[{'name':'x','port':19204,'request_type':1000,'response_type':11000,'read_only':True}],'fields':{}}
        return SeerClient('127.0.0.1',profile)

    def test_lock_control_requires_nick_name(self):
        client=self._client()
        with self.assertRaisesRegex(ValueError,'nick_name'):
            client.command('lock_control',{})

    def test_unlock_control_requires_nick_name(self):
        client=self._client()
        with self.assertRaisesRegex(ValueError,'nick_name'):
            client.command('unlock_control',{})
