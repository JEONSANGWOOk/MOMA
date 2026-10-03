import json, math, socket, tempfile, unittest
from pathlib import Path
from unittest.mock import patch
from seer_control.model import MapModel, Simulator
from seer_control.transport import HEADER, MAX_PAYLOAD, ReadOnlyClient, receive_exact
ROOT=Path(__file__).resolve().parents[1]

class ModelTests(unittest.TestCase):
    def setUp(self):
        self.map=MapModel.load(ROOT/'maps/demo.json');self.robot=Simulator(self.map)
    def test_shortest_path(self):
        self.assertEqual(self.map.route('LM1','LM3'),['LM1','LM2','LM3'])
    def test_disconnected_route_rejected(self):
        self.map.edges=[]
        with self.assertRaises(ValueError):self.map.route('LM1','LM3')
    def test_map_roundtrip(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'map.json';self.map.save(path)
            self.assertEqual(self.map.data(),MapModel.load(path).data())
    def test_bad_map_rejected(self):
        data=self.map.data();data['nodes'][0]=dict(id='LM1',x=float('nan'),y=0)
        with self.assertRaises(ValueError):MapModel(data)
        data=self.map.data();data['edges']=[['LM1','missing']]
        with self.assertRaises(ValueError):MapModel(data)
    def test_navigation_completes(self):
        self.robot.navigate('LM2')
        for _ in range(200):self.robot.tick(.1)
        self.assertEqual(self.robot.state.last_node,'LM2');self.assertEqual(self.robot.state.task,'완료')
    def test_stop_latches(self):
        self.robot.navigate('LM4');self.robot.command('stop');old=self.robot.state.x
        self.robot.tick(.1);self.assertEqual(self.robot.state.x,old);self.assertFalse(self.robot.route)
        with self.assertRaises(ValueError):self.robot.navigate('LM2')
        self.robot.command('reset');self.robot.navigate('LM2')
    def test_deadman_expires(self):
        self.robot.drive(.2,0)
        for _ in range(10):self.robot.tick(.1)
        old=self.robot.state.x;self.robot.tick(.1)
        self.assertEqual(old,self.robot.state.x);self.assertEqual(self.robot.state.speed,0)
    def test_motor_off(self):
        self.robot.drive(.2,0);self.robot.command('motor_off');old=self.robot.state.x
        self.robot.tick(.1);self.assertEqual(old,self.robot.state.x)
        with self.assertRaises(ValueError):self.robot.drive(.2,0)
    def test_pause_resume(self):
        self.robot.navigate('LM4');self.robot.tick(.1);self.robot.command('pause');old=self.robot.state.x
        for _ in range(4):self.robot.tick(.1)
        self.assertEqual(old,self.robot.state.x);self.robot.command('resume')
        for _ in range(4):self.robot.tick(.1)
        self.assertGreater(self.robot.state.x,old)
    def test_obstacle(self):
        self.robot.navigate('LM4');self.robot.command('obstacle');old=self.robot.state.x
        for _ in range(10):self.robot.tick(.1)
        self.assertEqual(old,self.robot.state.x)
    def test_scan_mapping(self):
        points=self.robot.scan();self.assertGreater(len(points),10)
        self.assertTrue(all(math.isfinite(v) for p in points for v in p))
        self.robot.command('mapping_start');self.robot.tick(.1)
        self.assertGreater(len(self.robot.cloud),0)
        self.robot.command('mapping_stop');self.assertFalse(self.robot.mapping)
    def test_charge(self):
        self.robot.command('charge')
        for _ in range(200):self.robot.tick(.1)
        self.assertTrue(self.robot.state.charging);self.assertEqual(self.robot.state.last_node,'CP1')

class FragmentedSocket:
    def __init__(self,data):self.data=bytearray(data);self.sent=b''
    def recv(self,n):
        out=bytes(self.data[:min(n,3)]);del self.data[:len(out)];return out
    def sendall(self,data):self.sent+=data
    def settimeout(self,n):pass
    def __enter__(self):return self
    def __exit__(self,*args):pass

class TransportTests(unittest.TestCase):
    def profile(self):
        # Fixture IDs only. These are NOT SEER API IDs.
        return dict(verified=True,transport='candidate-seer-16-byte',
            status_queries=[dict(name='position',port=12345,request_type=42,response_type=43,read_only=True)],
            fields={'x':{'path':'position.x','scale':.001}})
    def packet(self,sequence=1,kind=43,length=None):
        body=json.dumps({'x':3500}).encode()
        return HEADER.pack(0x5a,1,sequence,len(body) if length is None else length,kind,b'\0'*6)+body
    def test_default_blocked(self):
        profile=json.loads((ROOT/'config/api_profile.json').read_text())
        profile['verified']=False
        with self.assertRaises(ValueError):ReadOnlyClient('127.0.0.1',profile)
    def test_fragmentation_mapping(self):
        sock=FragmentedSocket(self.packet())
        with patch('socket.create_connection',return_value=sock):
            state,raw=ReadOnlyClient('127.0.0.1',self.profile()).snapshot()
        self.assertEqual(state['x'],3.5);self.assertEqual(HEADER.unpack(sock.sent[:16])[4],42)
    def test_sequence_type_validation(self):
        for kw in ({'sequence':2},{'kind':99}):
            with patch('socket.create_connection',return_value=FragmentedSocket(self.packet(**kw))):
                with self.assertRaises(ValueError):ReadOnlyClient('localhost',self.profile()).snapshot()
    def test_payload_limit(self):
        with patch('socket.create_connection',return_value=FragmentedSocket(self.packet(length=MAX_PAYLOAD+1))):
            with self.assertRaises(ValueError):ReadOnlyClient('localhost',self.profile()).snapshot()
    def test_eof(self):
        with self.assertRaises(ConnectionError):receive_exact(FragmentedSocket(b'123'),4)
    def test_timeout(self):
        with patch('socket.create_connection',side_effect=socket.timeout('timeout')):
            with self.assertRaises(socket.timeout):ReadOnlyClient('localhost',self.profile()).snapshot()
    def test_write_blocked(self):
        p=self.profile();p['status_queries'][0]['read_only']=False
        with self.assertRaises(ValueError):ReadOnlyClient('localhost',p)
if __name__=='__main__':unittest.main()
