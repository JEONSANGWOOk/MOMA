import json, unittest
from unittest.mock import patch
from pathlib import Path
from seer_control.smap import load_smap_data, build_smap_from_model, make_path_record
from seer_control.live import SeerClient
from seer_control.transport import HEADER
from test_core import FragmentedSocket
ROOT=Path(__file__).resolve().parents[1]

class SmapEditTests(unittest.TestCase):
    def fixture(self):
        return {
            'header':{'mapName':'floorA','mapType':'2D-map','resolution':0.02},
            'normalPosList':[{'x':0,'y':0}],
            'advancedPointList':[
                {'className':'LandMark','instanceName':'LM1','pos':{'x':0,'y':0},'dir':0.0,
                 'property':[{'key':'spin','type':'bool','value':'dHJ1ZQ==','boolValue':True}]},
                {'className':'LandMark','instanceName':'LM2','pos':{'x':2,'y':0},'dir':1.0,'property':[]}],
            'advancedCurveList':[
                {'className':'BezierPath','instanceName':'LM1-LM2',
                 'startPos':{'className':'LandMark','instanceName':'LM1','pos':{'x':0,'y':0}},
                 'endPos':{'className':'LandMark','instanceName':'LM2','pos':{'x':2,'y':0}},
                 'controlPos1':{'x':0.5,'y':0.4},'controlPos2':{'x':1.5,'y':0.4},
                 'property':[{'key':'direction','type':'int32','value':'MQ==','int32Value':1}], 'devices':[]}]
        }

    def test_parser_preserves_graph_edges(self):
        m=load_smap_data(self.fixture())
        self.assertEqual(m.edges,[['LM1','LM2']])
        self.assertEqual(m.path_records[0]['a'],'LM1')
        self.assertEqual(m.path_records[0]['properties']['direction'],1)

    def test_add_point_and_path_builds_valid_smap(self):
        m=load_smap_data(self.fixture())
        m.nodes['LM3']={'id':'LM3','x':3.0,'y':1.0,'r':1.57,'angle':1.57,'spin':True,'type':'LocationMark','draft':True}
        raw=make_path_record(m.nodes['LM2'],m.nodes['LM3'],'BezierPath',{'direction':1},m.path_records[0]['raw'])
        m.edges.append(['LM2','LM3'])
        m.path_records.append({'a':'LM2','b':'LM3','className':'BezierPath','instanceName':'LM2-LM3','raw':raw,
                               'controls':[(raw['controlPos1']['x'],raw['controlPos1']['y']),(raw['controlPos2']['x'],raw['controlPos2']['y'])],
                               'properties':{'direction':1},'draft':True})
        out=build_smap_from_model(m)
        pts={p['instanceName']:p for p in out['advancedPointList']}
        self.assertIn('LM3',pts); self.assertAlmostEqual(pts['LM3']['dir'],1.57)
        paths={(p['startPos']['instanceName'],p['endPos']['instanceName']):p for p in out['advancedCurveList']}
        self.assertIn(('LM2','LM3'),paths)
        self.assertEqual(paths[('LM2','LM3')]['className'],'BezierPath')
        # Unrelated map layer is preserved.
        self.assertEqual(out['normalPosList'],[{'x':0,'y':0}])

class UploadApiTests(unittest.TestCase):
    def test_upload_map_4010_sends_whole_smap(self):
        profile=json.loads((ROOT/'config/api_profile.json').read_text())
        client=SeerClient('localhost',profile)
        body={'ret_code':0}
        data=json.dumps(body).encode()
        sock=FragmentedSocket(HEADER.pack(0x5a,1,1,len(data),14010,b'\0'*6)+data)
        smap={'header':{'mapName':'floorA'},'advancedPointList':[],'advancedCurveList':[]}
        with patch('socket.create_connection',return_value=sock) as call:
            client.upload_map(smap)
        self.assertEqual(call.call_args.args[0],('localhost',19207))
        self.assertEqual(json.loads(sock.sent[16:]),smap)

if __name__=='__main__':unittest.main()
