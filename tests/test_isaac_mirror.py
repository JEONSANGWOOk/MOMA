import json
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from urllib.request import urlopen,Request
from urllib.error import HTTPError
from http.server import ThreadingHTTPServer
from seer_control.geometry3d import identity,transform
from seer_control.isaac_mirror import MirrorPublisher,map_snapshot,sample_points
from tools.isaac_mirror.serve import handler

class MirrorTests(unittest.TestCase):
    def test_mesh_materials_and_transform_are_separate(self):
        with tempfile.TemporaryDirectory() as folder:
            pub=MirrorPublisher(folder)
            try:
                face=((0,0,0),(1,0,0),(0,1,0))
                mesh=NS(faces=[face],face_materials=[0],material_colors={0:'#123456'})
                pub.export([],[(mesh,identity(),'#ffffff')],[],dict(mode='SIM',connected=True))
                manifest=json.loads((Path(folder)/'manifest.json').read_text())
                self.assertEqual(next(iter(manifest['geometry'].values()))['color'],'#123456')
                revision=manifest['revision']
                pub.export([],[(mesh,transform((2,3,0)),'#ffffff')],[],dict(mode='REAL',connected=True))
                frame=json.loads((Path(folder)/'frame.json').read_text())
                self.assertEqual(frame['revision'],revision)
                self.assertEqual(frame['instances'][0]['matrix'][0][3],2)
                self.assertEqual(frame['sequence'],2)
            finally:pub.close();pub.thread.join(1)
    def test_robot_fk_and_quads(self):
        with tempfile.TemporaryDirectory() as folder:
            pub=MirrorPublisher(folder)
            try:
                quad=((0,0,0),(1,0,0),(1,1,0),(0,1,0))
                robot=NS(links={'J1':[([quad],identity(),'#abcdef')]},link_transforms=lambda q,w:{'J1':transform((q['q'],0,0))})
                pub.export([(robot,{'q':.4},identity(),None,1)],[],[],{})
                manifest=json.loads((Path(folder)/'manifest.json').read_text())
                geo=next(iter(manifest['geometry'].values()))
                self.assertEqual(geo['counts'],[3,3])
                self.assertEqual(len(geo['points']),4)
                self.assertEqual(len(geo['indices']),6)
                frame=json.loads((Path(folder)/'frame.json').read_text())
                self.assertEqual(frame['instances'][0]['matrix'][0][3],.4)
            finally:pub.close();pub.thread.join(1)
    def test_server_has_read_only_fixed_routes(self):
        with tempfile.TemporaryDirectory() as folder:
            Path(folder,'frame.json').write_text('{"version":1}')
            server=ThreadingHTTPServer(('127.0.0.1',0),handler(Path(folder)))
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            base=f'http://127.0.0.1:{server.server_port}'
            try:
                with urlopen(base+'/frame') as response:self.assertEqual(json.load(response)['version'],1)
                for path in ('/../settings.json','/command'):
                    with self.assertRaises(HTTPError):urlopen(base+path)
                with self.assertRaises(HTTPError):urlopen(Request(base+'/frame',data=b'{}'))
            finally:server.shutdown();server.server_close();thread.join(1)

    def test_map_and_lidar_export_and_map_revision(self):
        with tempfile.TemporaryDirectory() as folder:
            publisher=MirrorPublisher(folder)
            try:
                model=NS(name='test',cloud=[(0,0),(1,1)],nodes={'LM1':dict(x=1,y=2)},edges=[],walls=[],curves=[])
                data=map_snapshot(model)
                publisher.export([],[],[],{'_map':data,'_lidar':[[.1,.2]],'connected':True})
                scene=json.loads(Path(folder,'manifest.json').read_text())
                frame=json.loads(Path(folder,'frame.json').read_text())
                self.assertEqual(scene['map']['nodes'][0]['id'],'LM1')
                self.assertEqual(frame['lidar'],[[.1,.2]])
                revision=frame['revision']
                publisher.export([],[],[],{'_map':data,'_lidar':[]})
                frame=json.loads(Path(folder,'frame.json').read_text())
                self.assertEqual(frame['revision'],revision)
                self.assertEqual(frame['lidar'],[])
            finally:publisher.close();publisher.thread.join(1)
    def test_point_count_cap_and_nonfinite_filter(self):
        self.assertLessEqual(len(sample_points([(1,2)]*9001,4000)),4000)
        self.assertEqual(sample_points([(float('nan'),0),(1,2)]),[[1,2]])

    def test_full_source_replaces_preview_geometry(self):
        from unittest.mock import patch
        face=((0,0,0),(1,0,0),(0,1,0))
        full=NS(links={'base_link':[([face,face],identity(),'#abcdef')]},full_visuals={},source_triangle_count=2,
                link_transforms=lambda q,w:{'base_link':w})
        preview=NS(source_static_glb=dict(path='source.glb',rotation=(0,0,0),offset=(0,0,0),color='#abcdef'))
        with tempfile.TemporaryDirectory() as folder:
            publisher=MirrorPublisher(folder)
            try:
                with patch('seer_control.isaac_mirror.load_glb_static_robot',return_value=full) as loader:
                    publisher.export([(preview,{},identity(),None,1)],[],[],{})
                    loader.assert_called_once_with('source.glb',max_faces=None,rotation=(0,0,0),offset=(0,0,0),color='#abcdef')
                scene=json.loads(Path(folder,'manifest.json').read_text())
                self.assertEqual(sum(g['triangles'] for g in scene['geometry'].values()),2)
                import gzip
                self.assertEqual(json.loads(gzip.decompress(Path(folder,'manifest.json.gz').read_bytes())),scene)
            finally:publisher.close();publisher.thread.join(1)
