import hashlib,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from seer_control.fairino_model import install,installed,RELATIVE
from seer_control.geometry3d import simplify_faces,box
class OfficialModelTests(unittest.TestCase):
 def fixture(self):
  source=('<robot name="fixture"><link name="base"/>'+''.join('<link name="link'+str(i)+'"/><joint name="j'+str(i)+'" type="revolute"><parent link="'+('base' if i==1 else 'link'+str(i-1))+'"/><child link="link'+str(i)+'"/><limit lower="-3" upper="3"/></joint>' for i in range(1,7))+'</robot>').encode()
  info=dict(revision='test',files={RELATIVE:hashlib.sha256(source).hexdigest()})
  return source,info
 def test_local_checkout_installs_verified_model(self):
  data,info=self.fixture()
  with tempfile.TemporaryDirectory() as folder,patch('seer_control.fairino_model.manifest',return_value=info):
   source=Path(folder)/'source';path=source/RELATIVE;path.parent.mkdir(parents=True);path.write_bytes(data)
   target=Path(folder)/'cache';result=install(target,source)
   self.assertEqual(installed(target),result);self.assertTrue((result.parents[2]/'SOURCE.json').is_file())
 def test_tampered_source_rejected_and_not_selected(self):
  data,info=self.fixture()
  with tempfile.TemporaryDirectory() as folder,patch('seer_control.fairino_model.manifest',return_value=info):
   source=Path(folder)/'source';path=source/RELATIVE;path.parent.mkdir(parents=True);path.write_bytes(data+b'bad')
   target=Path(folder)/'cache'
   with self.assertRaises(ValueError):install(target,source)
   self.assertIsNone(installed(target))
 def test_cached_files_are_checked_without_network(self):
  data,info=self.fixture()
  with tempfile.TemporaryDirectory() as folder,patch('seer_control.fairino_model.manifest',return_value=info):
   source=Path(folder)/'source';path=source/RELATIVE;path.parent.mkdir(parents=True);path.write_bytes(data)
   target=Path(folder)/'cache';result=install(target,source)
   with patch('urllib.request.urlopen') as fetch:self.assertEqual(install(target),result);fetch.assert_not_called()
   result.write_bytes(b'tampered');self.assertIsNone(installed(target))
 def test_download_uses_pinned_revision_and_checks_digest(self):
  import io
  data,info=self.fixture()
  with tempfile.TemporaryDirectory() as folder,patch('seer_control.fairino_model.manifest',return_value=info),patch('urllib.request.urlopen',return_value=io.BytesIO(data)) as fetch:
   result=install(folder);self.assertTrue(result.is_file());self.assertIn('/test/'+RELATIVE,fetch.call_args.args[0])
 def test_wrong_download_does_not_activate_model(self):
  import io
  data,info=self.fixture()
  with tempfile.TemporaryDirectory() as folder,patch('seer_control.fairino_model.manifest',return_value=info),patch('urllib.request.urlopen',return_value=io.BytesIO(b'bad')):
   with self.assertRaises(ValueError):install(folder)
   self.assertIsNone(installed(folder))
 def test_vertex_merging_keeps_connected_box_surface(self):
  # Triangulated box repeated many times: merge coincident vertices and duplicate faces.
  faces=[]
  for face in box((1,1,1)):
   faces.extend([(face[0],face[1],face[2]),(face[0],face[2],face[3])])
  result=simplify_faces(faces*100,12)
  self.assertEqual(len(result),12)
  edges={}
  for face in result:
   for a,b in zip(face,face[1:]+face[:1]):
    key=tuple(sorted((a,b)));edges[key]=edges.get(key,0)+1
  self.assertTrue(all(count==2 for count in edges.values()))
if __name__=='__main__':unittest.main()
