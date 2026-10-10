import json
import tempfile
import unittest
from pathlib import Path
from seer_control.vision_stream import publish_frame,read_frame


class StreamTests(unittest.TestCase):
    def test_latest_frame_is_independent_of_archive_and_uses_control_filter(self):
        with tempfile.TemporaryDirectory() as folder:
            live=Path(folder)/'live.json';log=Path(folder)/'archive.jsonl'
            log.write_text(json.dumps(dict(timestamp=100,board=dict(valid=True)))+'\n',encoding='utf-8')
            for stamp in (100.03,100.06,100.09):
                publish_frame(live,dict(timestamp=stamp,board=dict(valid=True,value='display'),control_board=dict(valid=True,value='servo')))
                result=read_frame(log,live,now=stamp)
                self.assertEqual(result['timestamp'],stamp)
                self.assertEqual(result['board']['value'],'servo')
            self.assertEqual(json.loads(log.read_text())['timestamp'],100)

    def test_fresh_invalid_live_frame_never_falls_back_to_valid_archive(self):
        with tempfile.TemporaryDirectory() as folder:
            live=Path(folder)/'live.json';log=Path(folder)/'archive.jsonl'
            log.write_text(json.dumps(dict(timestamp=100,board=dict(valid=True)))+'\n',encoding='utf-8')
            publish_frame(live,dict(timestamp=100.1,board=dict(valid=False,reason='lost')))
            self.assertFalse(read_frame(log,live,now=100.1)['board']['valid'])

    def test_missing_or_partial_snapshot_falls_back_to_archive(self):
        with tempfile.TemporaryDirectory() as folder:
            live=Path(folder)/'live.json';log=Path(folder)/'archive.jsonl'
            log.write_text(json.dumps(dict(timestamp=100,board=dict(valid=True)))+'\n',encoding='utf-8')
            self.assertEqual(read_frame(log,live,now=100)['timestamp'],100)
            live.write_text('{',encoding='utf-8')
            self.assertEqual(read_frame(log,live,now=100)['timestamp'],100)


if __name__=='__main__':unittest.main()
