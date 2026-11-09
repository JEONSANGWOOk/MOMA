import ast
import json
import os
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
from seer_control.fr5_readonly_service import snapshot,write_state
from seer_control.fr5_readonly_feedback import read,alive

class ReadOnlySDKTests(unittest.TestCase):
    def test_snapshot_reads_outputs_without_any_write_api(self):
        robot=NS(GetActualJointPosDegree=lambda:(0,[1,2,3,4,5,6]),GetActualTCPNum=lambda:(0,1),GetDO=lambda:(0,[0,128]),GetToolDO=lambda:(0,0))
        data=snapshot(robot,dict(stamp=time.monotonic(),count=3))
        self.assertIs(data['io']['DO']['7'],True);self.assertEqual(data['tool_index'],1)
        with self.assertRaises(ValueError):snapshot(robot,dict(stamp=time.monotonic()-2,count=3))
        with self.assertRaises(ValueError):snapshot(robot,dict(stamp=time.monotonic(),count=0))

    def test_windows_reader_sharing_failure_does_not_require_robot_reconnect(self):
        with tempfile.TemporaryDirectory() as temp:
            original=Path.replace;calls=[]
            def flaky(path,target):
                calls.append(1)
                if len(calls)<3:raise PermissionError('reader holds snapshot')
                return original(path,target)
            with patch.object(Path,'replace',flaky),patch('seer_control.fr5_readonly_service.time.sleep'):
                self.assertTrue(write_state(Path(temp),{'valid':True}))
            self.assertEqual(json.loads((Path(temp)/'fr5_direct_feedback.json').read_text()),{'valid':True})

    def test_stale_wrong_robot_and_wrong_receiver_do_not_drive_display(self):
        with tempfile.TemporaryDirectory() as temp,patch('seer_control.app.USER_DIR',Path(temp)):
            folder=Path(temp);config=dict(enabled=True,controller_ip='robot',ssh_host='relay',amr_hosts=['relay'])
            (folder/'fr5_readonly.json').write_text(json.dumps(config));(folder/'fr5_readonly.pid').write_text('123')
            data=dict(valid=True,read_only=True,controller_ip='robot',pid=123,sample_monotonic=time.monotonic())
            def check(data,host='relay'):
                (folder/'fr5_direct_feedback.json').write_text(json.dumps(data))
                app=NS(real=True,host=NS(get=lambda:host),_fr5_readonly_check=time.monotonic())
                return read(app,time.monotonic())
            self.assertIsNotNone(check(data))
            self.assertIsNone(check(dict(data,sample_monotonic=time.monotonic()-2)))
            self.assertIsNone(check(dict(data,pid=124)))
            self.assertIsNone(check(data,host='another-robot'))

    def test_service_has_no_motion_enable_or_output_calls(self):
        path=Path(__file__).resolve().parents[1]/'seer_control/fr5_readonly_service.py'
        tree=ast.parse(path.read_text(encoding='utf-8'))
        calls=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
        self.assertFalse(set(calls)&{'MoveJ','MoveL','ServoMoveStart','RobotEnable','Mode','SetDO','SetToolDO','StopMotion','SetToolCoord'})
        self.assertTrue(alive(os.getpid()))

if __name__=='__main__':unittest.main()
