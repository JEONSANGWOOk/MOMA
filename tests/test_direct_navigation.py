import ast
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

ROOT=Path(__file__).resolve().parents[1]

class DirectNavigationTests(unittest.TestCase):
    def test_web_move_sends_only_navigation_without_localization(self):
        tree=ast.parse((ROOT/'web_control/server.py').read_text(encoding='utf-8'))
        method=next(n for n in ast.walk(tree) if isinstance(n,ast.AsyncFunctionDef) and n.name=='command_request')
        execute=next(n for n in method.body if isinstance(n,ast.FunctionDef) and n.name=='execute')
        client=Mock();client.command.return_value={'ret_code':0}
        context=NS(epoch=1,navigation_cancel_seq=0,require_live=Mock(),arm_travel_guard=Mock(),mode='control')
        env={'self':context,'c':client,'name':'navigate','payload':{'id':'LM1'},'epoch':1,'cancel_seq':0}
        exec(compile(ast.Module(body=[execute],type_ignores=[]),'direct-navigation','exec'),env)
        self.assertEqual(env['execute'](),{'ret_code':0})
        self.assertEqual(client.method_calls,[('command',('navigate',{'id':'LM1'}),{})])
        context.arm_travel_guard.assert_called_once()
        context.epoch=2
        with self.assertRaises(ValueError):env['execute']()
        self.assertEqual(client.command.call_count,1)

    def test_desktop_guard_keeps_cancel_and_freshness_without_laser_dependency(self):
        tree=ast.parse((ROOT/'seer_control/app.py').read_text(encoding='utf-8'))
        guard=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='guard_navigation')
        context=NS(generation=3,connected=True,control_enabled=True,last_state=10.,_navigation_cancel_seq=0)
        env={'self':context,'stop':NS(is_set=lambda:False),'generation':3,'cancel_seq':0,'time':NS(monotonic=lambda:10.1)}
        exec(compile(ast.Module(body=[guard],type_ignores=[]),'navigation-guard','exec'),env)
        env['guard_navigation']()
        context._navigation_cancel_seq=1
        with self.assertRaises(ValueError):env['guard_navigation']()
