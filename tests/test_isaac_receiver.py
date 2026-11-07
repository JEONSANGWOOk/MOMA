import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

class IsaacReceiverTests(unittest.TestCase):
    def test_update_can_access_global_omni_before_optional_physics(self):
        path=Path(__file__).resolve().parents[1]/'tools/isaac_mirror/isaac_live_view.py'
        tree=ast.parse(path.read_text(encoding='utf-8'))
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='IsaacMirror')
        update=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='update')
        module=ast.Module(body=[update],type_ignores=[])
        context=Mock();context.get_stage.return_value=None
        ns={'omni':NS(usd=NS(get_context=lambda:context))}
        exec(compile(module,str(path),'exec'),ns)
        ns['update'](Mock(),None)
        context.get_stage.assert_called_once()

    def test_initial_focus_excludes_factory_map_and_distant_equipment(self):
        path=Path(__file__).resolve().parents[1]/'tools/isaac_mirror/isaac_live_view.py'
        cls=next(n for n in ast.parse(path.read_text(encoding='utf-8')).body if isinstance(n,ast.ClassDef))
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='focus_paths')
        ns={};exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),ns)
        view=NS(ROOT='/World/MOMA_Mirror',instances={
            'base':{'physics_role':'robot'},'arm':{'physics_role':'robot'},
            'hook':{'physics_role':'tool'},'factory':{'physics_role':'equipment'},
            'door':{'physics_role':'door'},'not_loaded':{'physics_role':'robot'}},
            transforms=dict.fromkeys(['base','arm','hook','factory','door']),
            targets=dict.fromkeys(['base','arm','hook','factory','door']))
        self.assertEqual(ns['focus_paths'](view),['/World/MOMA_Mirror/base','/World/MOMA_Mirror/arm','/World/MOMA_Mirror/hook'])
