import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from seer_control.app import Console
from seer_control.model import MapModel, Simulator
from seer_control.smap import path_record_geometry, make_path_record


class CurveEditorTests(unittest.TestCase):
    def test_open_smap_installs_same_map_in_simulator(self):
        m=MapModel(dict(format='amr-console-map-v1',name='SMAP',nodes=[
            dict(id='A',x=0,y=0),dict(id='B',x=3,y=0)],edges=[['A','B']],walls=[]))
        raw=make_path_record(m.nodes['A'],m.nodes['B'])
        raw['controlPos1']['y']=2;raw['controlPos2']['y']=2
        m.path_records=[dict(a='A',b='B',raw=raw,properties={})]
        c=SimpleNamespace(real=False,task_running=False,sim=Mock(),release_drive=Mock(),
            guarded=lambda fn:fn(),refresh_tasks=Mock(),refresh_nodes=Mock(),fit_map=Mock(),
            log=Mock(),connection_text=Mock())
        with patch('seer_control.app.filedialog.askopenfilename',return_value='test.smap'),patch('seer_control.app.load_smap',return_value=m):
            Console.open_smap(c)
        self.assertIs(c.map,c.sim.map)
        self.assertTrue(c.connected)
        c.sim.navigate('B')
        self.assertGreater(max(y for x,y in c.sim.navigation_points()),1)

    def test_simulator_follows_directional_curves_and_pause_cancel(self):
        m=MapModel(dict(format='amr-console-map-v1',name='M',nodes=[
            dict(id='A',x=0,y=0),dict(id='B',x=3,y=0)],edges=[['A','B']],walls=[]))
        m.path_records=[]
        for a,b,y in [('A','B',2),('B','A',-2)]:
            raw=make_path_record(m.nodes[a],m.nodes[b])
            raw['controlPos1']['y']=y;raw['controlPos2']['y']=y
            m.path_records.append(dict(a=a,b=b,raw=raw,properties={}))
        s=Simulator(m);s.navigate('B')
        self.assertGreater(max(y for x,y in s.navigation_points()),1)
        trace=[]
        for _ in range(60):s.tick(.1);trace.append((s.state.x,s.state.y))
        self.assertGreater(max(y for x,y in trace),1)
        s.command('pause');p=(s.state.x,s.state.y)
        s.tick(.1);self.assertEqual(p,(s.state.x,s.state.y))
        s.command('resume')
        for _ in range(200):s.tick(.1)
        self.assertEqual((s.state.x,s.state.y),(3,0))
        self.assertEqual(s.state.task,'완료')
        s.navigate('A');trace=[]
        for _ in range(200):s.tick(.1);trace.append(s.state.y)
        self.assertLess(min(trace),-1)
        self.assertEqual(s.state.last_node,'A')
        s.navigate('B')
        for _ in range(20):s.tick(.1)
        s.command('cancel');p=(s.state.x,s.state.y)
        s.tick(.1);self.assertEqual(p,(s.state.x,s.state.y))
        self.assertFalse(s._waypoints)

    def test_select_drag_and_json_roundtrip_preserve_routes(self):
        m=MapModel(dict(format='amr-console-map-v1',name='M',nodes=[
            dict(id='A',x=0,y=0),dict(id='B',x=3,y=0),dict(id='C',x=6,y=0)],
            edges=[['A','B'],['B','C']],walls=[]))
        c=SimpleNamespace(map=m,curve_edit_record=None,curve_drag_index=None,
            editable=Mock(),xy=lambda x,y:(x*100,y*100),world=lambda x,y:(x/100,y/100),
            refresh_nodes=Mock(),draw_map=Mock(),guarded=lambda fn:fn(),map_dirty=False)
        c._set_curve_controls=lambda rec,controls:Console._set_curve_controls(c,rec,controls)
        Console._curve_click(c,SimpleNamespace(x=150,y=0))
        rec=c.curve_edit_record
        self.assertEqual(m.route('A','C'),['A','B','C'])
        self.assertEqual(m.route('C','A'),['C','B','A'])
        Console._curve_click(c,SimpleNamespace(x=100,y=0))
        self.assertEqual(c.curve_drag_index,0)
        Console.map_drag(c,SimpleNamespace(x=100,y=100))
        self.assertEqual(rec['raw']['controlPos1'],dict(x=1.,y=1.))
        self.assertGreater(max(y for x,y in path_record_geometry(rec['raw'])),0)
        self.assertTrue(c.map_dirty)
        reverse=next(r for r in m.path_records if r['a']=='B' and r['b']=='A')
        self.assertEqual(reverse['controls'],list(reversed(rec['controls'])))
        s=Simulator(m);s.state.x=3;s.navigate('A')
        self.assertGreater(max(y for x,y in s.navigation_points()),0)
        trace=[]
        for _ in range(200):s.tick(.1);trace.append(s.state.y)
        self.assertGreater(max(trace),0)
        self.assertEqual(s.state.last_node,'A')
        restored=MapModel(m.data())
        self.assertEqual(restored.path_records[0]['controls'][0],(1.,1.))
        self.assertEqual(restored.route('C','A'),['C','B','A'])


if __name__=='__main__':unittest.main()
