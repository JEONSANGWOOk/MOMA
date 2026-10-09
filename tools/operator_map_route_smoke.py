"""Exercise user-mode map route editing with temporary settings, without hardware."""
import sys
import tempfile
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

with tempfile.TemporaryDirectory() as folder, patch.object(Path, 'home', return_value=Path(folder)):
    import seer_control.app as am
    import seer_control.studio_ui as ui
    from seer_control.model import MapModel, Simulator
    am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json'
    app=am.Console();errors=[];dialogs=[]
    app.report_callback_exception=lambda t,v,b:errors.append(str(v))
    try:
        app.pose_autosave.set(False)
        app.map=MapModel(dict(format='amr-console-map-v1',nodes=[
            dict(id='A',x=0,y=0),dict(id='B',x=3,y=0),dict(id='C',x=3,y=3)],
            edges=[['A','B'],['B','C']],walls=[],obstacles=[]))
        app.sim=Simulator(app.map,app.decision_journal)
        app.ui_role.set('사용자');app._role_apply();app.geometry('1280x720');app.update()
        assert app.role_active=='사용자' and app.tabs.select()==str(app.operator_page)
        canvas=app.operator_map;canvas.redraw()
        event_clock=[1000]
        def click(node, shift=False):
            event_clock[0]+=1000
            x,y=canvas.xy(app.map.nodes[node]['x'],app.map.nodes[node]['y'])
            canvas.event_generate('<Button-1>',x=round(x),y=round(y),state=1 if shift else 0,time=event_clock[0])
            app.update()
        click('B');click('C')
        assert app.operator_kind.get()=='지도 경로 이동'
        assert list(app.operator_cycle_nodes.get(0,'end'))==['B','C']
        assert canvas.route()==['A','B','C'] and canvas.find_withtag('mission_route')
        assert app.operator_cycle_panel.winfo_ismapped()
        plan=app._operator_preview()
        assert [a['goal'] for a in plan['actions']]==['B','C']
        assert app.operator_pending
        click('B',shift=True)
        assert list(app.operator_cycle_nodes.get(0,'end'))==['C']
        assert app.operator_pending is None and app.operator_plan is None
        assert str(app.operator_start_button.cget('state'))=='disabled'
        click('B');click('C')
        click('C',shift=True)
        assert list(app.operator_cycle_nodes.get(0,'end'))==['C','B']
        app.operator_cycle_nodes.selection_set(0);app.operator_cycle_nodes.focus_set()
        app.operator_cycle_nodes.event_generate('<Delete>');app.update()
        assert list(app.operator_cycle_nodes.get(0,'end'))==['B']
        app._operator_route_clear();canvas.redraw()
        assert canvas.route()==[] and not canvas.find_withtag('mission_route')
        try:app._operator_preview()
        except ValueError:pass
        else:raise AssertionError('Empty route was accepted')
        # The existing roundtrip mode still adds visits and returns to its origin.
        app._operator_open_cycle();click('B');click('C')
        assert [a['goal'] for a in app._operator_preview()['actions']]==['B','C','A']
        app.studio_runner.start([dict(type='Wait',duration_s=10)])
        before=list(app.operator_cycle_nodes.get(0,'end'))
        with patch.object(am.messagebox,'showerror',side_effect=lambda *a,**k:dialogs.append(a)):
            click('B');click('C',shift=True)
        assert list(app.operator_cycle_nodes.get(0,'end'))==before and len(dialogs)==2
        app.studio_runner.cancel()
        assert not errors,errors
        print('PASS: user-mode Tk clicks, ordered route preview, Shift/Delete/clear removal, invalidation, roundtrip and active mission guard')
    finally:
        app.close()
