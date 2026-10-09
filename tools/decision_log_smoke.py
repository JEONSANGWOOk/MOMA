"""Tk + SIM audit integration in temporary settings; no hardware/network."""
import json
import argparse
from pathlib import Path
import sys
import tempfile
import time
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output-dir',type=Path)
    args=parser.parse_args()
    with tempfile.TemporaryDirectory() as folder, patch.object(Path,'home',return_value=Path(folder)):
        from seer_control.app import Console
        from seer_control.model import MapModel, Simulator
        from seer_control.studio_core import EditHistory
        import seer_control.app as app_module
        import seer_control.studio_ui as studio_module
        app_module.USER_DIR=Path(folder);studio_module.SETTINGS=Path(folder)/'settings.json'
        app=Console();app.withdraw();errors=[]
        app.report_callback_exception=lambda kind,value,trace:errors.append(str(value))
        try:
            app.pose_autosave.set(False)
            m=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id=k,x=x,y=y)
                for k,x,y in [('A',0,0),('B',3,0),('C',0,2),('D',3,2)]],
                edges=[['A','B'],['A','C'],['C','D'],['D','B']],walls=[],
                obstacles=[dict(id='BOX',x=1.5,y=0,radius=.3,map_fixed=False)]))
            app.map=m;app.sim=Simulator(m,app.decision_journal)
            app.studio_runner.start([dict(type='Path Nav',goal='B'),dict(type='Wait',duration_s=.1)])
            app.sim.obstacle_policy='auto';app.sim.auto_static_s=.5
            app.sim.auto_wait_s=.5;app.sim.auto_scenarios['static']='wait_reroute'
            clock=time.monotonic()
            for i in range(800):
                app.sim.tick(.1);app.studio_runner.tick(clock+i*.1,.1)
                if app.studio_runner.status=='COMPLETED':break
            assert app.studio_runner.status=='COMPLETED',app.studio_runner.error
            assert app.sim.state.last_node=='B'
            # Audit survives map undo snapshots and remains visible on the log page.
            assert EditHistory().capture(m)['decision_journal'] is app.decision_journal
            for _ in range(12):app._decision_tick()
            assert any(level=='판단' and '근거:' in text for stamp,level,text in app.logs)
            report=app.studio_runner.report
            assert any(e['source']=='SIM.대체 경로' for e in report.decisions)
            report.save(Path(folder)/'mission.json')
            stored=json.loads((Path(folder)/'mission.json').read_text(encoding='utf-8'))
            assert stored['success']==2 and stored['decisions']
            assert app.decision_journal.flush(),app.decision_journal.error
            lines=(app.decision_journal.folder/'decisions.jsonl').read_text(encoding='utf-8').splitlines()
            assert len(lines)>10
            assert all({'situation','evidence','conclusion','action','session','id'}<=set(json.loads(line)) for line in lines)
            assert not errors,errors
            if args.output_dir:
                args.output_dir.mkdir(parents=True,exist_ok=True)
                for suffix in ('jsonl','txt'):
                    (args.output_dir/('decision_log_smoke.'+suffix)).write_bytes((app.decision_journal.folder/('decisions.'+suffix)).read_bytes())
            print(f'PASS: Tk audit display, obstacle reroute, mission report, map snapshots and {len(lines)} persisted decisions; no hardware/network')
        finally:app.close()


if __name__=='__main__':main()
