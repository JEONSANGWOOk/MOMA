"""Run on a desktop (or xvfb-run) after downloading. Does not access a robot."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from seer_control.app import Console
app=Console()
try:
    app.update()
    assert app.connected and not app.real
    app.target.set('LM2');app.navigate()
    for _ in range(200):app.sim.tick(.1)
    assert app.sim.state.last_node=='LM2'
    app.draw_map();app.update()
    for page in (app.nodes_page,app.tasks_page,app.telemetry_page,app.settings_page,app.logs_page):
        app.tabs.select(page);app.update()
    app.disconnect();app.update()
    assert not app.connected
    print('GUI smoke: PASS (simulation only)')
finally:
    app.destroy()
