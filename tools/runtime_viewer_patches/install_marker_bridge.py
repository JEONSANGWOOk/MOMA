"""Run on controller: python3 install_marker_bridge.py /home/sdc/ws_sdc/viewer.
Only edits files after compiling; never starts a scenario or restarts the service.
"""
import ast
from pathlib import Path
import shutil
import sys
import time


def prepare(folder):
    folder=Path(folder)
    scenario=folder/'scenario.py';viewer=folder/'viewer_moma.py'
    a=scenario.read_text(encoding='utf-8');b=viewer.read_text(encoding='utf-8')
    if 'equipment_marker_bridge.capture' not in a:
        tree=ast.parse(a);main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main4')
        lines=a.splitlines(True);matches=[n for n in ast.walk(main) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='marker' for t in n.targets) and 'target_pose' in ast.get_source_segment(a,n)]
        if len(matches)!=1:raise ValueError('Expected exactly one WORLD marker assignment in main4')
        n=matches[0]
        if 'COORD.WORLD' not in ast.get_source_segment(a,n):raise ValueError('Marker coordinate must be WORLD')
        lines.insert(n.end_lineno,'    import equipment_marker_bridge\n    equipment_marker_bridge.capture(moma,marker)\n')
        a=''.join(lines)
    if 'equipment_marker_bridge.status()' not in b:
        needle='"robot": {'
        if b.count(needle)!=1:raise ValueError('Unexpected viewer status schema; no files changed')
        # Runtime imports scenario in the same process; both share bridge state.
        b='import equipment_marker_bridge\n'+b if not b.startswith('from __future__') else b
        if 'import equipment_marker_bridge' not in b:raise ValueError('Place bridge import after future imports manually')
        b=b.replace(needle,needle+'\n                "equipment_marker": equipment_marker_bridge.status(),',1)
    if "if self.path == '/api/equipment_marker':" not in b:
        endpoint="        if self.path == '/api/connection_status':"
        if endpoint in b:
            b=b.replace(endpoint,'''        if self.path == '/api/equipment_marker':
            self.send_response(200)
            self.send_header('Content-type','application/json')
            self.end_headers()
            self.wfile.write(json.dumps(equipment_marker_bridge.status()).encode('utf-8'))
            return

'''+endpoint,1)
    compile(a,str(scenario),'exec');compile(b,str(viewer),'exec')
    return [(scenario,a),(viewer,b)]


def main():
    folder=Path(sys.argv[1]).resolve();changes=prepare(folder)
    source=Path(__file__).with_name('equipment_marker_bridge.py')
    compile(source.read_text(encoding='utf-8'),str(source),'exec')
    stamp=time.strftime('%Y%m%d-%H%M%S')
    for path,text in changes:
        shutil.copy2(path,str(path)+'.bak-marker-'+stamp)
        path.write_text(text,encoding='utf-8')
    target=folder/source.name
    if source.resolve()!=target.resolve():shutil.copy2(source,target)
    print('Installed. Restart Viewer when idle and reload scenario. No robot commands sent.')

if __name__=='__main__':main()
