"""Exercise the real MOMA host and optional read-only D455 capture; never moves FR5."""
import argparse,json,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from seer_control.app import Console
from seer_control.vision_camera_ui import preview_record
from tools.window_capture import capture_window


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--camera',action='store_true');args=parser.parse_args()
    app=Console();errors=[];stamps=set();start=time.monotonic();visited=[]
    def callback_error(typ,value,tb):
        import traceback
        errors.append(str(value));traceback.print_exception(typ,value,tb)
    app.report_callback_exception=callback_error
    app.after(100,lambda:app.tabs.select(app.vision_panel_page))
    def run():
        workspace=app.vision_panel_workspace
        if workspace is None:
            if time.monotonic()-start>8:errors.append('Embedded workspace did not initialize');app.close();return
            app.after(100,run);return
        if args.camera and workspace.vision_camera.process is None and not stamps:workspace.vision_camera.start()
        link=workspace.key_panel_link
        if link.client.connected or link.plan:errors.append('Unexpected real robot connection/motion')
        record=preview_record(workspace.vision_camera.path)
        if record:stamps.add(record['timestamp'])
        elapsed=time.monotonic()-start
        for mode,at in [('REAL',2),('SIM+REAL',3),('SIM',4)]:
            if elapsed>at and mode not in visited:
                workspace.key_panel_modes.mode.set(mode);workspace.key_panel_modes.change();visited.append(mode)
        if elapsed>4 and 'map' not in visited:
            app.tabs.select(app.operation_page);visited.append('map');app.after(250,lambda:app.tabs.select(app.vision_panel_page))
        if elapsed>6 and (not args.camera or len(stamps)>=3) or elapsed>18:
            if args.camera and len(stamps)<3:errors.append('No changing live D455 preview frames')
            app.tabs.select(app.vision_panel_page);app.update_idletasks()
            path=Path(__file__).resolve().parents[1]/'.delivery'
            capture_window(app,path/'moma_vision_integrated.png')
            result=dict(gui_errors=errors,embedded=workspace.winfo_toplevel() is app,
                camera_requested=args.camera,live_preview_frames=len(stamps),tabs=[app.tabs.tab(t,'text') for t in app.tabs.tabs()],
                mode=link.mode,physical_robot_connected=link.client.connected,real_motion_active=link.plan is not None,
                sim_pose=list(workspace.key_panel_link.twin.sim.q),visited_existing_map='map' in visited,visited_modes=[m for m in visited if m!='map'])
            (path/'moma_vision_verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
            app.close();return
        app.after(150,run)
    app.after(700,run);app.mainloop()
    if errors:raise RuntimeError('; '.join(errors))


if __name__=='__main__':main()
