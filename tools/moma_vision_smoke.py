"""Exercise the real MOMA host and optional read-only D455 capture; never moves FR5."""
import argparse,json,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from seer_control.app import Console
from seer_control.vision_camera_ui import preview_record
from tools.window_capture import capture_window


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--camera',action='store_true');parser.add_argument('--model-preview',action='store_true');parser.add_argument('--live-start-test',action='store_true');parser.add_argument('--photo-screen-test',action='store_true');parser.add_argument('--marker-only-test',action='store_true');args=parser.parse_args()
    app=Console();errors=[];stamps=set();start=time.monotonic();visited=[];live_routing_verified=False;photo_verified=False;marker_verified=False
    def callback_error(typ,value,tb):
        import traceback
        errors.append(str(value));traceback.print_exception(typ,value,tb)
    app.report_callback_exception=callback_error
    app.after(100,lambda:app.tabs.select(app.vision_panel_page))
    def run():
        nonlocal live_routing_verified,photo_verified,marker_verified
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
                if mode=='SIM' and args.model_preview:workspace.key_model_preview()
        if elapsed>5 and args.live_start_test and not live_routing_verified:
            workspace.key_model_preview();workspace.key_panel_begin()
            if workspace.key_vision_source.get()!='D455 실시간' or not workspace.key_live_pending():errors.append('Preview start did not switch to pending live D455 tracking')
            workspace.key_panel_stop()
            if workspace.key_live_pending():errors.append('Stop did not cancel live tracking start')
            live_routing_verified=not errors
        if elapsed>5 and args.photo_screen_test and not photo_verified:
            workspace.key_photo_start()
            if workspace.key_vision_source.get()!='사진 화면 추종 (SIM)' or not workspace.key_live_pending():errors.append('Photo SIM start did not enter photo-only pending tracking')
            workspace.key_panel_stop()
            if workspace.key_live_pending():errors.append('Stop did not cancel photo tracking start')
            window=workspace.vision_camera.show_product_photo()
            if window is None:errors.append('Photo display did not open')
            else:window.destroy()
            photo_verified=not errors
        if elapsed>5 and args.marker_only_test and not marker_verified:
            if workspace.key_hybrid_enabled.get():errors.append('Marker-only tracking is not the default')
            workspace.key_live_start()
            if workspace.key_vision_source.get()!='D455 실시간' or not workspace.key_live_pending():errors.append('Marker-only start did not wait for live markers')
            workspace.key_hybrid_enabled.set(True);workspace.key_change_tracking()
            if not workspace.key_hybrid_enabled.get() or workspace.key_live_pending():errors.append('Hybrid selection did not cancel pending marker start')
            workspace.key_hybrid_enabled.set(False);workspace.key_change_tracking()
            if workspace.key_hybrid_enabled.get():errors.append('Cannot return to marker-only tracking')
            workspace.key_panel_stop();marker_verified=not errors
        if elapsed>4 and 'map' not in visited:
            app.tabs.select(app.operation_page);visited.append('map');app.after(250,lambda:app.tabs.select(app.vision_panel_page))
        if elapsed>6 and (not args.camera or len(stamps)>=3) or elapsed>18:
            if args.camera and len(stamps)<3:errors.append('No changing live D455 preview frames')
            app.tabs.select(app.vision_panel_page);app.update_idletasks()
            path=Path(__file__).resolve().parents[1]/'.delivery'
            capture_window(app,path/'moma_vision_integrated.png')
            result=dict(gui_errors=errors,embedded=workspace.winfo_toplevel() is app,
                camera_requested=args.camera,live_preview_frames=len(stamps),live_start_routing_verified=live_routing_verified,photo_screen_verified=photo_verified,marker_only_verified=marker_verified,tabs=[app.tabs.tab(t,'text') for t in app.tabs.tabs()],
                mode=link.mode,physical_robot_connected=link.client.connected,real_motion_active=link.plan is not None,
                sim_pose=list(workspace.key_panel_link.twin.sim.q),visited_existing_map='map' in visited,visited_modes=[m for m in visited if m!='map'])
            (path/'moma_vision_verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
            app.close();return
        app.after(150,run)
    app.after(700,run);app.mainloop()
    if errors:raise RuntimeError('; '.join(errors))


if __name__=='__main__':main()
