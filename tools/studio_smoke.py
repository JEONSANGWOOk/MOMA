"""Desktop integration smoke; temporary settings only, no robot/network access."""
import sys
import math
import tempfile
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as app_module
import seer_control.studio_ui as ui_module
from seer_control.smap import make_path_record, path_record_geometry

with tempfile.TemporaryDirectory() as directory:
    app_module.USER_DIR=Path(directory)
    ui_module.SETTINGS=Path(directory)/'studio_settings.json'
    app=app_module.Console()
    errors=[]
    app.report_callback_exception=lambda typ,value,tb:errors.append(str(value))
    def fail_dialog(title,message,**kwargs):raise AssertionError(f'{title}: {message}')
    app_module.messagebox.showerror=fail_dialog
    try:
        app.pose_autosave.set(False)
        app.update()
        assert len(app.tabs.tabs())==8
        def button_texts(widget):
            found=[]
            if widget.winfo_class()=='Button':found.append(widget.cget('text'))
            for child in widget.winfo_children():found.extend(button_texts(child))
            return found
        operation_buttons=button_texts(app.operation_page)
        editor_buttons=button_texts(app.nodes_page)
        for title in ('로컬 JSON','로컬 SMAP','저장','맵 생성 시작','맵 생성 종료','스캔 CSV 저장'):
            assert title not in operation_buttons and editor_buttons.count(title)==1
        # Navigation and layout must remain usable on common laptop/desktop sizes.
        assert len(app.navigation_buttons)==8
        for width,height in ((1366,768),(1920,1080)):
            app.geometry(f'{width}x{height}+0+0');app.update()
            app.tabs.select(app.operation_page);app.update()
            assert app.canvas.winfo_width()>500
            assert app.canvas.winfo_height()>250
            assert app.workspace_title.cget('text')=='지도 / 제어'
            if '--capture' in sys.argv:
                from window_capture import capture_window
                capture_window(app,Path(__file__).resolve().parents[1]/'artifacts'/f'workspace_{width}.png')
        # Create a curved route in the active map and inspect actual Tk canvas geometry.
        a,b='LM1','LM2'
        raw=make_path_record(app.map.nodes[a],app.map.nodes[b])
        raw['controlPos1']['y']=1.;raw['controlPos2']['y']=1.
        rec=dict(a=a,b=b,raw=raw,properties={},controls=[(raw[k]['x'],raw[k]['y']) for k in ('controlPos1','controlPos2')])
        app.map.path_records=[]
        for u,v in app.map.edges:
            for src,dst in ((u,v),(v,u)):
                r=make_path_record(app.map.nodes[src],app.map.nodes[dst])
                app.map.path_records.append(dict(a=src,b=dst,raw=r,properties={},controls=[(r[k]['x'],r[k]['y']) for k in ('controlPos1','controlPos2')]))
        rec=next(r for r in app.map.path_records if r['a']==a and r['b']==b)
        app._set_curve_controls(rec,[(3.5,1),(5.5,1)])
        app.refresh_nodes()
        # The editor owns an interactive map using the same model and handlers.
        for width,height in ((1366,768),(1920,1080)):
            app.geometry(f'{width}x{height}+0+0')
            app.tabs.select(app.nodes_page);app.update()
            assert app.canvas is app.editor_canvas
            assert app.editor_canvas.winfo_width()>450 and app.editor_canvas.winfo_height()>250
            app.node_tree.selection_set('LM2');app.update()
            assert app.selected=='LM2' and 'LM2' in app.node_prop_text.cget('text')
            app.draw_map();px,py=app.xy(app.map.nodes['LM1']['x'],app.map.nodes['LM1']['y'])
            app.map_click(SimpleNamespace(x=px,y=py));app.update()
            assert app.selected=='LM1' and 'LM1' in app.node_tree.selection()
            geometry=path_record_geometry(rec['raw'])
            px,py=app.xy(*geometry[len(geometry)//2])
            app.map_click(SimpleNamespace(x=px,y=py));app.update()
            assert app.editor_selection_kind=='path'
            assert app.editor_canvas.find_withtag('selected_editor_path')
            assert app.editor_lists.index(app.editor_lists.select())==1
            labels=app.editor_canvas.find_withtag('map_label')
            node_labels=app.editor_canvas.find_withtag('node_label')
            assert len(node_labels)==len(app.map.nodes),'Demo node IDs must all remain visible'
            boxes=[app.editor_canvas.bbox(item) for item in labels]
            for index,box in enumerate(boxes):
                for other in boxes[index+1:]:
                    assert not (box[0]<other[2] and box[2]>other[0] and box[1]<other[3] and box[3]>other[1]),'Map labels overlap'
            assert len(app.editor_canvas.find_withtag('selected_path_diagnostic'))==1,'Only the selected path should have diagnostic text'
            if '--capture' in sys.argv:
                from window_capture import capture_window
                capture_window(app,Path(__file__).resolve().parents[1]/'artifacts'/f'node_editor_{width}.png')
        app.edit_mode.set('곡선 편집')
        app.curve_edit_record=rec;app.draw_map()
        for width,height in ((1366,768),(1920,1080)):
            app.geometry(f'{width}x{height}+0+0');app.update();app.draw_map()
            notices=app.canvas.find_withtag('map_notice')
            assert app.canvas.find_withtag('curve_edit_notice')
            boxes=sorted((app.canvas.bbox(item) for item in notices),key=lambda box:box[1])
            assert all(a[3]<b[1] for a,b in zip(boxes,boxes[1:])), 'Map header notices overlap vertically'
            if '--capture' in sys.argv:
                capture_window(app,Path(__file__).resolve().parents[1]/'artifacts'/f'curve_editor_{width}.png')
        px,py=app.xy(*rec['controls'][0]);app.map_click(SimpleNamespace(x=px,y=py))
        px,py=app.xy(3.5,.8);app.map_drag(SimpleNamespace(x=px,y=py));app.map_release()
        assert abs(rec['controls'][0][1]-.8)<.001
        app._set_curve_controls(rec,[(3.5,1),(5.5,1)])
        app.edit_mode.set('선택')
        app.tabs.select(app.operation_page);app.update()
        assert app.canvas is app.operation_canvas
        # A retained editor tool must never modify the map from the driving view.
        app.edit_mode.set('Point 추가');count=len(app.map.nodes)
        app.map_click(SimpleNamespace(x=30,y=150))
        assert len(app.map.nodes)==count
        app.edit_mode.set('곡선 편집');app.draw_map()
        assert not app.canvas.find_withtag('curve_edit_notice')
        app.edit_mode.set('선택')
        # Loop setup has its own viewport; map double-click appends a stop.
        app._select_node('LM1');app.quick_loop_mission();app.update()
        dialog=next(child for child in app.winfo_children() if child.winfo_class()=='Toplevel' and child.title()=='순환 미션 만들기')
        preview=dialog.preview
        assert preview.winfo_width()>250 and preview.winfo_height()>150
        assert list(dialog.route_list.get(0,'end'))==['LM1']
        px,py=preview.xy(app.map.nodes['LM2']['x'],app.map.nodes['LM2']['y'])
        preview.click(SimpleNamespace(x=px,y=py),True);app.update()
        assert list(dialog.route_list.get(0,'end'))==['LM1','LM2']
        assert preview.find_withtag('mission_route')
        assert preview.find_withtag('preview_robot')
        def find_button(widget,text):
            if widget.winfo_class()=='Button' and widget.cget('text')==text:return widget
            for child in widget.winfo_children():
                result=find_button(child,text)
                if result:return result
        find_button(dialog,'시작점으로 닫기').invoke();app.update()
        assert list(dialog.route_list.get(0,'end'))==['LM1','LM2','LM1']
        assert preview.itemcget(preview.find_withtag('preview_status')[0],'text').startswith('순환 완성')
        assert app.canvas is app.operation_canvas,'Preview must not replace main canvas'
        if '--capture' in sys.argv:
            capture_window(dialog,Path(__file__).resolve().parents[1]/'artifacts'/'loop_mission_preview.png')
        dialog.route_list.delete(0,'end')
        for key in ('LM8','LM3','LM7','LM7','LM8'):dialog.route_list.insert('end',key)
        preview.redraw();app.update()
        assert not preview.plan['errors'],preview.plan['errors']
        assert len(preview.plan['nodes'])>len(preview.plan['stops'])
        assert len(preview.find_withtag('mission_route'))==sum(len(leg['nodes'])-1 for leg in preview.plan['legs'])
        assert not preview.find_withtag('missing_mission_path')
        dialog.route_policy.set('예상 시간 우선');app.update()
        assert preview.plan['policy']=='time'
        dialog.route_policy.set('우회 (제외 노드)');dialog.excluded_nodes.set('LM7');app.update()
        assert preview.plan['errors'] and preview.find_withtag('missing_mission_path')
        assert all(preview.type(item)!='line' for item in preview.find_withtag('missing_mission_path')),'Unreachable legs must never draw invented diagonals'
        dialog.excluded_nodes.set('LM2');app.update()
        assert not preview.plan['errors'] and 'LM2' not in preview.plan['nodes']
        if '--capture' in sys.argv:
            capture_window(dialog,Path(__file__).resolve().parents[1]/'artifacts'/'loop_route_planning.png')
        # Generate and execute multi-hop legs as single destination actions.
        from seer_control.route_planner import plan_actions
        planned=preview.plan
        actions=plan_actions(planned)
        assert [a['goal'] for a in actions]==['LM3','LM7','LM8']
        assert len(actions[0]['route_nodes'])>2
        app.sim.state.x=app.map.nodes['LM8']['x'];app.sim.state.y=app.map.nodes['LM8']['y']
        app.sim.state.last_node='LM8';app.sim.state.theta=0
        app.task_chains=[dict(name='Pass-through loop',repeat_count=1,tasks=[dict(groups=[dict(actions=actions)])])]
        app.active_chain=0;app.run_tasks()
        transit_speeds=[]
        for index in range(2000):
            app.sim.tick(.1)
            if app.sim.state.last_node=='LM6' and app.sim.route and app.sim.state.target=='LM3':transit_speeds.append(abs(app.sim.state.speed))
            app._studio_tick(5000.+index*.1,.1)
        assert transit_speeds and min(transit_speeds)>0,transit_speeds
        assert app.studio_runner.status=='COMPLETED',app.studio_runner.error
        assert app.sim.state.last_node=='LM8' and app.sim.state.speed==0
        app.sim.state.x=app.map.nodes['LM1']['x'];app.sim.state.y=app.map.nodes['LM1']['y']
        app.sim.state.last_node='LM1';app.sim.state.theta=0
        dialog.route_policy.set('최단 거리');dialog.excluded_nodes.set('')
        dialog.route_list.delete(0,'end')
        for key in ('LM1','LM2','LM1'):dialog.route_list.insert('end',key)
        find_button(dialog,'순환 미션 생성').invoke();app.update()
        assert app.task_chains[-1]['loop_route']==['LM1','LM2','LM1']
        assert not dialog.winfo_exists()
        app.run_tasks()
        for index in range(600):
            app.sim.tick(.1);app._studio_tick(2000.+index*.1,.1)
        assert app.studio_runner.status=='COMPLETED',app.studio_runner.error
        assert app.sim.state.last_node=='LM1'
        app.tabs.select(app.operation_page);app.update()
        app.sim.navigate('LM2');app.draw_map();app.update()
        lines=[app.canvas.coords(i) for i in app.canvas.find_all() if app.canvas.type(i)=='line' and app.canvas.itemcget(i,'fill')=='#51d8b6']
        assert lines and len(lines[0])>8,'Green path must contain curve samples'
        assert max(lines[0][1::2])-min(lines[0][1::2])>5,'Green path is still straight'
        app.sim.stop()
        # A complete Taskchain including hardware-independent actions.
        app.task_chains=[dict(name='Smoke',tasks=[dict(groups=[dict(actions=[
            dict(type='Set DO',channel=1,value=True),dict(type='Translation',distance_m=.1,speed_mps=.1),
            dict(type='Rotation',angle_deg=30,speed_dps=30),dict(type='Wait',duration_s=.2),
            dict(type='Arm Action',operation='work',duration_s=.2),dict(type='Arm Action',operation='safe_pose',duration_s=.2)])])])]
        app.active_chain=0;app.run_tasks()
        now=1000.
        for i in range(100):app.sim.tick(.1);app._studio_tick(now+i*.1,.1)
        assert app.studio_runner.status=='COMPLETED',app.studio_runner.error
        assert app.sim.do[1] and app.studio_arm_safe
        # Dynamic obstacle placement/removal through the actual UI handler while navigating.
        app.tabs.select(app.nodes_page);app.update()
        app.edit_mode.set('SIM 장애물');app.sim.navigate('LM3')
        app.draw_map();px,py=app.xy(4.5,1.2);event=SimpleNamespace(x=px,y=py)
        assert app._studio_map_click(event)
        for _ in range(400):app.sim.tick(.1)
        assert app.sim.state.blocked
        app.draw_map();px,py=app.xy(4.5,1.2)
        app._studio_map_click(SimpleNamespace(x=px,y=py))
        assert not app.map.obstacles
        for _ in range(400):app.sim.tick(.1)
        assert app.sim.state.last_node=='LM3' and not app.sim.state.blocked
        app._update_location_display(app.sim.status(),now)
        assert app.location_display['last']=='LM3'
        assert '직전 LM3' in app.location_nodes.cget('text')
        assert 'X ' in app.location_coordinates.cget('text')
        app.draw_map()
        assert app.canvas.find_withtag('robot_location_caption')
        app.edit_mode.set('선택')
        # All main tabs and nested extension tabs can be selected and rendered.
        for tab_id in app.tabs.tabs():app.tabs.select(tab_id);app.update()
        extension=app.tabs.nametowidget(app.tabs.tabs()[-1])
        sub=extension.winfo_children()[0]
        for tab_id in sub.tabs():sub.select(tab_id);app.update()
        # Editor undo/redo restores properties and geometry.
        app._studio_edit(lambda:app.map.obstacles.append(dict(x=4,y=0,radius=.25)),'Smoke obstacle')
        app._studio_undo(False);assert not app.map.obstacles
        app._studio_undo(True);assert app.map.obstacles
        # Model configuration save and calibration use the isolated settings path.
        app._studio_calculate();app._studio_apply_calibration()
        assert ui_module.SETTINGS.exists()
        # 3D uses the same live map and AMR pose; 2D remains available for editing.
        app.tabs.select(app.operation_page);app.geometry('1366x768+0+0');app.update()
        app.view_mode.set('3D');app._spatial_switch();app.update();app.draw_map()
        view=app.world3d
        assert view.winfo_ismapped() and not app.operation_canvas.winfo_ismapped()
        for tag in ('world_amr','world_arm','world_map','world_path'):assert view.find_withtag(tag),tag
        assert view.scene_pose['x']==app.sim.status()['x']
        # 3D clicks use the same destination selection and navigation handlers.
        node=app.map.nodes['LM2'];sx,sy=view.map_xy(node['x'],node['y'])
        event=SimpleNamespace(x=sx,y=sy)
        yaw=view.camera.yaw;view.begin_drag(event,'orbit');view.end_drag(event)
        assert app.selected=='LM2' and app.target.get()=='LM2'
        assert view.camera.yaw==yaw and view.find_withtag('world_selected_node')
        original_confirm=app_module.messagebox.askyesno
        original_navigate=app.navigate;goals=[]
        app_module.messagebox.askyesno=lambda *args,**kwargs:True
        app.navigate=lambda:goals.append(app.target.get())
        try:view.double_click(event);assert goals==['LM2']
        finally:app.navigate=original_navigate;app_module.messagebox.askyesno=original_confirm
        sent=[];original_send=app._send_reloc
        app._send_reloc=lambda x,y,angle=None:sent.append((x,y,angle))
        try:
            app.reloc_mode='manual';view.begin_drag(event,'orbit')
            ex,ey=view.map_xy(node['x']+.5,node['y']+.5)
            end=SimpleNamespace(x=ex,y=ey);view.drag_camera(end)
            assert view.find_withtag('world_reloc')
            view.end_drag(end);app.update()
            assert len(sent)==1 and abs(sent[0][2]-math.pi/4)<1e-6
            assert view.camera.yaw==yaw
            app.reloc_mode='auto';view.begin_drag(event,'orbit');view.end_drag(event);app.update()
            assert len(sent)==2 and sent[-1][2] is None
        finally:app._send_reloc=original_send;app.reloc_mode=None;app.reloc_candidate=None
        app.sim.arm.update(status='RUNNING',operation='work',progress=0.)
        app.draw_map();before=dict(view.scene_joint_values)
        app.sim.arm['progress']=1.;app.draw_map()
        assert before!=view.scene_joint_values
        if '--capture' in sys.argv:
            capture_window(app,Path(__file__).resolve().parents[1]/'artifacts/world3d_map.png')
        for preset in ('위','정면','측면','뒤','사선','로봇 추적'):
            app.camera_view.set(preset);app._spatial_camera();app.update()
            assert all(math.isfinite(v) for v in view.camera.target)
        if '--capture' in sys.argv:
            capture_window(app,Path(__file__).resolve().parents[1]/'artifacts/world3d_robot.png')
        app.projection_view.set('직교');app._spatial_projection();app.update()
        assert not view.camera.perspective
        view.begin_drag(SimpleNamespace(x=100,y=100),'orbit')
        yaw=view.camera.yaw;view.drag_camera(SimpleNamespace(x=130,y=120));assert view.camera.yaw!=yaw
        asset=view.load_asset('arm',app._spatial_demo_path());assert len(asset.movable())==6
        view.arm_follow.set(False);view.positions['arm']['joint_2']=.8;app.draw_map()
        assert view.scene_joint_values['joint_2']==.8
        mesh=Path(directory)/'cad.obj';mesh.write_text('v 0 0 0\nv 1 0 0\nv 0 1 1\nf 1 2 3',encoding='utf-8')
        view.load_asset('cad',mesh);view.follow.set(False);view.camera.target=[0,0,.5];view.camera.distance=5
        app.draw_map();assert view.find_withtag('world_cad')
        dialog=app._spatial_dialog();app.update()
        assert len(dialog.joint_frame.winfo_children())==6
        dialog.destroy();app._spatial_save()
        app.view_mode.set('2D');app._spatial_switch();app.update()
        assert app.operation_canvas.winfo_ismapped() and not view.winfo_ismapped()
        # Simulated pad samples exercise the real UI bridge without hardware commands.
        from seer_control.gamepad import Sample
        import time
        if app.pad_after:app.after_cancel(app.pad_after);app.pad_after=None
        app.tabs.select(app.operation_page);app.focus_force();app.operation_canvas.focus_set();app.update()
        app.sim.stop();app.sim.arm.update(status='IDLE',pose='SAFE');app.task_running=False;app.manual.set(True);app.pad_enabled.set(True)
        app.speed.set('.15');app.angular.set('20');app.studio_arm_safe=True
        now=time.monotonic()
        def pad(x=0,y=0,buttons=0):return Sample({'X':x,'Y':y},buttons,now,('test-pad',))
        app._pad_process(pad(),now);app._pad_process(pad(y=-1,buttons=16),now)
        assert app.held and app.held[0]=='gamepad' and abs(app.sim.v-.15)<1e-6
        old=(app.sim.state.x,app.sim.state.y);app.sim.tick(.1)
        assert old!=(app.sim.state.x,app.sim.state.y)
        app._pad_process(pad(x=1,y=-1,buttons=16),now)
        assert app.sim.v>0 and app.sim.w<0
        app._pad_process(pad(),now);assert app.held is None and app.sim.v==app.sim.w==0
        app._pad_process(pad(y=-1,buttons=16),now);app._pad_process(None,now)
        assert app.held is None and app.sim.v==0
        app._pad_process(pad(y=-1,buttons=16),now);assert app.held is None
        app._pad_process(pad(),now);app._pad_process(pad(buttons=4),now)
        assert not app.pad_enabled.get() and app.held is None
        # Real mode stages a leased velocity only: no live network transport in this test.
        app.real=True;app.control_enabled=True;app.last_state=now;app.live['emergency']=False
        app.pad_enabled.set(True);app.pad_gate.reset()
        app._pad_process(pad(),now);app._pad_process(pad(y=-1,buttons=16),now)
        assert app._jog_desired['vx']==.15 and app._jog_desired['_expires']==now+.25
        app._pad_process(None,now);assert app._jog_desired is None
        app.real=False;app.manual.set(False);app.pad_enabled.set(False)
        dialog=app._pad_dialog();app.update();dialog.destroy()
        assert not errors,errors
        print('Studio GUI smoke: PASS (8 tool tabs, node/path editor map and list sync, curve drag, curved green canvas path, full Actions, dynamic obstacle stop/resume, undo/redo, calibration)')
    finally:
        app.studio_bridge.close();app.destroy()
