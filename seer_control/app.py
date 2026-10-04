"""Tk desktop console. Tk main thread owns simulation and every widget."""
import csv
import json
import math
import os
import platform
import queue
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, simpledialog
from .model import MapModel, Simulator, number
from .transport import port_probe, PersistentSeerSession
from .live import SeerClient, points, laser_points, extract_laser_points, extract_laser_scans, COMMANDS
from .smap import load_smap, load_smap_data, build_smap_from_model, validate_smap_edit, make_path_record, make_area_record, path_record_geometry
from .pose_recovery import normalized_pose_record, save_pose, load_pose, map_matches, confidence_ok, nearest_reachable_node
from .studio_ui import StudioMixin
from .spatial_ui import SpatialMixin
from .gamepad_ui import GamepadMixin
from .gamepad import jog_packet
from .manual_safety import controller_manual_reason
from .studio_core import ACTION_DEFAULTS, validate_actions, point_segment_distance
from .location import LocationTracker
from .mission_preview import MissionMapPreview
from .route_planner import plan_stops, plan_actions
from .ui_scale import UIScaleMixin

ROOT = Path(getattr(sys, '_MEIPASS', Path(__file__).resolve().parents[1]))
from .theme import BG, WHITE, INK, MUTED, PANEL, BLUE, GREEN, RED, ORANGE, DARK, GRID, CHROME, configure_styles
USER_DIR = Path.home()/'.seer_amr_console'


def _shape_summary(value, depth=0):
    """Small, non-huge diagnostic summary for unknown SEER sensor payloads."""
    if depth > 2:
        return type(value).__name__
    if isinstance(value, dict):
        out={}
        for k,v in list(value.items())[:12]:
            if isinstance(v,(dict,list)):
                out[k]=_shape_summary(v, depth+1)
            else:
                out[k]=type(v).__name__
        return out
    if isinstance(value, list):
        if not value:
            return 'list[0]'
        return {'len':len(value),'item0':_shape_summary(value[0], depth+1)}
    return type(value).__name__


class Console(UIScaleMixin, GamepadMixin, SpatialMixin, StudioMixin, tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('AMR Control Studio | MoMa Standalone · v1.16 Workspace')
        # Responsive startup size: fit the active monitor instead of assuming one fixed resolution.
        sw=self.winfo_screenwidth(); sh=self.winfo_screenheight()
        start_w=min(1600,int(sw*.94))
        start_h=min(1000,int(sh*.88))
        self.geometry(f'{start_w}x{start_h}+{max(0,(sw-start_w)//2)}+{max(0,(sh-start_h)//2)}')
        self.minsize(min(800,sw-40), min(480,sh-80))
        self.configure(bg=BG)
        self.font = 'Malgun Gothic' if platform.system() == 'Windows' else 'Noto Sans CJK KR'
        self.option_add('*Font', (self.font, 9))
        self.option_add('*Checkbutton.selectColor', WHITE)
        self.option_add('*Checkbutton.foreground', INK)
        style = ttk.Style(self)
        configure_styles(style, self.font)
        self.map = MapModel.load(ROOT/'maps/demo.json')
        self.sim = Simulator(self.map)
        self.connected = True
        self.real = False
        self.commands = queue.Queue(maxsize=4)
        self.control_enabled = False
        self.robot_stations = {}
        self.loading_map = False
        self.map_cloud = []
        self.current_robot_map = ''
        # SEER 4005/4006 require the same non-empty nick_name in the request body.
        self.control_nick = 'SEER_AMR_Control'
        self.downloading_map = False
        self.live, self.raw = {}, {}
        self.last_state = 0.0
        self.profile = ROOT/'config/api_profile.json'
        self.map_path = ROOT/'maps/demo.json'
        self.events = queue.Queue()
        self.worker_stop = None
        self.generation = 0
        self.logs = []
        self.tasks = []
        self.task_index = None
        self.task_running = False
        self.mission_repeat_target = 1  # 0 = infinite
        self.mission_repeat_done = 0
        self.mission_seen_active = False
        self.mission_goal_sent_at = 0.0
        self.mission_wait_until = 0.0
        self.mission_pending_send = False
        self.mission_cycle_name = ''
        self.selected = 'LM4'
        self.zoom = 1.0
        self.pan = [0.0, 0.0]
        self.drag = None
        self.pending_link = None
        self.heading_drag_key = None
        self.curve_edit_record = None
        self.curve_drag_index = None
        self.reloc_mode = None          # None | 'manual' | 'auto'
        self.reloc_drag_start = None
        self.reloc_candidate = None     # (x,y,angle|None)
        self.area_drag_start = None
        self.area_drag_current = None
        self.selected_area = None
        self.task_chains = []
        self.active_chain = None
        self.active_task = None
        self.active_group = None
        self.held = None
        self.scan_points = []
        self.real_laser_points = []
        self.real_laser_scans = []
        self.last_laser_rx = 0.0
        self.last_sensor1101_rx = 0.0
        self._sensor1101_logged = False
        self.lidar_source = 'NONE'
        self.lidar_raw_count = 0
        self.lidar_last_diag = 0.0
        self.block_status = {}
        self._laser_shape_logged = False
        self.lidar_mount_overrides = []
        self._lidar_calibration_running = False
        self._lidar_calibrated_for_map = ''
        self.lidar_map_alignment = (0.0,0.0,0.0)
        self.lidar_alignment_score = 0.0
        self._lidar_alignment_running = False
        self._lidar_alignment_map = ''
        self.last_scan = 0
        self._jog_lock = threading.Lock()
        self._jog_desired = None
        self._jog_wakeup = None
        self._jog_last_report = 0.0
        self._last_operation_draw = 0.0
        self._map_prepare_token = 0
        self.map_dirty = False
        self.map_push_in_progress = False
        # Persistent last-pose recovery (real + simulator)
        self.pose_autosave = tk.BooleanVar(value=True)
        self.pose_autorecover = tk.BooleanVar(value=False)
        self.pose_autoconfirm = tk.BooleanVar(value=False)
        self.pose_conf_threshold = tk.StringVar(value='0.80')
        self.pose_resume_loop = tk.BooleanVar(value=False)
        self._last_pose_save_at = 0.0
        self._last_pose_prompted_key = None
        self._pending_pose_recovery = None
        self.sim_powered = True
        self.real_mapping_active = False
        self.real_mapping_cloud = []
        self.real_mapping_trace = []
        self._real_mapping_cells = set()
        self._mapping_laser_diag_at = 0.0
        self.auto_apply_uploaded_map = tk.BooleanVar(value=True)
        self.edit_mode = tk.StringVar(value='선택')
        self.mode = tk.StringVar(value='시뮬레이션')
        self.host = tk.StringVar(value='192.168.58.102')
        self.manual = tk.BooleanVar(value=False)
        self.task_goal = tk.StringVar(value='LM4')
        self.speed = tk.StringVar(value='0.10')
        self.angular = tk.StringVar(value='15')
        self.layers = {k:tk.BooleanVar(value=(k != 'LiDAR')) for k in ['그리드','벽','영역','노드','경로','장애물','LiDAR']}
        self.location_tracker = LocationTracker()
        self.location_display = {}
        self._build()
        self.bind('<KeyPress>', self.key_press, add='+')
        self.bind('<KeyRelease>', self.key_release, add='+')
        self.tabs.bind('<<NotebookTabChanged>>',lambda e:self.release_drive(),add='+')
        self.bind_all('<ButtonRelease-1>', self.release_drive, add='+')
        self.bind('<FocusOut>', lambda e:self.after_idle(self.focus_guard), add='+')
        self.bind('<Escape>', lambda e:self.action('stop'))
        self.protocol('WM_DELETE_WINDOW', self.close)
        self.refresh_nodes()
        self._update_pose_status()
        self._startup_pose_recovery_check()
        self.log('INFO', '시뮬레이션 시작 · 표시되는 로봇/센서 정보는 합성 데이터입니다.')
        self._studio_init()
        self._obstacles_restore()
        self._ui_scale_init()
        self._spatial_restore()
        self._pad_start()
        self.last_tick = time.monotonic()
        self.after(100, self.tick)

    def label(self, parent, text='', size=10, color=INK, bold=False, bg=PANEL, **kw):
        return tk.Label(parent, text=text, bg=bg, fg=color,
                        font=(self.font, size, 'bold' if bold else 'normal'), **kw)

    def button(self, parent, text, command, color=None, **kw):
        kw.setdefault('pady', 4)
        return tk.Button(parent, text=text, command=command, bg=color or '#e2e8f0',
                         fg=WHITE if color else INK, activebackground=color or '#d0e1f4',
                         activeforeground=WHITE if color else INK, relief='flat', bd=0,
                         padx=8, cursor='hand2', **kw)

    def card(self, parent, title):
        f = tk.Frame(parent, bg=PANEL, highlightbackground='#ccd5e0', highlightthickness=1)
        self.label(f, title, bold=True).pack(anchor='w', padx=10, pady=(7, 5))
        body = tk.Frame(f, bg=PANEL)
        body.pack(fill='both', expand=True, padx=10, pady=(0, 8))
        return f, body

    def _fit_dialog(self, win, width, height, min_width=420, min_height=320):
        """Keep dialogs fully visible on 1366x768 through large monitors."""
        sw=max(800,win.winfo_screenwidth()); sh=max(600,win.winfo_screenheight())
        w=min(int(width), max(min_width, sw-80)); h=min(int(height), max(min_height, sh-120))
        x=max(0,(sw-w)//2); y=max(0,(sh-h)//3)
        win.geometry(f'{w}x{h}+{x}+{y}')
        try: win.minsize(min(min_width,w), min(min_height,h))
        except Exception: pass
        return w,h

    def _scrollable_frame(self, parent, bg=PANEL):
        """Create a vertically scrollable frame for short displays and high DPI scaling."""
        outer=tk.Frame(parent,bg=bg)
        canvas=tk.Canvas(outer,bg=bg,highlightthickness=0,bd=0,width=1,height=1)
        sb=ttk.Scrollbar(outer,orient='vertical',command=canvas.yview)
        content=tk.Frame(canvas,bg=bg)
        window=canvas.create_window((0,0),window=content,anchor='nw')
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side='left',fill='both',expand=True)
        sb.pack(side='right',fill='y')
        def sync(_=None):
            canvas.configure(scrollregion=canvas.bbox('all'))
            try: canvas.itemconfigure(window,width=canvas.winfo_width())
            except Exception: pass
        content.bind('<Configure>',sync)
        canvas.bind('<Configure>',sync)
        def wheel(e):
            delta=-1 if getattr(e,'delta',0)>0 else 1
            if getattr(e,'num',None)==4: delta=-1
            elif getattr(e,'num',None)==5: delta=1
            canvas.yview_scroll(delta*3,'units')
            return 'break'
        canvas.bind('<MouseWheel>',wheel,add='+'); canvas.bind('<Button-4>',wheel,add='+'); canvas.bind('<Button-5>',wheel,add='+')
        content.bind('<MouseWheel>',wheel,add='+'); content.bind('<Button-4>',wheel,add='+'); content.bind('<Button-5>',wheel,add='+')
        return outer,content,canvas

    def _responsive_layout(self, event=None):
        if event is not None and event.widget is not self:return
        self._ui_scale_schedule()
        w=max(1,self.winfo_width()); h=max(1,self.winfo_height())
        side=max(235,min(360,int(w*0.24)))
        for widget in (getattr(self,'selected_node_options',None),getattr(self,'command_status',None),getattr(self,'reloc_info',None),getattr(self,'node_prop_text',None)):
            try: widget.configure(wraplength=max(210,side-35))
            except Exception: pass

    def _register_navigation(self, page, title, symbol='◆'):
        button = tk.Button(self.navigation, text=f'{symbol}  {title}', anchor='w',
                           command=lambda: self.tabs.select(page), bg=CHROME, fg='#c4d0df',
                           activebackground='#354d69', activeforeground=WHITE,
                           relief='flat', bd=0, padx=12, pady=11, cursor='hand2')
        button.pack(fill='x', padx=5, pady=2)
        self.navigation_buttons[str(page)] = button

    def _sync_navigation(self, event=None):
        selected = self.tabs.select()
        for page, button in self.navigation_buttons.items():
            button.configure(bg=BLUE if page == selected else CHROME,
                             fg=WHITE if page == selected else '#c4d0df')
        if selected:
            self.workspace_title.configure(text=self.tabs.tab(selected, 'text'))
        if hasattr(self,'operation_canvas'):
            active=self.editor_canvas if selected==str(self.nodes_page) and hasattr(self,'editor_canvas') else self.operation_canvas
            if self.canvas is not active:
                self.release_drive()
                self.drag=None
                self.heading_drag_key=None
                self.curve_drag_index=None
                self._view_transform=None
                self.canvas=active
            self.after_idle(self.draw_map)

    def _build(self):
        top = tk.Frame(self, bg=CHROME)
        top.pack(fill='x')
        self.label(top, 'AMR STUDIO', 14, WHITE, True, bg=CHROME).pack(side='left', padx=15, pady=9)
        self.label(top, 'ROBOT WORKSPACE', 9, '#98acc5', bg=CHROME).pack(side='left', padx=8)
        self.badge = self.label(top, '● SIMULATION', 9, '#54ddb0', True, bg=CHROME)
        self.badge.pack(side='right', padx=16)
        bar = tk.Frame(self, bg=PANEL)
        bar.pack(fill='x')
        ttk.Combobox(bar, textvariable=self.mode, values=['시뮬레이터','실기 · 조회 전용','실기 · 제어'],
                     state='readonly', width=18).pack(side='left', padx=(12, 8), pady=6)
        self.label(bar, 'Robot IP').pack(side='left', padx=(4, 6))
        ttk.Entry(bar, textvariable=self.host, width=17).pack(side='left')
        self.button(bar, '연결', self.connect, BLUE).pack(side='left', padx=5)
        self.button(bar, '연결 해제', self.disconnect).pack(side='left')
        self.button(bar, '연결 설정 / 진단', lambda:self.tabs.select(self.settings_page)).pack(side='right', padx=10)
        self.connection_text = self.label(bar, 'DEMO 연결됨 · TCP 전송 없음', 9, MUTED)
        self.connection_text.pack(side='left', padx=10)
        self.cards = {}
        row = tk.Frame(self, bg=WHITE)
        row.pack(fill='x', pady=(1, 0))
        for i, (key, title) in enumerate([('mode','상태'), ('battery','배터리'),
                    ('pose','위치 · m'), ('localization','위치 신뢰도'),
                    ('safety','안전'), ('task','현재 작업')]):
            row.columnconfigure(i, weight=1, uniform='cards')
            f = tk.Frame(row, bg=WHITE, highlightbackground='#dce3ec', highlightthickness=1)
            f.grid(row=0,column=i,sticky='nsew')
            self.label(f,title,8,MUTED,bg=WHITE).pack(anchor='w',padx=10,pady=(4,0))
            value = self.label(f,'—',10,INK,True,bg=WHITE,anchor='w')
            value.pack(fill='x',padx=10,pady=(0,5))
            self.cards[key] = value
        footer = tk.Frame(self,bg=CHROME)
        footer.pack(fill='x',side='bottom')
        self.footer = self.label(footer,'',9,'#c4d0df',bg=CHROME,anchor='w')
        self.footer.pack(side='left',padx=8,pady=1)
        self.label(footer,'소프트웨어 정지 ≠ 하드웨어 비상정지',9,'#ffb6b9',bg=CHROME).pack(side='right',padx=12)
        shell = tk.Frame(self, bg=BG)
        shell.pack(fill='both',expand=True)
        self.navigation = tk.Frame(shell, bg=CHROME, width=150)
        self.navigation.pack(side='left',fill='y')
        self.navigation.pack_propagate(False)
        self.label(self.navigation, 'WORKSPACE', 8, '#98acc5', True, bg=CHROME).pack(anchor='w',padx=16,pady=(15,8))
        self.navigation_buttons = {}
        work = tk.Frame(shell, bg=BG)
        work.pack(side='left',fill='both',expand=True)
        titlebar = tk.Frame(work,bg=PANEL)
        titlebar.pack(fill='x')
        self.workspace_title = self.label(titlebar, '지도 / 제어', 11, INK, True)
        self.workspace_title.pack(side='left',padx=12,pady=7)
        self._ui_scale_control(titlebar)
        self.label(titlebar, '지도: 우클릭 이동 · 휠 확대 · Esc 정지', 8, MUTED).pack(side='right',padx=12)
        self.tabs = ttk.Notebook(work,style='Workspace.TNotebook')
        self.tabs.pack(fill='both',expand=True,padx=8,pady=(0,6))
        self.operation_page = ttk.Frame(self.tabs)
        self.nodes_page = ttk.Frame(self.tabs)
        self.tasks_page = ttk.Frame(self.tabs)
        self.telemetry_page = ttk.Frame(self.tabs)
        self.robot_info_page = ttk.Frame(self.tabs)
        self.settings_page = ttk.Frame(self.tabs)
        self.logs_page = ttk.Frame(self.tabs)
        for page, title, symbol in [(self.operation_page,'지도 / 제어','▦'), (self.nodes_page,'노드 / 경로','◇'),
               (self.tasks_page,'Taskchain','≡'),(self.telemetry_page,'실시간 데이터','⌁'),
               (self.robot_info_page,'로봇 운영 정보','◎'),(self.settings_page,'연결 / API','⚙'),(self.logs_page,'이벤트 로그','▤')]:
            self.tabs.add(page,text=title)
            self._register_navigation(page,title,symbol)
        self.tabs.bind('<<NotebookTabChanged>>',self._sync_navigation,add='+')
        self._operation()
        self._nodes()
        self._tasks()
        self._telemetry()
        self._robot_info()
        self._settings()
        self._logs()
        self._menu()
        self.bind('<Configure>',self._responsive_layout,add='+')
        self._sync_navigation()

    def _operation(self):
        page = self.operation_page
        page.columnconfigure(0,weight=4,minsize=450)
        page.columnconfigure(1,weight=1,minsize=270)
        page.rowconfigure(1,weight=1)
        ribbon = tk.Frame(page,bg=PANEL)
        ribbon.grid(row=0,column=0,columnspan=2,sticky='ew',pady=(3,0))
        self.label(ribbon,'로봇 운행 · 목적지 선택 / 주행 / 재배치',10,INK,True).pack(side='left',padx=10,pady=2)
        self.map_focus=False
        self.map_focus_button=self.button(ribbon,'지도 크게',self._map_focus_toggle)
        self.map_focus_button.pack(side='right',padx=3)
        self.map_focus_stop=self.button(ribbon,'■ 정지',lambda:self.action('stop'),RED)
        self.button(ribbon,'노드 / 경로 편집 열기',lambda:self.tabs.select(self.nodes_page)).pack(side='right',padx=6,pady=3)
        left = tk.Frame(page,bg=PANEL)
        left.grid(row=1,column=0,sticky='nsew',pady=2,padx=(0,3))
        toolbar = tk.Frame(left,bg=PANEL)
        toolbar.pack(fill='x',padx=6,pady=2)
        self.label(toolbar,'MAP',9,bold=True).pack(side='left',padx=(0,6))
        for title, command in [('맞춤',self.fit_map),('LiDAR 정합',self.align_lidar_to_map),
                              ('＋',lambda:self.change_zoom(1.2)),('−',lambda:self.change_zoom(1/1.2))]:
            self.button(toolbar,title,command).pack(side='left',padx=2)
        layers=tk.Frame(left,bg=PANEL);layers.pack(fill='x',padx=6,pady=(0,3))
        self.operation_layer_buttons={}
        for name,var in self.layers.items():
            button=tk.Checkbutton(layers,text=name,variable=var,bg=PANEL,command=self.draw_map)
            button.pack(side='left');self.operation_layer_buttons[name]=button
        self.operation_sensor_label=self.label(left,'LiDAR 대기 · 장애물 상태 확인 중',9,MUTED,anchor='w')
        self.operation_sensor_label.pack(fill='x',padx=8,pady=(0,1))
        viewport=tk.Frame(left,bg=PANEL)
        viewport.pack(fill="both",expand=True,padx=10)
        self.canvas = tk.Canvas(viewport,bg=DARK,highlightthickness=0)
        self.operation_canvas = self.canvas
        self.canvas.pack(fill='both',expand=True)
        self.canvas.bind('<Configure>',lambda e:self.draw_map() if self.canvas is self.operation_canvas else None)
        self.canvas.bind('<Button-1>',self.map_click)
        self.canvas.bind('<B1-Motion>',self.map_drag)
        self.canvas.bind('<ButtonRelease-1>',self.map_release)
        self.canvas.bind('<Double-Button-1>',self.map_double_click)
        self.canvas.bind('<ButtonPress-3>',self.pan_start)
        self.canvas.bind('<B3-Motion>',self.pan_move)
        self.canvas.bind('<MouseWheel>',lambda e:self.change_zoom(1.12 if e.delta>0 else 1/1.12))
        self.canvas.bind('<Button-4>',lambda e:self.change_zoom(1.12))
        self.canvas.bind('<Button-5>',lambda e:self.change_zoom(1/1.12))
        self._spatial_build(toolbar,viewport)
        info_bar=tk.Frame(left,bg=PANEL);info_bar.pack(fill='x',padx=6,pady=1)
        self.button(info_bar,'상세',self._map_info_dialog).pack(side='right')
        self.map_info = self.label(info_bar,'',8,MUTED,anchor='w')
        self.map_info.pack(side='left',fill='x',expand=True)
        self._map_info_details='지도 상태 요약'
        right_outer,right,_right_canvas = self._scrollable_frame(page,bg=BG)
        right_outer.grid(row=1,column=1,sticky='nsew',pady=5)
        self.operation_right_outer=right_outer
        self.operation_right_canvas=_right_canvas
        f,b = self.card(right,'즉시 제어')
        f.pack(fill='x',pady=(0,8))
        self.button(b,'■  소프트웨어 정지  [Esc]',lambda:self.action('stop'),RED).pack(fill='x')
        row = tk.Frame(b,bg=PANEL); row.pack(fill='x',pady=(6,0))
        for title,cmd in [('정지 해제','reset'),('Motor ON','motor_on'),('OFF','motor_off')]:
            self.button(row,title,lambda n=cmd:self.action(n)).pack(side='left',expand=True,fill='x',padx=1)
        f,b = self.card(right,'실시간 위치 / 통과 노드')
        f.pack(fill='x',pady=(0,8))
        self.location_nodes = self.label(b,'직전 —  →  다음 —',10,BLUE,True,anchor='w',justify='left',wraplength=290)
        self.location_nodes.pack(fill='x')
        self.location_coordinates = self.label(b,'X —  Y —  θ —',9,INK,anchor='w')
        self.location_coordinates.pack(fill='x',pady=(4,2))
        self.location_detail = self.label(b,'위치 수신 대기',8,MUTED,anchor='w',justify='left',wraplength=290)
        self.location_detail.pack(fill='x')
        self._obstacles_controls(right)
        f,b = self.card(right,'목적지 / Task')
        f.pack(fill='x',pady=(0,8))
        self.target = tk.StringVar(value='LM4')
        self.target_combo = ttk.Combobox(b,textvariable=self.target,state='readonly')
        self.target_combo.pack(fill='x')
        self.target_combo.bind('<<ComboboxSelected>>',self.target_changed)
        self.selected_node_label = self.label(b,'지도에서 노드를 클릭하세요',9,MUTED,anchor='w')
        self.selected_node_label.pack(fill='x',pady=(5,0))
        self.selected_node_options = self.label(b,'',8,'#52677f',anchor='w',justify='left',wraplength=300)
        self.button(b,'선택 노드로 이동',self.navigate,BLUE).pack(fill='x',pady=6)
        row = tk.Frame(b,bg=PANEL); row.pack(fill='x')
        for title,cmd in [('일시정지','pause'),('재개','resume'),('취소','cancel')]:
            self.button(row,title,lambda n=cmd:self.action(n)).pack(side='left',expand=True,fill='x',padx=1)
        self.button(b,'충전 노드로 복귀',self.return_to_charge,GREEN).pack(fill='x',pady=(6,0))
        self.button(b,'순환 미션 만들기',lambda:(self.tabs.select(self.tasks_page),self.quick_loop_mission()),ORANGE).pack(fill='x',pady=(6,0))
        f,b = self.card(right,'자기위치 재배치 / Relocate')
        f.pack(fill='x',pady=(0,8))
        rr=tk.Frame(b,bg=PANEL); rr.pack(fill='x')
        self.button(rr,'수동 재배치',self.start_manual_reloc,BLUE).pack(side='left',expand=True,fill='x',padx=(0,2))
        self.button(rr,'자동 재배치',self.start_auto_reloc,ORANGE).pack(side='left',expand=True,fill='x',padx=(2,0))
        self.reloc_info=self.label(b,'수동: 맵 클릭→드래그로 방향 지정 / 자동: 맵의 대략 위치 클릭→360° 방향 탐색',8,MUTED,anchor='w',justify='left',wraplength=300)
        self.reloc_info.pack(fill='x',pady=(5,4))
        rr2=tk.Frame(b,bg=PANEL); rr2.pack(fill='x')
        self.button(rr2,'위치 확정 (2003)',self.confirm_relocation,GREEN).pack(side='left',expand=True,fill='x',padx=(0,2))
        self.button(rr2,'취소 (2004)',self.cancel_relocation).pack(side='left',expand=True,fill='x',padx=(2,0))
        f,b = self.card(right,'마지막 위치 / 재기동 복구')
        f.pack(fill='x',pady=(0,8))
        self.pose_status=self.label(b,'저장 위치 없음',8,MUTED,anchor='w',justify='left',wraplength=300)
        self.pose_status.pack(fill='x',pady=(0,5))
        r=tk.Frame(b,bg=PANEL); r.pack(fill='x')
        self.button(r,'마지막 위치 보기',self.show_last_pose).pack(side='left',expand=True,fill='x',padx=(0,2))
        self.button(r,'마지막 위치로 재배치',self.recover_last_pose,BLUE).pack(side='left',expand=True,fill='x',padx=(2,0))
        r2=tk.Frame(b,bg=PANEL); r2.pack(fill='x',pady=(4,0))
        self.button(r2,'가까운 노드 찾기',self.find_nearest_node).pack(side='left',expand=True,fill='x',padx=(0,2))
        self.button(r2,'가까운 노드로 이동',lambda:self.find_nearest_node(move=True),GREEN).pack(side='left',expand=True,fill='x',padx=(2,0))
        if not self.real:
            pass
        r3=tk.Frame(b,bg=PANEL); r3.pack(fill='x',pady=(4,0))
        self.button(r3,'SIM 전원 OFF',self.sim_power_off,RED).pack(side='left',expand=True,fill='x',padx=(0,2))
        self.button(r3,'SIM 재부팅',self.sim_reboot,ORANGE).pack(side='left',expand=True,fill='x',padx=(2,0))
        self.button(b,'저장 위치 초기화',self.clear_last_pose).pack(fill='x',pady=(4,0))

        f,b = self.card(right,'수동 조작 · 누르는 동안 이동')
        f.pack(fill='x',pady=(0,8))
        tk.Checkbutton(b,text='수동 조작 활성화  (W/A/S/D)',variable=self.manual,bg=PANEL,
                       command=self.release_drive).pack(anchor='w')
        own = tk.Frame(b,bg=PANEL); own.pack(fill='x',pady=(3,5))
        self.button(own,'제어권 가져오기 (4005)',self.acquire_control,ORANGE).pack(side='left',expand=True,fill='x',padx=(0,2))
        self.button(own,'제어권 해제 (4006)',self.release_control).pack(side='left',expand=True,fill='x',padx=(2,0))
        grid = tk.Frame(b,bg=PANEL); grid.pack(pady=3)
        for title,cmd,r,c in [('▲','forward',0,1),('↶','left',1,0),('■','zero',1,1),
                              ('↷','right',1,2),('▼','back',2,1)]:
            button = self.button(grid,title,lambda:None,RED if cmd=='zero' else None,width=4,pady=3)
            button.grid(row=r,column=c,padx=3,pady=2)
            button.bind('<ButtonPress-1>',lambda e,n=cmd:self.press_drive(n))
        row = tk.Frame(b,bg=PANEL); row.pack(fill='x',pady=(4,0))
        self.label(row,'m/s',9,MUTED).pack(side='left')
        ttk.Entry(row,textvariable=self.speed,width=7).pack(side='left',padx=6)
        self.label(row,'deg/s',9,MUTED).pack(side='left')
        ttk.Entry(row,textvariable=self.angular,width=7).pack(side='left',padx=6)
        self._pad_build(b)
        f,b = self.card(right,'실기 명령 상태')
        f.pack(fill='x',pady=(0,8))
        self.command_status = self.label(b,'아직 전송된 명령 없음',9,MUTED,anchor='w',justify='left',wraplength=285)
        self.command_status.pack(fill='x')
        self.button(b,'상세 명령 로그 보기',lambda:self.tabs.select(self.logs_page)).pack(fill='x',pady=(6,0))

    def _map_info_dialog(self):
        messagebox.showinfo('지도 상태 / 조작 안내',self._map_info_details,parent=self)

    def _map_focus_toggle(self):
        self.map_focus=not self.map_focus
        if self.map_focus:
            self.operation_right_outer.grid_remove();self.operation_page.columnconfigure(1,minsize=0,weight=0)
            self.map_focus_button.configure(text='제어 패널 보기');self.map_focus_stop.pack(side='left',padx=3)
        else:
            self.operation_right_outer.grid();self.operation_page.columnconfigure(1,minsize=max(220,round(310*getattr(self,'ui_scale_ratio',1))),weight=1)
            self.map_focus_button.configure(text='지도 크게');self.map_focus_stop.pack_forget()
        self.after_idle(self.draw_map)

    def _nodes(self):
        top = tk.Frame(self.nodes_page,bg=BG); top.pack(fill='x',pady=8)
        files=tk.Frame(top,bg=BG);files.pack(fill='x',pady=(0,4))
        for title,fn in [('로컬 JSON',self.load_map),('로컬 SMAP',self.open_smap),('저장',self.save_map),
                         ('로봇 지도 가져오기',self.pull_robot_map),('Stations 조회',self.fetch_stations),('로봇 지도 적용',self.robot_load_map)]:
            self.button(files,title,fn,BLUE if fn==self.pull_robot_map else None).pack(side='left',padx=2)
        top1=tk.Frame(top,bg=BG); top1.pack(fill='x')
        top2=tk.Frame(top,bg=BG); top2.pack(fill='x',pady=(3,0))
        self.label(top1,'지도 · 노드 · 경로 편집',12,INK,True,bg=BG).pack(side='left',padx=(0,12))
        self.button(top1,'선택 속성',self.edit_editor_selection).pack(side='left',padx=8)
        self.button(top1,'삭제',self.delete_editor_selection).pack(side='left',padx=2)
        self.button(top2,'선택 두 노드 방향별 연결',self.connect_nodes).pack(side='left',padx=2)
        self.button(top2,'맵 검증',self.validate_map_editor).pack(side='left',padx=(10,2))
        self.button(top2,'AMR 업데이트',self.push_map_to_robot,GREEN).pack(side='left',padx=2)
        tk.Checkbutton(top2,text='업로드 후 현재맵 재적용',variable=self.auto_apply_uploaded_map,bg=BG,activebackground=BG,fg=INK,selectcolor=PANEL).pack(side='left',padx=6)
        body=tk.PanedWindow(self.nodes_page,orient='horizontal',bg=BG,sashwidth=6)
        body.pack(fill='both',expand=True)
        map_frame=tk.Frame(body,bg=PANEL)
        inspector=tk.Frame(body,bg=PANEL,width=390)
        body.add(map_frame,stretch='always',minsize=450)
        body.add(inspector,stretch='never',minsize=310,width=390)
        tools=tk.Frame(map_frame,bg=PANEL);tools.pack(fill='x',padx=6,pady=5)
        self.label(tools,'지도 편집',10,INK,True).pack(side='left',padx=(0,8))
        ttk.Combobox(tools,textvariable=self.edit_mode,values=['선택','Point 추가','Point 이동','경로 연결','곡선 편집','Advanced Area','벽 만들기','가상벽','다각형 영역','점군 지우개','SIM 장애물'],state='readonly',width=16).pack(side='left')
        for title,fn in [('맞춤',self.fit_map),('+',lambda:self.change_zoom(1.2)),('−',lambda:self.change_zoom(1/1.2))]:
            self.button(tools,title,fn).pack(side='left',padx=2)
        layers=tk.Frame(map_frame,bg=PANEL);layers.pack(fill='x',padx=6,pady=(0,4))
        for name,var in self.layers.items():
            tk.Checkbutton(layers,text=name,variable=var,bg=PANEL,activebackground=PANEL,command=self.draw_map).pack(side='left')
        self.editor_canvas=tk.Canvas(map_frame,bg=DARK,highlightthickness=0)
        self.editor_canvas.pack(fill='both',expand=True,padx=6)
        self.editor_canvas.bind('<Configure>',lambda e:self.draw_map() if self.canvas is self.editor_canvas else None)
        for event,callback in [('<Button-1>',self.map_click),('<B1-Motion>',self.map_drag),('<ButtonRelease-1>',self.map_release),
                               ('<Double-Button-1>',self.map_double_click),('<ButtonPress-3>',self.pan_start),('<B3-Motion>',self.pan_move)]:
            self.editor_canvas.bind(event,callback)
        self.editor_canvas.bind('<MouseWheel>',lambda e:self.change_zoom(1.12 if e.delta>0 else 1/1.12))
        self.editor_canvas.bind('<Button-4>',lambda e:self.change_zoom(1.12))
        self.editor_canvas.bind('<Button-5>',lambda e:self.change_zoom(1/1.12))
        self.label(map_frame,'지도 클릭 ↔ 목록 선택 · 곡선 편집: 경로 클릭 후 C1/C2 드래그 · 우클릭 이동',8,MUTED,anchor='w').pack(fill='x',padx=8,pady=5)
        mapping=tk.Frame(map_frame,bg=PANEL);mapping.pack(fill='x',padx=6,pady=(0,5))
        self.button(mapping,'맵 생성 시작',lambda:self.action('mapping_start'),BLUE).pack(side='left')
        self.button(mapping,'맵 생성 종료',lambda:self.action('mapping_stop')).pack(side='left',padx=4)
        self.button(mapping,'스캔 CSV 저장',self.save_scan).pack(side='left')
        self.mapping_status=self.label(mapping,'SLAM: DEMO',8,MUTED)
        self.mapping_status.pack(side='right')
        self.editor_lists=ttk.Notebook(inspector)
        self.editor_selection_kind='node'
        self.editor_lists.pack(fill='both',expand=True,padx=5,pady=5)
        left=tk.Frame(self.editor_lists,bg=PANEL)
        paths=tk.Frame(self.editor_lists,bg=PANEL)
        areas=tk.Frame(self.editor_lists,bg=PANEL)
        for frame,title in [(left,'노드'),(paths,'경로'),(areas,'영역')]:self.editor_lists.add(frame,text=title)
        right_outer,right,_nodes_right_canvas=self._scrollable_frame(inspector,bg=PANEL)
        right_outer.pack(fill='both',expand=True,padx=5,pady=(0,5))
        _nodes_right_canvas.configure(height=220)
        self.nodes_right_canvas=_nodes_right_canvas
        self.node_tree = ttk.Treeview(left,columns=('id','x','y','angle','spin','kind','opts'),displaycolumns=('id','x','y','angle'),height=7,show='headings',selectmode='extended')
        for col,title,width in [('id','Node ID',130),('x','X (m)',100),('y','Y (m)',100),('angle','Angle',90),
                                ('spin','Spin',60),('kind','Type',130),('opts','Options',280)]:
            self.node_tree.heading(col,text=title); self.node_tree.column(col,width=min(width,85),minwidth=55,stretch=True)
        self.node_tree.pack(side='left',fill='both',expand=True)
        ttk.Scrollbar(left,orient='vertical',command=self.node_tree.yview).pack(side='right',fill='y')
        self.node_tree.configure(yscrollcommand=left.winfo_children()[-1].set)
        self.node_tree.bind('<<TreeviewSelect>>',self.node_tree_select)
        self.node_tree.bind('<Double-Button-1>',lambda e:self.edit_node())
        self.path_tree = ttk.Treeview(paths,columns=('path','traffic','drive','style','speed'),displaycolumns=('traffic','drive','speed'),show='headings',height=7,selectmode='extended')
        for col,title,width in [('path','Path',190),('traffic','통행 방향',210),('drive','주행',100),('style','MoveStyle',100),('speed','MaxSpeed',100)]:
            self.path_tree.heading(col,text=title); self.path_tree.column(col,width=170 if col=='traffic' else 80,minwidth=55,stretch=True)
        self.path_tree.pack(fill='both',expand=True)
        self.path_tree.bind('<Double-Button-1>',lambda e:self.edit_selected_path())
        self.path_tree.bind('<<TreeviewSelect>>',lambda e:self._path_tree_select())
        pr=tk.Frame(paths,bg=BG);pr.pack(fill='x',pady=(4,2))
        self.button(pr,'속성',self.edit_selected_path,BLUE).pack(side='left',padx=(0,4))
        self.button(pr,'방향별 삭제',self.delete_selected_path).pack(side='left',padx=2)
        self.button(pr,'역방향 추가',self.add_reverse_selected_path).pack(side='left',padx=2)
        self.area_tree = ttk.Treeview(areas,columns=('id','speed','decel','stop','expand','forbidden'),displaycolumns=('id','speed','forbidden'),show='headings',height=5,selectmode='browse')
        for col,title,width in [('id','Area',120),('speed','MaxSpeed',90),('decel','감속거리',90),('stop','정지거리',90),('expand','확장폭',85),('forbidden','금지',60)]:
            self.area_tree.heading(col,text=title); self.area_tree.column(col,width=width,stretch=(col=='id'))
        self.area_tree.pack(fill='both',expand=True)
        self.area_tree.bind('<<TreeviewSelect>>',lambda e:self._area_tree_select())
        self.area_tree.bind('<Double-Button-1>',lambda e:self.edit_selected_area())
        ar=tk.Frame(areas,bg=BG); ar.pack(fill='x',pady=(4,2))
        self.button(ar,'선택 Area 속성',self.edit_selected_area,ORANGE).pack(side='left',padx=(0,4))
        self.button(ar,'Area 삭제',self.delete_selected_area).pack(side='left',padx=2)
        self.edge_text = self.label(paths,'',8,MUTED,bg=BG,anchor='w',wraplength=350)
        self.edge_text.pack(fill='x',pady=6)
        self.label(right,'선택 항목 속성',11,INK,True).pack(anchor='w',padx=12,pady=(12,6))
        self.node_prop_text=self.label(right,'맵에서 Point를 선택하세요.',9,'#52677f',anchor='nw',justify='left',wraplength=285)
        self.node_prop_text.pack(fill='x',padx=12,pady=(0,8))
        self.button(right,'선택 항목 속성 편집',self.edit_editor_selection,BLUE).pack(fill='x',padx=12,pady=3)
        self.button(right,'Point 전체 옵션 보기',self.show_selected_station_options).pack(fill='x',padx=12,pady=3)
        self.label(right,'Point 추가 방법',10,INK,True).pack(anchor='w',padx=12,pady=(18,4))
        self.label(right,'Point: 맵 클릭으로 생성 → 방향 핸들 드래그\nPath: 두 Point 선택 → 방향별 A→B / B→A 생성·전진/후진 독립 설정\nAdvanced Area: 맵에서 드래그해 영역 생성 → 속도/장애물 거리 설정',
                   9,MUTED,anchor='nw',justify='left',wraplength=285).pack(fill='x',padx=12)
        self.label(self.nodes_page,'Standalone Map Editor: Point/Bezier 편집 → 맵 검증 → 4010 Upload → 4011 검증 → 선택 시 2022 재적용. Push 전 로컬 백업을 자동 생성합니다.',
                   color=MUTED,bg=BG).pack(anchor='w',pady=8)

    def edit_editor_selection(self):
        kind=getattr(self,'editor_selection_kind','node')
        if kind=='path':self.edit_selected_path()
        elif kind=='area':self.edit_selected_area()
        else:self.edit_node()

    def delete_editor_selection(self):
        kind=getattr(self,'editor_selection_kind','node')
        if kind=='path':self.delete_selected_path()
        elif kind=='area':self.delete_selected_area()
        else:self.delete_node()

    def _tasks(self):
        bar=tk.Frame(self.tasks_page,bg=BG); bar.pack(fill='x',pady=10)
        for title,fn,color in [('New',self.tc_new,BLUE),('순환 미션 만들기',self.quick_loop_mission,ORANGE),('Load',self.load_tasks,None),('Save',self.save_tasks,None),('Task List',self.tc_task_list,None),('미션 시작',self.run_tasks,GREEN),('미션 정지',self.cancel_tasks,RED)]:
            self.button(bar,title,fn,color).pack(side='left',padx=3)
        self.tc_loop=tk.BooleanVar(value=False)
        tk.Checkbutton(bar,text='무한 반복',variable=self.tc_loop,bg=BG,fg=INK,selectcolor=PANEL,activebackground=BG,activeforeground=INK).pack(side='left',padx=(12,4))
        self.label(bar,'반복 횟수',9,MUTED,bg=BG).pack(side='left',padx=(4,3))
        self.tc_repeat_count=tk.StringVar(value='1')
        tk.Spinbox(bar,from_=1,to=999,textvariable=self.tc_repeat_count,width=5,bg=PANEL,fg=INK,insertbackground=INK,buttonbackground='#46515e').pack(side='left')
        self.mission_status=self.label(bar,'미션 대기',9,MUTED,bg=BG)
        self.mission_status.pack(side='right',padx=8)
        pane=tk.PanedWindow(self.tasks_page,orient='horizontal',bg=BG,sashwidth=5); pane.pack(fill='both',expand=True)
        cols=[]
        for title in ['Taskchain','Task','Group','Draggable Tab','Properties / Operation']:
            f=tk.Frame(pane,bg=PANEL,highlightbackground='#ccd5e0',highlightthickness=1); pane.add(f,stretch='always'); cols.append(f)
            self.label(f,title,10,INK,True).pack(anchor='w',padx=9,pady=(8,5))
        self.tc_chain_list=tk.Listbox(cols[0],bg='#ffffff',fg=INK,selectbackground='#2376cf',relief='flat',exportselection=False)
        self.tc_chain_list.pack(fill='both',expand=True,padx=8,pady=5); self.tc_chain_list.bind('<<ListboxSelect>>',lambda e:self.tc_select_chain())
        r=tk.Frame(cols[0],bg=PANEL); r.pack(fill='x',padx=8,pady=6)
        self.button(r,'New',self.tc_new).pack(side='left',expand=True,fill='x',padx=1); self.button(r,'Delete',self.tc_delete_chain).pack(side='left',expand=True,fill='x',padx=1)
        self.tc_task_listbox=tk.Listbox(cols[1],bg='#ffffff',fg=INK,selectbackground='#2376cf',relief='flat',exportselection=False)
        self.tc_task_listbox.pack(fill='both',expand=True,padx=8,pady=5); self.tc_task_listbox.bind('<<ListboxSelect>>',lambda e:self.tc_select_task())
        r=tk.Frame(cols[1],bg=PANEL); r.pack(fill='x',padx=8,pady=6)
        self.button(r,'New Task',self.tc_new_task).pack(side='left',expand=True,fill='x',padx=1); self.button(r,'Delete Task',self.tc_delete_task).pack(side='left',expand=True,fill='x',padx=1)
        self.tc_group_list=tk.Listbox(cols[2],bg='#ffffff',fg=INK,selectbackground='#2376cf',relief='flat',exportselection=False)
        self.tc_group_list.pack(fill='both',expand=True,padx=8,pady=5); self.tc_group_list.bind('<<ListboxSelect>>',lambda e:self.tc_select_group())
        r=tk.Frame(cols[2],bg=PANEL); r.pack(fill='x',padx=8,pady=6)
        self.button(r,'Add Group',self.tc_add_group).pack(side='left',expand=True,fill='x',padx=1); self.button(r,'Delete',self.tc_delete_group).pack(side='left',expand=True,fill='x',padx=1)
        actions=list(ACTION_DEFAULTS)
        self.tc_action_list=tk.Listbox(cols[3],bg='#ffffff',fg=INK,selectbackground='#2376cf',relief='flat',exportselection=False)
        for a in actions:self.tc_action_list.insert('end',a)
        self.tc_action_list.pack(fill='both',expand=True,padx=8,pady=5)
        self.tc_action_list.bind('<Double-Button-1>',lambda e:self.tc_add_action())
        self.label(cols[3],'더블클릭 → 선택 Group에 추가\n(RoboShop의 Drag & Drop 동작을 단순화)',8,MUTED,anchor='w',justify='left').pack(fill='x',padx=8,pady=5)
        self.button(cols[3],'선택 Action 추가',self.tc_add_action,BLUE).pack(fill='x',padx=8,pady=6)
        self.tc_group_actions=tk.Listbox(cols[4],bg='#ffffff',fg=INK,selectbackground='#2376cf',relief='flat',exportselection=False,height=9)
        self.tc_group_actions.pack(fill='x',padx=8,pady=5); self.tc_group_actions.bind('<<ListboxSelect>>',lambda e:self.tc_action_selected())
        self.tc_prop=tk.Text(cols[4],height=13,bg='#ffffff',fg='#263445',insertbackground=INK,relief='flat',wrap='word')
        self.tc_prop.pack(fill='both',expand=True,padx=8,pady=5)
        self.button(cols[4],'선택 Action 속성 편집',self.tc_edit_action).pack(fill='x',padx=8,pady=3)
        self.button(cols[4],'선택 Action 삭제',self.tc_delete_action).pack(fill='x',padx=8,pady=3)
        self.label(self.tasks_page,'빠른 순환 미션: [순환 미션 만들기] → Point를 순서대로 추가 → 시작 Point를 마지막에 다시 넣기 → 반복 횟수/무한 반복 선택 → [미션 시작]. 일반 Taskchain 편집도 그대로 사용할 수 있습니다.',color=MUTED,bg=BG).pack(anchor='w',pady=8)
        self.tc_new()

    def _telemetry(self):
        self.telemetry = tk.Text(self.telemetry_page,bg='#ffffff',fg='#263445',insertbackground=PANEL,
                                 font=('Consolas',11),relief='flat',padx=20,pady=16,state='disabled')
        self.telemetry.pack(fill='both',expand=True,pady=10)
        self.button(self.telemetry_page,'상태 JSON 저장',self.save_snapshot).pack(anchor='e',pady=(0,10))

    def _robot_info(self):
        top=tk.Frame(self.robot_info_page,bg=BG); top.pack(fill='x',pady=(10,4))
        self.label(top,'SEER 로봇 실시간 운영 정보 · API 1000/1002/1007/1100/1101',12,INK,True,bg=BG).pack(side='left')
        self.robot_info_status=self.label(top,'연결 대기',9,MUTED,bg=BG); self.robot_info_status.pack(side='right')
        self.robot_info_text=tk.Text(self.robot_info_page,bg=WHITE,fg=INK,insertbackground=INK,font=(self.font,10),relief='flat',padx=16,pady=14,state='disabled',wrap='none')
        self.robot_info_text.pack(fill='both',expand=True,pady=(4,10))

    @staticmethod
    def _pick(d,*names,default='—'):
        if not isinstance(d,dict): return default
        for name in names:
            if name in d and d[name] not in (None,''): return d[name]
        return default

    def _robot_info_string(self):
        raw=self.raw if isinstance(self.raw,dict) else {}
        all0=raw.get('all',{}) if isinstance(raw.get('all'),dict) else {}
        api1000=raw.get('api_1000',{}) if isinstance(raw.get('api_1000'),dict) else {}
        api1002=raw.get('api_1002',{}) if isinstance(raw.get('api_1002'),dict) else {}
        api1007=raw.get('api_1007',{}) if isinstance(raw.get('api_1007'),dict) else {}
        api1020=raw.get('api_1020',{}) if isinstance(raw.get('api_1020'),dict) else {}
        api1021=raw.get('api_1021',{}) if isinstance(raw.get('api_1021'),dict) else {}
        api1022=raw.get('api_1022',{}) if isinstance(raw.get('api_1022'),dict) else {}
        all2=raw.get('all2_1101',{}) if isinstance(raw.get('all2_1101'),dict) else {}
        s=self.live if isinstance(self.live,dict) else {}

        # Search the API that actually owns the field first, then batch responses.
        sources=[api1000,api1002,api1007,api1020,api1021,api1022,all2,all0]
        def P(*names,default='—',prefer=None):
            seq=(prefer or [])+sources
            seen=set()
            for d in seq:
                if id(d) in seen: continue
                seen.add(id(d))
                if not isinstance(d,dict): continue
                for name in names:
                    if name in d and d[name] not in (None,''):
                        return d[name]
            return default
        def F(v,digits=3):
            return f'{v:.{digits}f}' if isinstance(v,(int,float)) and not isinstance(v,bool) else str(v)
        def pct(v):
            if isinstance(v,(int,float)) and not isinstance(v,bool):
                return f'{(v*100.0 if 0<=v<=1.0001 else v):.1f} %'
            return str(v)
        def yesno(v):
            if v is True:return '예'
            if v is False:return '아니오'
            return str(v)
        def mode(v):
            return {0:'수동',1:'자동','MANUAL':'수동','AUTO':'자동'}.get(v,v)
        def task_status(v):
            return {0:'없음',1:'대기',2:'실행 중',3:'일시정지',4:'완료',5:'실패',6:'취소'}.get(v,v)

        x=self._pick(s,'x',default=P('x')); y=self._pick(s,'y',default=P('y')); a=self._pick(s,'theta',default=P('angle','theta'))
        vx=P('vx'); vy=P('vy'); w=P('w','vw','omega')
        batt_level=P('battery_level','battery',prefer=[api1007])
        batt_voltage=P('voltage','battery_voltage',prefer=[api1007])
        batt_current=P('current','battery_current',prefer=[api1007])
        batt_temp=P('battery_temp','battery_temperature',prefer=[api1007])
        charging=P('charging','is_charging',prefer=[api1007])
        groups=[
          ('로봇 실행 상태',[
            ('실행 모드',mode(P('mode','robot_mode','work_mode'))),
            ('누적 마일리지',P('total_mileage','mileage','odometer','total_odo')),
            ('오늘 누적 마일리지',P('today_mileage','daily_mileage','today_odo')),
            ('누적 실행 시간',P('total_run_time','total_runtime','running_time','total_time')),
            ('실행 시간',P('run_time','runtime','current_run_time')),
            ('컨트롤러 전압',P('controller_voltage','control_voltage','electric')),
            ('컨트롤러 온도',P('controller_temperature','controller_temp','temperature')),
            ('컨트롤러 습도',P('controller_humidity','controller_hum','humidity')),
            ('배터리 레벨',pct(batt_level)),('배터리 온도',batt_temp),
            ('배터리 전압',batt_voltage),('배터리 전류',batt_current),
            ('배터리 주기',P('battery_cycle','battery_cycles','cycle_count')),
            ('충전 중',yesno(charging)),
            ('최대 충전 전압',P('max_charge_voltage','charge_voltage_max','max_charging_voltage')),
            ('최대 충전 전류',P('max_charge_current','charge_current_max','max_charging_current'))]),
          ('로봇 모션 상태',[
            ('로봇 위치',F(x)+' m, '+F(y)+' m, '+F(a)+' rad'),
            ('실제 속도',F(vx)+' / '+F(vy)+' / '+F(w)),
            ('자신감',P('confidence','localization_confidence')),('블록',P('blocked')),
            ('가장 가까운 장애물 X',P('block_x')),('가장 가까운 장애물 Y',P('block_y')),
            ('블록 센서 ID',P('block_di','block_sensor_id')),('브레이크',P('brake','brake_state')),
            ('EMC',P('emergency','emergency_stop','emc','driver_emc')),
            ('IMU',P('imu','imu_data','imu_header')),('SRC 모드',P('src_mode')),
            ('새시 정지 상태',P('chassis_stop','chassis_stopped'))]),
          ('로봇 내비게이션 상태',[
            ('탐색 유형',P('nav_type','navigation_type','task_type')),
            ('항해 상태',task_status(P('nav_status','navigation_status','task_status',prefer=[api1020]))),
            ('현재 지점',P('current_station','current_point','current_id')),
            ('현재 대상',P('target_id','target',prefer=[api1020])),
            ('현재 대상 좌표',P('target_point','target_pose','target_x',prefer=[api1020])),
            ('현재 대상 dist',P('target_dist','target_distance')),
            ('과거 워크스테이션',P('last_station','previous_station')),
            ('위치 상태',P('reloc_status','status','state',prefer=[api1021])),
            ('지도 로딩 상태',P('loadmap_status','status','state',prefer=[api1022])),
            ('슬램 상태',P('slam_status')),('로봇 영역',P('area_ids','area_id')),
            ('현재 사용되는 지도',P('current_map','map_name'))]),
          ('로봇 기본 정보',[
            ('로봇 ID',P('robot_id','id','serial_number','robotId',prefer=[api1000])),
            ('로봇 이름',P('robot_name','name','robotName',prefer=[api1000])),
            ('로봇 노트',P('robot_note','note','remark',prefer=[api1000])),
            ('로봇 모델',P('robot_model','model','robot_type',prefer=[api1000])),
            ('로보킷 버전',P('robokit_version','robot_version','software_version','version',prefer=[api1000])),
            ('펌웨어 버전',P('firmware_version','fw_version','firmware',prefer=[api1000])),
            ('차이로 버전',P('chassis_version','chassis_firmware_version')),
            ('TCP API 버전',P('tcp_api_version','api_version','protocol_version',prefer=[api1000])),
            ('모드버스 버전',P('modbus_version')),('지도 버전',P('map_version')),('모델 버전',P('model_version')),
            ('MAC',P('mac','mac_address','ethernet_mac',prefer=[api1000])),
            ('유선 네트워크 카드 모델',P('ethernet_model','wired_nic_model')),
            ('Wi-Fi MAC',P('wifi_mac','wifi_mac_address','wlan_mac',prefer=[api1000])),
            ('무선 네트워크 카드 모델',P('wifi_model','wireless_nic_model'))])
        ]
        out=[]
        for title,items in groups:
            out += ['━'*15+'  '+title+'  '+'━'*15] + [f'{label:<18} : {value}' for label,value in items] + ['']
        received=[str(api) for api in (1000,1002,1007,1020,1021,1022) if isinstance(raw.get(f'api_{api}'),dict)]
        out.append('[API] 1100/1101 + 개별 조회 수신: '+(', '.join(received) if received else '대기 중'))
        out.append('※ — 표시는 해당 펌웨어/API 응답에 필드가 없거나 아직 수신되지 않은 항목입니다.')
        return '\n'.join(out)

    def _settings(self):
        f,b = self.card(self.settings_page,'노트북 → 로봇 연결')
        f.pack(fill='x',pady=12)
        text = ('1. 로봇 AP 또는 같은 스위치에 노트북을 연결합니다.\n'
                '2. 로봇 IP/서브넷을 확인하고, 노트북에 중복되지 않는 같은 대역 IP를 설정합니다.\n'
                '   매뉴얼 유선 IP 예시: 192.168.192.5. 실제 로봇 IP를 우선 확인하세요.\n'
                '3. 아래에 문서로 확인한 TCP 포트를 입력하고 연결 진단을 실행합니다.\n'
                '4. 실기 · 조회 전용으로 먼저 연결합니다. 이동하려면 실기 · 제어로 다시 연결합니다.')
        self.label(b,text,10,INK,justify='left',anchor='w').pack(fill='x')
        row = tk.Frame(b,bg=PANEL); row.pack(fill='x',pady=12)
        self.label(row,'진단 포트 (쉼표 구분)').pack(side='left')
        self.ports = tk.StringVar(value='19204,19205,19206,19207')
        ttk.Entry(row,textvariable=self.ports,width=28).pack(side='left',padx=10)
        self.button(row,'TCP 연결 진단',self.diagnose,BLUE).pack(side='left')
        self.diagnostic = self.label(b,'대기 · 포트가 열려 있어도 API 호환성을 보장하지 않습니다.',color=MUTED,justify='left')
        self.diagnostic.pack(anchor='w')
        f,b = self.card(self.settings_page,'마지막 위치 자동 저장 / 복구')
        f.pack(fill='x',pady=(0,12))
        tk.Checkbutton(b,text='마지막 위치 자동 저장',variable=self.pose_autosave,bg=PANEL,fg=INK,selectcolor=PANEL,activebackground=PANEL,activeforeground=INK).pack(anchor='w')
        tk.Checkbutton(b,text='연결/재부팅 시 마지막 위치 자동 복구 제안',variable=self.pose_autorecover,bg=PANEL,fg=INK,selectcolor=PANEL,activebackground=PANEL,activeforeground=INK).pack(anchor='w')
        tk.Checkbutton(b,text='Confidence 기준 충족 시 자동 위치 확정(2003)',variable=self.pose_autoconfirm,bg=PANEL,fg=INK,selectcolor=PANEL,activebackground=PANEL,activeforeground=INK).pack(anchor='w')
        tk.Checkbutton(b,text='위치복구 후 순환미션의 가장 가까운 노드에서 재개',variable=self.pose_resume_loop,bg=PANEL,fg=INK,selectcolor=PANEL,activebackground=PANEL,activeforeground=INK).pack(anchor='w')
        rr=tk.Frame(b,bg=PANEL); rr.pack(fill='x',pady=(6,0))
        self.label(rr,'Confidence 기준').pack(side='left')
        ttk.Entry(rr,textvariable=self.pose_conf_threshold,width=8).pack(side='left',padx=6)
        self.label(b,'※ 저장 pose는 초기 추정값입니다. 전원 OFF 중 로봇이 움직였을 수 있으므로 Map 일치와 Localization Confidence를 확인한 뒤 확정합니다.',8,MUTED,justify='left',anchor='w',wraplength=900).pack(fill='x',pady=(5,0))

        f,b = self.card(self.settings_page,'API 프로파일 / 지원 범위')
        f.pack(fill='x',pady=(0,12))
        self.profile_label = self.label(b,str(self.profile),9,MUTED,anchor='w')
        self.profile_label.pack(fill='x')
        self.button(b,'프로파일 JSON 선택',self.choose_profile).pack(anchor='w',pady=10)
        self.label(b, '공식 SEER TCP API 1.2.1 / Python·Java SDK 기반 (실기 미검증)\n'
                     '실기 조회: 위치, LiDAR Hit Point, 배터리, Task, 경로, 알람\n'
                     '실기 제어: 목적지 이동, Pause/Resume/Cancel, 재위치·확인\n'
                     'SMAP: 로컬/로봇 4011 JSON 지도의 점군·노드·Bezier 경로 읽기\n'
                     '맵 다운로드: 1300으로 현재 지도 확인 → 4011로 JSON 수신/표시/캐시\n'
                     '수동 조그: 2010을 누르는 동안 주기 전송, 해제 시 2000 정지\n'
                     '명령 TX/RX/오류와 현재 mode/task/safety를 이벤트 로그 및 우측 상태창에 표시합니다.',
                   justify='left',color=INK).pack(anchor='w')
        row=tk.Frame(b,bg=PANEL);row.pack(fill='x',pady=8)
        for title,fn in [('로봇 노드 조회',self.fetch_stations),('SMAP 열기',self.open_smap),
                         ('Confirm Loc',lambda:self.send_command('confirm_loc')),
                         ('자동 모드',lambda:self.send_command('mode',{'mode':1})),
                         ('수동 모드',lambda:self.send_command('mode',{'mode':0}))]:
            self.button(row,title,fn).pack(side='left',padx=2)
        self.button(b,'설정 저장',self.save_settings).pack(anchor='w',pady=(12,0))
        try:
            cfg = json.loads((USER_DIR/'settings.json').read_text(encoding='utf-8'))
            self.host.set(str(cfg.get('host',self.host.get())))
            self.ports.set(str(cfg.get('ports','')))
            p = Path(cfg.get('profile',str(self.profile)))
            if p.is_file():
                self.profile = p
                self.profile_label.config(text=str(p))
            self.pose_autosave.set(bool(cfg.get('pose_autosave',True)))
            self.pose_autorecover.set(bool(cfg.get('pose_autorecover',False)))
            self.pose_autoconfirm.set(bool(cfg.get('pose_autoconfirm',False)))
            self.pose_resume_loop.set(bool(cfg.get('pose_resume_loop',False)))
            self.pose_conf_threshold.set(str(cfg.get('pose_conf_threshold','0.80')))
        except (OSError,ValueError,TypeError):
            pass

    def _logs(self):
        bar = tk.Frame(self.logs_page,bg=BG); bar.pack(fill='x',pady=12)
        self.button(bar,'CSV 내보내기',self.export_logs,BLUE).pack(side='right')
        self.log_tree = ttk.Treeview(self.logs_page,columns=('time','level','message'),show='headings')
        for key,title,width in [('time','시간',165),('level','레벨',85),('message','내용',800)]:
            self.log_tree.heading(key,text=title); self.log_tree.column(key,width=width)
        scroll = ttk.Scrollbar(self.logs_page,command=self.log_tree.yview)
        self.log_tree.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right',fill='y')
        self.log_tree.pack(fill='both',expand=True)

    def _menu(self):
        menu = tk.Menu(self)
        home = tk.Menu(menu, tearoff=False)
        home.add_command(label='Map/Control',command=lambda:self.tabs.select(self.operation_page))
        home.add_command(label='연결 설정',command=lambda:self.tabs.select(self.settings_page))
        menu.add_cascade(label='Home',menu=home)
        connection = tk.Menu(menu,tearoff=False)
        connection.add_command(label='Connect',command=self.connect)
        connection.add_command(label='Disconnect',command=self.disconnect)
        menu.add_cascade(label='Connection',menu=connection)
        setting = tk.Menu(menu,tearoff=False)
        setting.add_command(label='API Profile',command=self.choose_profile)
        setting.add_command(label='Save Settings',command=self.save_settings)
        menu.add_cascade(label='Setting',menu=setting)
        other = tk.Menu(menu,tearoff=False)
        other.add_command(label='Export Logs',command=self.export_logs)
        other.add_command(label='Export Snapshot',command=self.save_snapshot)
        menu.add_cascade(label='Other',menu=other)
        help_menu = tk.Menu(menu,tearoff=False)
        help_menu.add_command(label='지원 범위',command=lambda:self.tabs.select(self.settings_page))
        help_menu.add_command(label='About',command=lambda:messagebox.showinfo('About',
            'AMR Control Studio 0.5.0\n독립 개발 Python GUI / SEER 공식 제품이 아닙니다.\n'
            '참고: RoboShop 매뉴얼 p.93–98, 143–156, Taskchain.\n'
            'API 원문 미확인: 실기 제어/SLAM은 구현되지 않았습니다.'))
        menu.add_cascade(label='Help',menu=help_menu)
        self.config(menu=menu)

    def unavailable(self,name):
        messagebox.showinfo(name, name+' 기능은 이번 버전에서 지원하지 않습니다.\n'
                            '로컬 맵 파일 저장/열기는 사용할 수 있습니다.')

    def relocate(self):
        if self.real:
            x=simpledialog.askfloat('Relocate','X (m)',parent=self)
            if x is None:return
            y=simpledialog.askfloat('Relocate','Y (m)',parent=self)
            if y is None:return
            angle=simpledialog.askfloat('Relocate','Heading (deg)',parent=self)
            if angle is None:return
            self.send_command('relocate',{'x':x,'y':y,'angle':math.radians(angle)});return
        def run():
            self.sim_required()
            self.editable()
            x=simpledialog.askfloat('Relocate (SIM)','X (m)',initialvalue=self.sim.state.x,parent=self)
            if x is None:return
            y=simpledialog.askfloat('Relocate (SIM)','Y (m)',initialvalue=self.sim.state.y,parent=self)
            if y is None:return
            angle=simpledialog.askfloat('Relocate (SIM)','Heading (deg)',
                        initialvalue=math.degrees(self.sim.state.theta),parent=self)
            if angle is None:return
            x,y,angle=number(x),number(y),number(angle)
            if not messagebox.askyesno('Confirm Loc (SIM)',f'X={x:.3f}, Y={y:.3f}, θ={angle:.1f}° 로 재배치할까요?'):return
            self.sim.state.x,self.sim.state.y,self.sim.state.theta=x,y,math.radians(angle)
            self.sim.state.last_node=self.map.nearest(x,y)
            self.log('COMMAND','[SIM] Relocate / Confirm Loc')
        self.guarded(run)

    def key_press(self,event):
        if self.focus_get() not in (self.operation_canvas,self.world3d) or not self.manual.get():return
        direction={'w':'forward','s':'back','a':'left','d':'right'}.get(event.keysym.lower())
        if direction:self.press_drive(direction)

    def key_release(self,event):
        if event.keysym.lower() in ('w','a','s','d'):self.release_drive()

    def log(self,level,message):
        row = [datetime.now().isoformat(timespec='seconds'),level,str(message)]
        self.logs.append(row)
        self.logs = self.logs[-3000:]
        item = self.log_tree.insert('', 'end', values=row)
        children = self.log_tree.get_children()
        if len(children)>1000:
            self.log_tree.delete(children[0])
        self.log_tree.see(item)

    def sim_required(self):
        if not self.connected:
            raise ValueError('먼저 연결하세요.')
        if self.real:
            raise ValueError('이 기능은 시뮬레이션 전용입니다.')

    def guarded(self,fn):
        try:
            return fn()
        except (ValueError,KeyError,OSError,TypeError) as e:
            self.log('ERROR',e)
            messagebox.showerror('작업 실패',str(e),parent=self)

    def connect(self):
        host = self.host.get().strip()
        real = self.mode.get().startswith('실기')
        if real:
            try:
                if not host:
                    raise ValueError('로봇 IP를 입력하세요.')
                client = SeerClient(host,json.loads(self.profile.read_text(encoding='utf-8-sig')))
            except (OSError,ValueError,KeyError,TypeError) as e:
                self.log('ERROR',str(e)); messagebox.showerror('실기 연결 불가',str(e)); return
        self.disconnect()
        self.real = real
        self.control_enabled = real and self.mode.get()=='실기 · 제어'
        self.robot_stations = {}
        self.loading_map = False
        self.downloading_map = False
        self.current_robot_map = ''
        # SEER 4005/4006 require the same non-empty nick_name in the request body.
        self.control_nick = 'SEER_AMR_Control'
        self.commands = queue.Queue(maxsize=4)
        commands = self.commands
        self.generation += 1
        generation = self.generation
        if not real:
            if self.map is not self.sim.map:
                self.map=MapModel.load(ROOT/'maps/demo.json');self.sim=Simulator(self.map);self.refresh_nodes()
            if not self.map.nodes:
                self.map=MapModel.load(ROOT/'maps/demo.json');self.sim=Simulator(self.map);self.refresh_nodes()
            self.connected = True
            self.live = {}
            self.connection_text.config(text='DEMO 연결됨 · TCP 전송 없음')
            self.log('INFO','시뮬레이션 연결')
            return
        if not hasattr(self.map,'smap_source'):
            self.map=MapModel(dict(format='amr-console-map-v1',name='Robot coordinates',nodes=[dict(id='_origin',x=0,y=0)],edges=[],walls=[]))
            self.map.nodes={}
            self.refresh_nodes()
        self.connection_text.config(text='TCP 응답 대기 중…')
        self.worker_stop = threading.Event()
        stop = self.worker_stop
        # Keep status polling and control commands on independent TCP clients/threads.
        # A slow command response (notably 3051) must never starve the live-state heartbeat.
        state_client = client
        command_client = SeerClient(host,json.loads(self.profile.read_text(encoding='utf-8-sig')))
        command_client.timeout = 8.0
        # Jog uses a persistent connection, matching SEER's official Python control example.
        # Reconnecting for every 2010 packet can be unreliable on some controller/firmware builds.
        jog_client = PersistentSeerSession(host,19205,timeout=3.0)
        self._jog_wakeup = threading.Event()
        jog_wakeup = self._jog_wakeup

        def state_work():
            while not stop.is_set():
                try:
                    state,raw = state_client.snapshot()
                    self.events.put(('state',generation,(state,raw,time.monotonic())))
                except Exception as e:
                    self.events.put(('error',generation,'상태 조회 실패: '+str(e)))
                    break
                stop.wait(.2)

        def command_work():
            while not stop.is_set():
                try:
                    try:
                        command,payload = commands.get(timeout=.1)
                    except queue.Empty:
                        continue
                    try:
                        started=time.monotonic()
                        if command in ('stations','maps','maps_for_pull'):
                            api = 1301 if command=='stations' else 1300
                            port = 19204
                        elif command=='download_map':
                            api,port = 4011,19207
                        else:
                            port,api = COMMANDS.get(command,(None,None))
                        self.events.put(('command_io',generation,('TX',command,port,api,payload,None,0.0)))
                        if command in ('stations','maps','maps_for_pull'):
                            result=command_client.query(api)
                        elif command=='download_map':
                            result=command_client.download_map(payload['map_name'])
                        else:
                            result=command_client.command(command,payload)
                        elapsed=(time.monotonic()-started)*1000.0
                        self.events.put(('command_io',generation,('RX',command,port,api,payload,result,elapsed)))

                        if command=='navigate':
                            try:
                                task_probe=command_client.query(1020)
                                self.events.put(('nav_probe',generation,task_probe))
                            except Exception as probe_error:
                                self.events.put(('command_io',generation,('PROBE_ERR','task_status',19204,1020,None,str(probe_error),0.0)))

                        if command=='load_map':
                            # Confirm load using the command client; live status continues in state_work.
                            deadline=time.monotonic()+30
                            while not stop.is_set() and time.monotonic()<deadline:
                                maps=command_client.query(1300)
                                if maps.get('current_map')==payload['map_name']:
                                    stations=command_client.query(1301)
                                    self.events.put(('map_loaded',generation,payload['map_name']))
                                    self.events.put(('stations',generation,('stations',stations)))
                                    break
                                stop.wait(.3)
                            else:
                                if not stop.is_set():
                                    raise ValueError('지도 로드 확인 시간 초과. 로봇의 실제 지도 상태를 확인하세요.')

                        event_kind=command if command in ('stations','maps','maps_for_pull','download_map') else 'command'
                        self.events.put((event_kind,generation,(command,result,payload)))
                    except Exception as error:
                        if command=='download_map':
                            self.events.put(('download_map_error',generation,str(error)))
                        else:
                            # Command timeout/failure does NOT mean the robot is offline.
                            # Do not retry automatically because the controller may have accepted it.
                            self.events.put(('command_error',generation,f'{command}: {error}'))
                except Exception as e:
                    self.events.put(('command_error',generation,'명령 worker 오류: '+str(e)))

        def jog_work():
            was_active=False
            while not stop.is_set():
                with self._jog_lock:
                    desired = jog_packet(self._jog_desired,time.monotonic())
                    if desired is not None and controller_manual_reason(self.live,self.last_state,time.monotonic()):
                        self._jog_desired=None;desired=None
                if desired is not None:
                    try:
                        started=time.monotonic()
                        self.events.put(('jog_io',generation,('TX',desired,None)))
                        result=jog_client.request(2010,desired)
                        self.events.put(('jog_io',generation,('RX',desired,(result,(time.monotonic()-started)*1000.0))))
                        was_active=True
                    except Exception as e:
                        self.events.put(('jog_io',generation,('ERROR',desired,str(e))))
                        # Do not flood the controller every 250 ms after a rejected command.
                        with self._jog_lock:
                            self._jog_desired=None
                        was_active=True
                    jog_wakeup.wait(.25); jog_wakeup.clear()
                else:
                    if was_active:
                        try:
                            started=time.monotonic(); result=jog_client.request(2000,None)
                            self.events.put(('jog_io',generation,('STOP',None,(result,(time.monotonic()-started)*1000.0))))
                        except Exception as e:
                            self.events.put(('jog_io',generation,('ERROR_STOP',None,str(e))))
                        was_active=False
                    jog_wakeup.wait(.1); jog_wakeup.clear()
            try:
                jog_client.close()
            except Exception:
                pass

        def sensor_work():
            # Persistent state session: avoid reconnecting for every high-rate 1101 packet.
            session=PersistentSeerSession(host,19204,timeout=2.0)
            last_error=''
            try:
                session.connect()
                while not stop.is_set():
                    try:
                        result=session.request(1101,None)
                        self.events.put(('sensor1101',generation,result))
                        last_error=''
                    except Exception as e:
                        text=str(e)
                        if text != last_error:
                            self.events.put(('sensor1101_error',generation,text)); last_error=text
                        try: session.connect()
                        except Exception: pass
                    stop.wait(.15)
            finally:
                session.close()

        def operation_info_work():
            # RoboShop 운영 정보는 1100/1101 한 응답에 모두 들어오지 않는다.
            # 공식 상태 API를 같은 19204 세션에서 저속 주기로 추가 조회한다.
            session=PersistentSeerSession(host,19204,timeout=2.0)
            # 1000=robot info, 1002=run state, 1007=battery, 1020=task.
            # 1021/1022는 firmware가 지원하는 경우 relocation/map-load 상태를 제공한다.
            apis=(1000,1002,1007,1020,1021,1022)
            last_errors={}
            try:
                session.connect()
                while not stop.is_set():
                    bundle={}
                    for api in apis:
                        if stop.is_set(): break
                        try:
                            bundle[api]=session.request(api,None)
                            last_errors.pop(api,None)
                        except Exception as e:
                            msg=str(e)
                            # 선택 API 하나가 미지원이어도 나머지 운영정보는 계속 수집한다.
                            if last_errors.get(api)!=msg:
                                self.events.put(('operation_api_error',generation,(api,msg)))
                                last_errors[api]=msg
                            try: session.connect()
                            except Exception: pass
                    if bundle:
                        self.events.put(('operation_info',generation,bundle))
                    stop.wait(1.5)
            finally:
                session.close()

        def laser_work():
            # 1009 fallback on its own persistent state connection.
            session=PersistentSeerSession(host,19204,timeout=2.0)
            last_error=''
            try:
                session.connect()
                while not stop.is_set():
                    try:
                        result=session.request(1009,None)
                        self.events.put(('laser',generation,result))
                        last_error=''
                    except Exception as e:
                        text=str(e)
                        if text != last_error:
                            self.events.put(('laser_error',generation,text)); last_error=text
                        try: session.connect()
                        except Exception: pass
                    stop.wait(.35)
            finally:
                session.close()

        threading.Thread(target=state_work,daemon=True,name='seer-state').start()
        threading.Thread(target=command_work,daemon=True,name='seer-command').start()
        threading.Thread(target=jog_work,daemon=True,name='seer-jog').start()
        threading.Thread(target=sensor_work,daemon=True,name='seer-all2-sensor').start()
        threading.Thread(target=laser_work,daemon=True,name='seer-laser-fallback').start()
        threading.Thread(target=operation_info_work,daemon=True,name='seer-operation-info').start()

    def disconnect(self):
        if hasattr(self,'pad_enabled'):self.pad_enabled.set(False);self.pad_gate.reset()
        if hasattr(self,'studio_runner') and self.studio_runner.active:self.studio_runner.cancel()
        self.control_enabled=False
        self.downloading_map=False
        self.real_laser_points=[]
        self.real_laser_scans=[]
        self.last_laser_rx=0.0
        self.last_sensor1101_rx=0.0
        self._laser_shape_logged=False
        self._sensor1101_logged=False
        self.lidar_source='NONE'
        self.lidar_raw_count=0
        self.block_status={}
        self.real_mapping_active=False
        self.real_mapping_cloud=[]
        self.real_mapping_trace=[]
        self._real_mapping_cells=set()
        with self._jog_lock:
            self._jog_desired=None
        if self._jog_wakeup:
            self._jog_wakeup.set()
        while not self.commands.empty():
            try:self.commands.get_nowait()
            except queue.Empty:break
        self.release_drive()
        self.sim.stop()
        self.cancel_tasks()
        self.connected = False
        self.live,self.raw = {},{}
        self.last_state = 0
        self.generation += 1
        if self.worker_stop:
            self.worker_stop.set()
        self.connection_text.config(text='연결 해제됨')

    def diagnose(self):
        try:
            host = self.host.get().strip()
            ports = [int(v.strip()) for v in self.ports.get().split(',')]
            if not host or not 1 <= len(ports) <= 10 or any(not 1<=v<=65535 for v in ports):
                raise ValueError()
        except ValueError:
            messagebox.showerror('입력 오류','IP와 1~65535 범위의 포트(최대 10개)를 입력하세요.'); return
        self.diagnostic.config(text='진단 중…')
        def work():
            self.events.put(('probe',None,port_probe(host,ports)))
        threading.Thread(target=work,daemon=True).start()

    def send_command(self,name,payload=None):
        def send():
            if not self.real or not self.connected:raise ValueError('실기 연결이 필요합니다.')
            if not self.control_enabled:raise ValueError('상단에서 실기 · 제어 모드로 연결하세요.')
            if self.loading_map and name not in ('cancel','pause'):raise ValueError('지도 전환 완료를 기다리세요.')
            if time.monotonic()-self.last_state>3:raise ValueError('상태 데이터가 오래되어 명령을 전송하지 않습니다.')
            if name not in ('cancel','pause') and (self.live.get('emergency') is True):raise ValueError('하드웨어 비상정지가 활성화되어 있습니다.')
            if name in ('cancel','pause'):
                while not self.commands.empty():
                    try:self.commands.get_nowait()
                    except queue.Empty:break
            try:self.commands.put_nowait((name,payload))
            except queue.Full:raise ValueError('명령 응답 대기 중입니다.')
            if name=='load_map':
                self.loading_map=True
                self.robot_stations={}
                self.map=MapModel(dict(format='amr-console-map-v1',name='로봇 지도 로드 중',nodes=[dict(id='_origin',x=0,y=0)],edges=[],walls=[]))
                self.map.nodes={}
                self.refresh_nodes()
            self.log('TX',name+' '+str(payload or {}))
        self.guarded(send)

    def robot_load_map(self):
        if not self.real or not self.connected:
            messagebox.showinfo('Load Map','실기 연결 후 로봇에 저장된 지도 목록을 조회합니다.');return
        if self.loading_map:return
        try:self.commands.put_nowait(('maps',None))
        except queue.Full:self.log('INFO','명령 처리 후 다시 시도하세요.')

    def _queue_map_download(self,map_name):
        if not self.real or not self.connected:
            raise ValueError('실기 연결이 필요합니다.')
        if self.downloading_map:
            raise ValueError('이미 지도 다운로드 중입니다.')
        if not isinstance(map_name,str) or not map_name:
            raise ValueError('로봇의 현재 지도 이름을 확인하지 못했습니다.')
        try:self.commands.put_nowait(('download_map',{'map_name':map_name}))
        except queue.Full:raise ValueError('다른 요청 처리 중입니다. 잠시 후 다시 시도하세요.')
        self.downloading_map=True
        self.log('TX','download_map '+map_name+' (4011)')
        self.connection_text.config(text='로봇 지도 다운로드 중…')

    def pull_robot_map(self):
        if not self.real or not self.connected:
            messagebox.showinfo('Pull Map','실기 연결 후 현재 로봇 지도를 다운로드합니다.');return
        if self.downloading_map:
            self.log('INFO','이미 Fast Pull 진행 중입니다.');return
        self.downloading_map=True
        self.connection_text.config(text='Fast Pull · 로봇 지도 다운로드 중…')
        generation=self.generation
        host=self.host.get().strip()
        known_map=(self.current_robot_map or '').strip()
        # self.profile is a pathlib.Path. SeerClient requires the parsed profile dict.
        # Parse it before starting the worker so Fast Pull cannot accidentally pass a Path
        # into ReadOnlyClient (which expects .get()).
        try:
            profile_data=json.loads(self.profile.read_text(encoding='utf-8-sig'))
        except (OSError,ValueError,TypeError) as e:
            self.downloading_map=False
            self.connection_text.config(text='실기 연결됨 · Fast Pull 설정 오류')
            self.log('ERROR','FAST PULL 프로파일 읽기 실패: '+str(e))
            return
        self.log('INFO',f'FAST PULL 시작 · cached map={known_map or "UNKNOWN"}')
        def work():
            try:
                from .live import SeerClient
                client=SeerClient(host,profile_data)
                total=time.monotonic()
                map_name=known_map
                t1300=0.0
                if not map_name:
                    t=time.monotonic()
                    maps=client.query(1300)
                    t1300=(time.monotonic()-t)*1000.0
                    map_name=str(maps.get('current_map') or '').strip()
                    if not map_name:
                        raise ValueError('1300 응답에서 current_map을 확인하지 못했습니다.')
                t=time.monotonic()
                data=client.download_map(map_name)
                t4011=(time.monotonic()-t)*1000.0
                network_total=(time.monotonic()-total)*1000.0
                self.events.put(('fast_map_downloaded',generation,(map_name,data,t1300,t4011,network_total)))
            except Exception as e:
                self.events.put(('fast_map_error',generation,str(e)))
        threading.Thread(target=work,daemon=True,name='seer-fast-pull').start()

    def _prepare_robot_map_async(self,map_name,data):
        """Parse/save a potentially large 4011 SMAP away from the Tk thread."""
        self._map_prepare_token += 1
        token=self._map_prepare_token
        generation=self.generation
        self.connection_text.config(text='로봇 지도 수신 완료 · 백그라운드 파싱 중…')
        self.log('INFO',f'4011 지도 수신 완료: {map_name} · 백그라운드 파싱 시작')
        def work():
            try:
                started=time.monotonic()
                model=load_smap_data(data,map_name)
                USER_DIR.mkdir(parents=True,exist_ok=True)
                cache=USER_DIR/'maps';cache.mkdir(parents=True,exist_ok=True)
                safe=''.join(ch if ch.isalnum() or ch in '._-' else '_' for ch in map_name).strip('._') or 'robot_map'
                target=cache/(safe+'.smap')
                source=getattr(model,'smap_source',data)
                # Compact JSON is much faster for large maps while preserving the full source.
                target.write_text(json.dumps(source,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
                elapsed=(time.monotonic()-started)*1000.0
                self.events.put(('map_prepared',generation,(token,map_name,model,target,elapsed)))
            except Exception as e:
                self.events.put(('map_prepare_error',generation,(token,str(e))))
        threading.Thread(target=work,daemon=True,name='seer-map-prepare').start()

    def _install_prepared_robot_map(self,map_name,model,target,elapsed):
        self.map=model;self.map_path=target;self.current_robot_map=map_name;self.map_dirty=False
        self.robot_stations=dict(model.nodes)
        self.tasks=[];self.refresh_tasks();self.refresh_nodes()
        self.zoom,self.pan=1.0,[0.0,0.0]
        self._view_transform=None
        self.draw_map()
        self.log('INFO',f'로봇 지도 표시 완료: {map_name} · 화면용 {len(model.cloud)} pts · {len(model.nodes)} nodes · {len(getattr(model,"curves",[]))} paths · 파싱 {elapsed:.0f} ms')
        if getattr(model,'curve_diagnostics',None):
            self.log('MAP','Path 형상 분석: '+model.curve_diagnostics)
        self.log('INFO','원본 SMAP 캐시: '+str(target))
        self.connection_text.config(text='실기 연결됨 · 지도 표시 완료')
        self._load_lidar_alignment()
        self.after(1800,lambda:self.align_lidar_to_map(auto=True))
        # 지도는 먼저 즉시 표시하고, 실제 로봇 노드(1301)는 뒤에서 별도 조회한다.
        try:
            self.commands.put_nowait(('stations',None))
            self.log('TX','stations {} (1301) · 실제 주행 가능 노드 동기화')
        except queue.Full:
            self.after(300,lambda: self.commands.put_nowait(('stations',None)) if self.commands.empty() else None)
            self.log('INFO','노드 동기화 대기 · 지도 표시는 완료됨')

    def show_robot_maps(self,data):
        maps=data.get('maps',[])
        if not isinstance(maps,list) or not all(isinstance(x,str) for x in maps):
            self.log('ERROR','지원하지 않는 지도 목록 응답');return
        window=tk.Toplevel(self);window.title('Load Map · 로봇 저장 지도');self._fit_dialog(window,460,260,420,260)
        generation=self.generation
        ttk.Label(window,text='현재 로봇 지도: '+str(data.get('current_map','UNKNOWN'))).pack(pady=18)
        selected=tk.StringVar(value=maps[0] if maps else '')
        ttk.Combobox(window,textvariable=selected,values=maps,state='readonly',width=40).pack(pady=8)
        ttk.Label(window,text='다운로드/표시는 조회 전용에서도 가능합니다.\n지도 전환은 실기 · 제어 모드에서만 가능합니다.').pack(pady=10)
        row=ttk.Frame(window);row.pack(pady=8)
        def pull():
            if generation!=self.generation:
                window.destroy();return
            if selected.get() not in maps:return
            self.guarded(lambda:self._queue_map_download(selected.get()))
            if self.downloading_map:window.destroy()
        def apply():
            if generation!=self.generation:
                window.destroy();return
            if selected.get() not in maps:return
            self.send_command('load_map',{'map_name':selected.get()})
            if self.loading_map:window.destroy()
        ttk.Button(row,text='다운로드 + 화면 표시',command=pull,state='normal' if maps else 'disabled').pack(side='left',padx=5)
        ttk.Button(row,text='로봇에 지도 로드',command=apply,state='normal' if maps and self.control_enabled else 'disabled').pack(side='left',padx=5)
        if not self.control_enabled:ttk.Label(window,text='현재 연결은 조회 전용: 다운로드/표시는 사용 가능').pack()

    def fetch_stations(self):
        if self.loading_map:return
        if not self.real or not self.connected:
            messagebox.showinfo('Stations','실기 연결 후 사용하세요.');return
        try:self.commands.put_nowait(('stations',None))
        except queue.Full:self.log('INFO','명령 처리 후 다시 조회하세요.')

    def open_smap(self):
        path=filedialog.askopenfilename(filetypes=[('SEER JSON SMAP','*.smap'),('JSON','*.json')])
        if not path:return
        def load():
            if self.task_running:raise ValueError('작업을 종료한 뒤 여세요.')
            model=load_smap(path)
            self.release_drive();self.sim.stop()
            self.map=model;self.map_path=Path(path)
            if not self.real:
                self.sim=Simulator(model);self.sim_powered=True;self.connected=True
                self.curve_edit_record=None;self.curve_drag_index=None
                self.scan_points=[];self.last_scan=0
                self.connection_text.config(text='SIM 연결됨 · 로컬 SMAP')
            self.tasks=[];self.refresh_tasks();self.refresh_nodes();self.fit_map()
            self.log('INFO','SMAP 로드: '+path)
        self.guarded(load)

    def choose_profile(self):
        path = filedialog.askopenfilename(filetypes=[('API profile','*.json')])
        if path:
            self.profile = Path(path)
            self.profile_label.config(text=path)

    def save_settings(self):
        def save():
            USER_DIR.mkdir(parents=True,exist_ok=True)
            (USER_DIR/'settings.json').write_text(json.dumps(dict(host=self.host.get(),ports=self.ports.get(),
                profile=str(self.profile),pose_autosave=self.pose_autosave.get(),pose_autorecover=self.pose_autorecover.get(),
                pose_autoconfirm=self.pose_autoconfirm.get(),pose_resume_loop=self.pose_resume_loop.get(),
                pose_conf_threshold=self.pose_conf_threshold.get()),ensure_ascii=False,indent=2),encoding='utf-8')
            self.log('INFO','연결 설정 저장됨. 재실행 시 항상 시뮬레이션으로 시작합니다.')
        self.guarded(save)

    def action(self,name):
        if name in ('stop','cancel','motor_off','pause') and hasattr(self,'pad_enabled'):
            self.pad_enabled.set(False);self.release_drive();self.pad_gate.reset()
        if hasattr(self,'studio_runner') and name in ('stop','cancel','motor_off'):
            self.studio_charge_inhibit=True;self.sim.auto_charge['enabled']=False
        if hasattr(self,'studio_runner') and self.studio_runner.active:
            if name=='pause':self.studio_runner.pause(time.monotonic());return
            if name=='resume':self.studio_runner.resume(time.monotonic());return
            if name in ('cancel','stop','motor_off','mapping_start','mapping_stop','charge'):self.cancel_tasks()
        if self.real:
            if name in ('pause','resume','cancel','stop'):
                self.send_command('cancel' if name=='stop' else name)
            elif name=='mapping_start':
                if not messagebox.askyesno('맵 생성 시작','실제 로봇에서 새 SLAM 스캔을 시작합니다.\n기존 주행 Task는 중지하고 주변 안전을 확인하세요.\n\n계속할까요?',parent=self):
                    return
                # Enter RoboShop-like blank mapping canvas immediately, before 6100 ACK.
                self.real_mapping_active=True
                self.real_mapping_cloud=[]
                self.real_mapping_trace=[]
                self._real_mapping_cells=set()
                self._mapping_laser_diag_at=0.0
                self.mapping_status.config(text='SLAM: 시작 요청 · 빈 화면 준비',fg=ORANGE)
                self.command_status.config(text='맵 생성 시작 요청 · 빈 작업화면 전환 · 6100 ACK 대기',fg=ORANGE)
                self.draw_map()
                self.send_command('slam_start',{})
            elif name=='mapping_stop':
                if not messagebox.askyesno('맵 생성 종료','현재 SLAM 스캔을 종료합니다.\n종료 후 로봇의 현재 지도를 다시 Pull합니다.\n\n계속할까요?',parent=self):
                    return
                self.mapping_status.config(text='SLAM: 종료 요청',fg=ORANGE)
                self.send_command('slam_stop',{})
            elif name=='charge':
                self.return_to_charge()
            else:
                self.unavailable(name)
            return
        def run():
            self.sim_required()
            if name in ('stop','cancel','motor_off','mapping_start','mapping_stop','charge'):
                self.held = None
                self.cancel_tasks()
            self.sim.command(name)
            self.log('COMMAND','[SIM] '+name)
        self.guarded(run)


    def _reloc_ready(self):
        if not (self.real and self.connected and self.control_enabled):
            messagebox.showerror('재배치','실기 · 제어 모드로 먼저 연결하세요.',parent=self); return False
        if self.real_mapping_active:
            messagebox.showwarning('재배치','맵 생성 중에는 재배치를 시작할 수 없습니다.',parent=self); return False
        return True

    def start_manual_reloc(self):
        if not self._reloc_ready(): return
        self.reloc_mode='manual'; self.reloc_drag_start=None; self.reloc_candidate=None
        self.edit_mode.set('선택')
        self.reloc_info.config(text='수동 재배치 대기 · 맵에서 실제 로봇 위치를 누른 채 실제 방향으로 드래그하세요.',fg=ORANGE)
        self.command_status.config(text='RELOC MANUAL · 맵 클릭→드래그로 X/Y/Heading 지정',fg=ORANGE)
        self.log('RELOC','수동 재배치 모드 · 맵 클릭/드래그로 x,y,angle 지정')

    def start_auto_reloc(self):
        if not self._reloc_ready(): return
        self.reloc_mode='auto'; self.reloc_drag_start=None; self.reloc_candidate=None
        self.edit_mode.set('선택')
        self.reloc_info.config(text='자동 재배치 대기 · 맵에서 실제 로봇의 대략적인 위치만 클릭하세요. 방향은 360° 범위에서 RBK가 탐색합니다.',fg=ORANGE)
        self.command_status.config(text='RELOC AUTO · 맵에서 대략 위치 클릭 · angle 생략(2π 탐색)',fg=ORANGE)
        self.log('RELOC','자동 재배치 모드 · 클릭한 x,y로 2002 전송, angle 생략하여 2π 탐색')

    def _send_reloc(self,x,y,angle=None):
        payload={'x':float(x),'y':float(y)}
        if angle is not None:
            payload['angle']=float(angle)
        mode='수동' if angle is not None else '자동(360°)'
        msg=f"{mode} 재배치를 실행할까요?\n\nX={x:.3f} m\nY={y:.3f} m"
        if angle is not None:
            msg += f"\nHeading={math.degrees(angle):.1f}°"
        else:
            msg += "\nHeading=자동 탐색(2π)"
        if not messagebox.askyesno('자기위치 재배치',msg,parent=self):
            return
        self.log('RELOC',f'2002 Reloc TX · {payload}')
        self.command_status.config(text=f'2002 Reloc 전송 · {mode} · X={x:.3f} Y={y:.3f}',fg=ORANGE)
        self.send_command('relocate',payload)

    def confirm_relocation(self):
        if not self._reloc_ready():
            return
        msg='Laser/맵 정합과 Localization Confidence를 확인했습니까?\n현재 위치를 2003 ConfirmLoc으로 확정할까요?'
        if not messagebox.askyesno('위치 확정',msg,parent=self):
            return
        self.log('RELOC','2003 ConfirmLoc TX')
        self.send_command('confirm_loc',{})

    def cancel_relocation(self):
        if not (self.real and self.connected and self.control_enabled): return
        self.reloc_mode=None; self.reloc_drag_start=None; self.reloc_candidate=None
        self.log('RELOC','2004 CancelReloc TX')
        self.send_command('cancel_reloc',{})
        self.draw_map()

    def return_to_charge(self):
        if not self.real:
            self.action('charge'); return
        if not (self.connected and self.control_enabled):
            messagebox.showerror('충전 노드 복귀','실기 · 제어 모드로 먼저 연결하세요.',parent=self); return
        candidates=[]
        for key,st in self.robot_stations.items():
            raw=st.get('raw1301',st) if isinstance(st,dict) else {}
            typ=str(st.get('type',raw.get('type',raw.get('className',''))) if isinstance(st,dict) else '')
            kind=str(st.get('kind','') if isinstance(st,dict) else '')
            raw_text=(json.dumps(raw,ensure_ascii=False) if isinstance(raw,dict) else str(raw)).lower()
            if kind=='dock' or typ.lower() in ('chargepoint','charge','charger','chargingpoint','charge_mark','chargemark') or str(key).upper().startswith(('CP','CHARGE')) or 'charge' in raw_text or 'dock' in raw_text or '충전' in raw_text:
                x=self._finite_value(st.get('x')) if isinstance(st,dict) else None
                y=self._finite_value(st.get('y')) if isinstance(st,dict) else None
                if x is not None and y is not None:candidates.append((key,st,x,y))
        if not candidates:
            messagebox.showerror('충전 노드 복귀','현재 활성 맵에서 ChargePoint/충전 노드를 찾지 못했습니다.\nStations 조회 후 충전 노드 타입을 확인하세요.',parent=self); return
        state=self.current_state(); rx=self._finite_value(state.get('x')); ry=self._finite_value(state.get('y'))
        if rx is not None and ry is not None:
            candidates.sort(key=lambda v:(v[2]-rx)**2+(v[3]-ry)**2)
        key,station,_,_=candidates[0]
        if not messagebox.askyesno('충전 노드 복귀',f'충전 노드 {key} 로 복귀할까요?\n\n3051 Path Navigation으로 이동합니다.',parent=self):return
        self.target.set(key); self._select_node(key)
        self.log('CHARGE',f'충전 노드 복귀 요청 · target={key} · payload={self._station_nav_payload(station)}')
        self.navigate()

    def _lidar_alignment_file(self):
        return USER_DIR/'lidar_map_alignment.json'

    def _load_lidar_alignment(self):
        self.lidar_map_alignment=(0.0,0.0,0.0); self.lidar_alignment_score=0.0
        key=(self.current_robot_map or getattr(self.map,'name','')).strip()
        if not key:return
        try:
            data=json.loads(self._lidar_alignment_file().read_text(encoding='utf-8'))
            rec=data.get(self.host.get().strip(),{}).get(key,{})
            tr=rec.get('transform')
            if isinstance(tr,list) and len(tr)>=3:
                self.lidar_map_alignment=tuple(float(v) for v in tr[:3])
                self.lidar_alignment_score=float(rec.get('score',0.0))
                self.log('LASER',f'저장된 LiDAR↔Map 정합값 로드 · dx={tr[0]:.3f} dy={tr[1]:.3f} yaw={math.degrees(tr[2]):.2f}° score={self.lidar_alignment_score:.3f}')
        except Exception: pass

    def _save_lidar_alignment(self):
        key=(self.current_robot_map or getattr(self.map,'name','')).strip()
        if not key:return
        USER_DIR.mkdir(parents=True,exist_ok=True); path=self._lidar_alignment_file()
        try:data=json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
        except Exception:data={}
        host=data.setdefault(self.host.get().strip(),{})
        host[key]={'transform':list(self.lidar_map_alignment),'score':self.lidar_alignment_score,'updated':datetime.now().isoformat(timespec='seconds')}
        path.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')

    def _lidar_map_xy(self,x,y):
        dx,dy,da=self.lidar_map_alignment
        state=self.current_state(); rx=self._finite_value(state.get('x')); ry=self._finite_value(state.get('y'))
        if rx is None or ry is None:return x+dx,y+dy
        c,s=math.cos(da),math.sin(da); ux=x-rx; uy=y-ry
        return rx+c*ux-s*uy+dx, ry+s*ux+c*uy+dy

    @staticmethod
    def _alignment_score(cells, pts, rx, ry, tr, cell=.10):
        dx,dy,da=tr; c,s=math.cos(da),math.sin(da); hits=0
        for x,y in pts:
            ux=x-rx; uy=y-ry; wx=rx+c*ux-s*uy+dx; wy=ry+s*ux+c*uy+dy
            gx=int(round(wx/cell)); gy=int(round(wy/cell)); found=False
            for ox in (-1,0,1):
                for oy in (-1,0,1):
                    if (gx+ox,gy+oy) in cells: found=True; break
                if found:break
            hits+=1 if found else 0
        return hits/max(1,len(pts))

    def align_lidar_to_map(self, auto=False):
        if self._lidar_alignment_running:return
        cloud=list(getattr(self.map,'cloud',[]) or []); pts=list(self.real_laser_points or [])
        st=self.current_state(); rx=self._finite_value(st.get('x')); ry=self._finite_value(st.get('y'))
        if not self.real or not self.connected or len(cloud)<300 or len(pts)<80 or rx is None or ry is None:
            if not auto:messagebox.showinfo('LiDAR 정합','실기 연결 + Pull Map 후 LiDAR 점군이 수신된 상태에서 실행하세요.',parent=self)
            return
        self._lidar_alignment_running=True; map_name=self.current_robot_map or self.map.name
        cloud_sample=cloud[::max(1,len(cloud)//16000)]; scan=pts[::max(1,len(pts)//320)]
        self.log('LASER',f'LiDAR↔Map 정합 시작 · map={map_name} mapPts={len(cloud_sample)} scanPts={len(scan)}')
        def work():
            try:
                cell=.10; cells={(int(round(x/cell)),int(round(y/cell))) for x,y in cloud_sample}
                best=(-1.0,(0.0,0.0,0.0))
                for ix in range(-8,9):
                    dx=ix*.10
                    for iy in range(-8,9):
                        dy=iy*.10
                        for ia in range(-6,7):
                            da=math.radians(ia*3.0); sc=self._alignment_score(cells,scan,rx,ry,(dx,dy,da),cell)
                            if sc>best[0]:best=(sc,(dx,dy,da))
                _,(bdx,bdy,bda)=best; fine=best
                for ix in range(-4,5):
                    dx=bdx+ix*.02
                    for iy in range(-4,5):
                        dy=bdy+iy*.02
                        for ia in range(-4,5):
                            da=bda+math.radians(ia*.5); sc=self._alignment_score(cells,scan,rx,ry,(dx,dy,da),cell)
                            if sc>fine[0]:fine=(sc,(dx,dy,da))
                self.events.put(('lidar_map_alignment_done',self.generation,(map_name,fine[1],fine[0],auto)))
            except Exception as e:self.events.put(('lidar_map_alignment_error',self.generation,str(e)))
        threading.Thread(target=work,daemon=True,name='seer-lidar-map-align').start()

    def _last_pose_file(self):
        return USER_DIR/'last_pose.json'

    def _current_map_name(self):
        return str((self.current_robot_map if self.real else getattr(self.map,'name','')) or '')

    def _confidence_value(self,state=None):
        state=state or self.current_state()
        for k in ('localization','confidence','localization_confidence'):
            try:
                v=state.get(k)
                if v is not None: return float(v)
            except Exception: pass
        return None

    def _pose_record_now(self):
        state=self.current_state()
        vals=[]
        for k in ('x','y','theta'):
            try: vals.append(float(state.get(k)))
            except Exception: return None
        return normalized_pose_record(self._current_map_name(),vals[0],vals[1],vals[2],
                                      self._confidence_value(state),'REAL' if self.real else 'SIMULATION')

    def save_last_pose(self,force=False):
        if not self.pose_autosave.get() and not force:return False
        rec=self._pose_record_now()
        if not rec or not rec.get('map_name'):return False
        try:
            save_pose(self._last_pose_file(),rec)
            self._last_pose_save_at=time.monotonic()
            self._update_pose_status(rec)
            return True
        except Exception as e:
            self.log('POSE','마지막 위치 저장 실패: '+str(e)); return False

    def _load_last_pose(self):
        try:return load_pose(self._last_pose_file())
        except Exception as e:
            self.log('POSE','마지막 위치 읽기 실패: '+str(e)); return None

    def _update_pose_status(self,rec=None):
        if not hasattr(self,'pose_status'):return
        rec=rec or self._load_last_pose()
        if not rec:
            self.pose_status.config(text='저장 위치 없음',fg=MUTED);return
        txt=(f"{rec.get('source','?')} · {rec.get('map_name','?')}\n"
             f"X {rec.get('x',0):.3f}  Y {rec.get('y',0):.3f}  θ {math.degrees(rec.get('theta',0)):.1f}°\n"
             f"Conf {rec.get('confidence','—')} · {rec.get('timestamp','')}")
        self.pose_status.config(text=txt,fg=GREEN if map_matches(rec.get('map_name'),self._current_map_name()) else ORANGE)

    def show_last_pose(self):
        rec=self._load_last_pose()
        if not rec:
            messagebox.showinfo('마지막 위치','저장된 위치가 없습니다.',parent=self);return
        same=map_matches(rec.get('map_name'),self._current_map_name())
        messagebox.showinfo('마지막 위치',
            f"Map: {rec.get('map_name')}\nX: {rec.get('x'):.3f} m\nY: {rec.get('y'):.3f} m\nHeading: {math.degrees(rec.get('theta')):.1f}°\n"
            f"Confidence: {rec.get('confidence','—')}\nSaved: {rec.get('timestamp','')}\nSource: {rec.get('source','')}\n\n"
            f"현재 Map과 {'일치합니다.' if same else '일치하지 않습니다. 자동 재배치를 차단합니다.'}",parent=self)

    def clear_last_pose(self):
        if not messagebox.askyesno('저장 위치 초기화','저장된 마지막 위치를 삭제할까요?',parent=self):return
        try:self._last_pose_file().unlink(missing_ok=True)
        except Exception as e:self.log('POSE','저장 위치 삭제 실패: '+str(e))
        self._update_pose_status(None)

    def recover_last_pose(self,auto=False):
        rec=self._load_last_pose()
        if not rec:
            if not auto:messagebox.showinfo('위치 복구','저장된 마지막 위치가 없습니다.',parent=self)
            return False
        current_map=self._current_map_name()
        if not self.real and rec.get('source') != 'SIMULATION':
            self.log('POSE','[SIM] 실제 로봇의 저장 위치는 시뮬레이션에 적용하지 않습니다.')
            return False
        if not map_matches(rec.get('map_name'),current_map):
            msg=f"저장 Map({rec.get('map_name')})과 현재 Map({current_map})이 다릅니다.\n자동 위치복구를 수행하지 않습니다."
            self.log('POSE',msg.replace('\n',' '))
            if not auto:messagebox.showerror('위치 복구',msg,parent=self)
            return False
        if self.real:
            if not (self.connected and self.control_enabled):
                if not auto:messagebox.showerror('위치 복구','실기 · 제어 모드로 연결한 뒤 실행하세요.',parent=self)
                return False
            self._pending_pose_recovery=rec
            self.reloc_candidate=(rec['x'],rec['y'],rec['theta'])
            self.reloc_info.config(text='마지막 저장 Pose로 2002 Reloc 요청 · Confidence 확인 중',fg=ORANGE)
            self.send_command('relocate',{'x':rec['x'],'y':rec['y'],'angle':rec['theta']})
            self.log('POSE',f"마지막 위치 2002 Reloc · x={rec['x']:.3f} y={rec['y']:.3f} theta={math.degrees(rec['theta']):.1f}°")
        else:
            if not self.sim_powered:
                if not auto:messagebox.showerror('SIM 위치 복구','먼저 SIM 재부팅을 실행하세요.',parent=self)
                return False
            self.sim.state.x=rec['x']; self.sim.state.y=rec['y']; self.sim.state.theta=rec['theta']
            self.sim.state.localization=max(self.sim.state.localization,float(rec.get('confidence',0.95) or 0.95))
            nearest=nearest_reachable_node(self.map,rec['x'],rec['y'])
            if nearest:self.sim.state.last_node=nearest[0]
            self.log('POSE',f"[SIM] 마지막 위치 복구 · x={rec['x']:.3f} y={rec['y']:.3f} theta={math.degrees(rec['theta']):.1f}°")
            self._update_pose_status(rec); self.draw_map()
            if self.pose_resume_loop.get(): self._resume_loop_from_nearest()
        return True

    def _maybe_auto_confirm_recovery(self):
        if not (self.real and self._pending_pose_recovery):return
        conf=self._confidence_value()
        try:threshold=float(self.pose_conf_threshold.get())
        except Exception:threshold=.80
        if conf is None:return
        self.reloc_info.config(text=f'위치복구 검사 · Confidence {conf:.3f} / 기준 {threshold:.3f}',fg=GREEN if conf>=threshold else ORANGE)
        if self.pose_autoconfirm.get() and confidence_ok(conf,threshold):
            self.send_command('confirm_loc',{})
            self.log('POSE',f'Confidence {conf:.3f} ≥ {threshold:.3f} · 2003 자동 위치 확정 요청')
            self._pending_pose_recovery=None
            if self.pose_resume_loop.get(): self.after(800,self._resume_loop_from_nearest)

    def find_nearest_node(self,move=False):
        state=self.current_state(); x=self._finite_value(state.get('x')); y=self._finite_value(state.get('y'))
        if x is None or y is None:
            messagebox.showerror('가까운 노드','현재 위치를 확인할 수 없습니다.',parent=self);return None
        nearest=nearest_reachable_node(self.map,x,y)
        if not nearest:
            messagebox.showinfo('가까운 노드','접근 가능한 노드를 찾지 못했습니다.',parent=self);return None
        key,dist=nearest
        self.selected=key; self.target.set(key); self._select_node(key); self.draw_map()
        self.log('NAV',f'가까운 접근 가능 노드 · {key} · {dist:.3f} m')
        if move:
            if self.real:self.navigate()
            else:self.sim.navigate(key)
        else:messagebox.showinfo('가까운 노드',f'{key}\n거리: {dist:.3f} m',parent=self)
        return nearest

    def _resume_loop_from_nearest(self):
        chain=self._mission_active_chain()
        route=chain.get('loop_route') if isinstance(chain,dict) else None
        if not route:return
        state=self.current_state(); x=self._finite_value(state.get('x')); y=self._finite_value(state.get('y'))
        if x is None or y is None:return
        candidates=[]
        for idx,key in enumerate(route):
            n=self.map.nodes.get(key)
            if not n:continue
            candidates.append((math.hypot(n['x']-x,n['y']-y),idx,key))
        if not candidates:return
        _,idx,key=min(candidates)
        rotated=route[idx:]+route[:idx]
        if rotated and rotated[-1]!=rotated[0]: rotated.append(rotated[0])
        chain['loop_route']=rotated
        self.log('TASK',f'위치복구 후 순환미션 재개 준비 · nearest={key} · route={rotated}')
        try:self.run_tasks()
        except Exception as e:self.log('TASK','순환미션 재개 실패: '+str(e))

    def sim_power_off(self):
        if self.real:
            messagebox.showinfo('SIM 전원 OFF','시뮬레이션 모드에서만 사용하세요.',parent=self);return
        self.save_last_pose(force=True)
        if hasattr(self,'studio_runner') and self.studio_runner.active:self.cancel_tasks()
        self.sim.stop(); self.sim_powered=False
        self.connection_text.config(text='SIM 전원 OFF · 마지막 위치 저장됨')
        self.log('POSE','[SIM] 전원 OFF 시뮬레이션 · runtime state 정지 / 마지막 pose 보존')

    def sim_reboot(self):
        if self.real:
            messagebox.showinfo('SIM 재부팅','시뮬레이션 모드에서만 사용하세요.',parent=self);return
        if hasattr(self,'studio_runner') and self.studio_runner.active:self.cancel_tasks()
        from .model import Simulator
        self.sim=Simulator(self.map); self.sim_powered=True; self.connected=True
        if hasattr(self,'studio_config'):self._studio_apply_sim_settings()
        self.connection_text.config(text='SIM 재부팅 · 저장 위치 확인')
        self._startup_pose_recovery_check()
        self.draw_map()

    def _startup_pose_recovery_check(self):
        rec=self._load_last_pose(); self._update_pose_status(rec)
        if not rec:return
        if not self.real:
            # Restore before the first tick can overwrite the saved pose.
            if rec.get('source')=='SIMULATION' and map_matches(rec.get('map_name'),self._current_map_name()):
                self.recover_last_pose(auto=True)
            return
        if not self.pose_autorecover.get():return
        key=(rec.get('map_name'),rec.get('timestamp'),self.real)
        if key==self._last_pose_prompted_key:return
        if not map_matches(rec.get('map_name'),self._current_map_name()):return
        self._last_pose_prompted_key=key
        if self.real:
            # Never confirm blindly; only send 2002 when control mode is available.
            if self.control_enabled:self.after(300,lambda:self.recover_last_pose(auto=True))
        else:self.after(300,lambda:self.recover_last_pose(auto=True))

    def acquire_control(self):
        if not (self.real and self.connected and self.control_enabled):
            messagebox.showerror('제어권','실기 · 제어 모드로 먼저 연결하세요.',parent=self); return
        if not messagebox.askyesno('제어권 가져오기',
            '현재 RoboShop/RDS/다른 클라이언트가 로봇 제어권을 가지고 있을 수 있습니다.\n'
            '4005 Preempt Control로 제어권을 가져오면 그쪽의 standalone 제어가 중단될 수 있습니다.\n\n계속할까요?',parent=self):
            return
        self.log('CONTROL',f'4005 Preempt Control 요청 · nick_name={self.control_nick}')
        self.command_status.config(text=f'CONTROL TX · 4005 제어권 요청 · {self.control_nick}',fg=ORANGE)
        self.send_command('lock_control',{'nick_name':self.control_nick})

    def release_control(self):
        if not (self.real and self.connected and self.control_enabled):
            return
        self.release_drive(force=True)
        self.log('CONTROL',f'4006 Release Control 요청 · nick_name={self.control_nick}')
        self.command_status.config(text=f'CONTROL TX · 4006 제어권 해제 · {self.control_nick}',fg=ORANGE)
        self.send_command('unlock_control',{'nick_name':self.control_nick})

    @staticmethod
    def _bool_value(value):
        if isinstance(value,bool): return value
        if isinstance(value,(int,float)): return bool(value)
        if isinstance(value,str):
            v=value.strip().lower()
            if v in ('true','1','yes','on'): return True
            if v in ('false','0','no','off',''): return False
        return value

    @staticmethod
    def _finite_value(value):
        try:
            if isinstance(value,bool): return None
            v=float(value)
            return v if math.isfinite(v) else None
        except (TypeError,ValueError):
            return None

    def _station_option_summary(self,node):
        if not isinstance(node,dict): return ''
        r=self._finite_value(node.get('r',node.get('angle')))
        deg=math.degrees(r) if r is not None else None
        parts=[]
        if deg is not None: parts.append(f'각도 {deg:.1f}° ({r:.3f} rad)')
        if 'spin' in node: parts.append(f'spin={self._bool_value(node.get("spin"))}')
        typ=node.get('type') or node.get('station_type') or node.get('className')
        if typ: parts.append(f'type={typ}')
        if node.get('desc'): parts.append(f'desc={node.get("desc")}')
        nav=self._station_nav_payload(node,include_id=False)
        extra=[f'{k}={v}' for k,v in nav.items() if k not in ('angle','spin')]
        if extra: parts.append(' · '.join(extra))
        return ' | '.join(parts)

    def _station_nav_payload(self,node,include_id=True):
        """Build 3051 payload from the full RoboShop station object.

        1301/SMAP fields are preserved verbatim in `robot_stations`.  Only
        destination options known to the navigation interface are sent, so
        display-only fields such as x/y/type/desc do not cause parameter errors.
        """
        node=node if isinstance(node,dict) else {}
        payload={}
        if include_id and node.get('id'): payload['id']=str(node['id'])

        # RoboShop/1301 normally calls station heading `r`; navigation calls it
        # `angle`. Keep an explicit angle if present, otherwise translate r.
        angle=self._finite_value(node.get('angle'))
        if angle is None: angle=self._finite_value(node.get('r'))
        if angle is not None: payload['angle']=angle

        # Exact navigation option names exposed by newer SEER interfaces.
        numeric=('maxSpeed','maxWSpeed','maxAcc','maxWAcc','duration','orientation',
                 'delay','startRotDir','endRotDir','reachDist','reachAngle')
        aliases={
            'maxspeed':'maxSpeed','max_speed':'maxSpeed',
            'maxwspeed':'maxWSpeed','max_w_speed':'maxWSpeed','maxrot':'maxWSpeed',
            'maxacc':'maxAcc','max_acc':'maxAcc',
            'maxwacc':'maxWAcc','max_w_acc':'maxWAcc','maxrotacc':'maxWAcc',
            'reachdist':'reachDist','reach_dist':'reachDist',
            'reachangle':'reachAngle','reach_angle':'reachAngle',
            'startrotdir':'startRotDir','endrotdir':'endRotDir',
        }
        normalized=dict(node)
        props=node.get('properties')
        if isinstance(props,dict): normalized.update(props)
        attrs=node.get('attrs')
        if isinstance(attrs,dict): normalized.update(attrs)
        raw=node.get('smap_raw')
        if isinstance(raw,dict):
            normalized={**raw,**normalized}

        # Resolve common case/style variants without losing the original object.
        for k,v in list(normalized.items()):
            target=aliases.get(str(k).replace('-','_').lower())
            if target and target not in normalized: normalized[target]=v

        for key in numeric:
            if key in normalized:
                val=self._finite_value(normalized.get(key))
                if val is not None: payload[key]=val
        if 'spin' in normalized: payload['spin']=self._bool_value(normalized.get('spin'))
        for key in ('method','skillName','sourceId','taskId'):
            val=normalized.get(key)
            if val not in (None,''): payload[key]=str(val)
        return payload

    def navigate(self):
        self.pad_enabled.set(False);self.release_drive();self.pad_gate.reset()
        if hasattr(self,'studio_runner'):self.studio_charge_inhibit=False
        if hasattr(self,'studio_runner') and not self.studio_arm_safe:
            messagebox.showerror('AMR 이동','로봇팔 safe_pose 완료 후 이동하세요.',parent=self);return
        if self.real:
            target=self.target.get()
            if target not in self.robot_stations:
                messagebox.showerror('목적지','Stations 버튼으로 로봇의 실제 노드를 먼저 조회하세요.');return
            station=self.robot_stations[target]
            nav_payload=self._station_nav_payload(station)
            self.log('NAV',f'요청 준비 target={target} | station_options={nav_payload} | mode={self.live.get("mode")} safety={self.live.get("safety")} task={self.live.get("task")} current_target={self.live.get("target")}')
            self.command_status.config(text=f'NAV TX 대기 · 3051 → {target} · {self._station_option_summary(station)}',fg=ORANGE)
            self.send_command('navigate',nav_payload);return
        def run():
            self.sim_required()
            if self.task_running:
                raise ValueError('Task 목록 실행을 취소한 뒤 개별 이동하세요.')
            self.held = None
            self.sim.navigate(self.target.get())
            self.log('COMMAND','[SIM] navigate '+self.target.get())
        self.guarded(run)

    def press_drive(self,direction):
        if hasattr(self,'pad_enabled') and self.pad_enabled.get():
            self.pad_enabled.set(False);self.release_drive();self.pad_gate.reset()
        if hasattr(self,'studio_runner') and not self.studio_arm_safe:return
        if direction == 'zero':
            if self.real:
                self.release_drive(force=True)
            else:
                self.action('cancel')
            return
        if not self.manual.get():
            return
        def press():
            v,w = float(self.speed.get()), math.radians(float(self.angular.get()))
            if not math.isfinite(v) or not math.isfinite(w) or not .01<=v<=.3 or not 0<w<=.6:
                raise ValueError('속도: 0.01~0.30 m/s, 회전: 0 초과~34 deg/s')
            motion = {'forward':(v,0.0),'back':(-v,0.0),'left':(0.0,w),'right':(0.0,-w)}[direction]
            if self.held == (direction, motion):
                return
            if self.real:
                if not self.connected or not self.control_enabled:
                    raise ValueError('실기 · 제어 모드로 연결하세요.')
                if time.monotonic()-self.last_state>3:
                    raise ValueError('상태 데이터가 오래되어 수동 조작하지 않습니다.')
                if self.live.get('emergency') is True:
                    raise ValueError('하드웨어 비상정지가 활성화되어 있습니다.')
                reason=controller_manual_reason(self.live,self.last_state,time.monotonic())
                if reason:
                    self.release_drive(force=True)
                    self.command_status.config(text='수동 정지 · '+reason,fg=ORANGE)
                    self.log('JOG','수동 조작 차단 · '+reason);return
                self.held=(direction,motion)
                desired={'vx':motion[0],'vy':0.0,'w':motion[1]}
                with self._jog_lock:
                    self._jog_desired=desired
                if self._jog_wakeup:self._jog_wakeup.set()
                self.log('JOG',f'START {direction} · 2010 persistent · vx={motion[0]:.3f}, vy=0.000, w={motion[1]:.3f} · robot_mode={self.live.get("mode")} blocked={self.live.get("blocked")} estop={self.live.get("emergency")}')
                self.command_status.config(text=f'JOG {direction} · 2010 지속 TCP 전송 중',fg=ORANGE)
            else:
                self.sim_required()
                if self.task_running:
                    raise ValueError('Task 실행을 취소한 뒤 수동 조작하세요.')
                self.held=(direction,motion)
                try:self.sim.drive(*motion)
                except ValueError:
                    self.release_drive()
                    if not self.sim._manual_stop_reason:raise
                    text='수동 정지 · '+self.sim._manual_stop_reason
                    if self.command_status.cget('text')!=text:self.log('JOG',text)
                    self.command_status.config(text=text,fg=ORANGE)
        self.guarded(press)

    def release_drive(self,event=None,force=False):
        previous=self.held
        self.held = None
        if self.real:
            with self._jog_lock:
                self._jog_desired=None
            if self._jog_wakeup:self._jog_wakeup.set()
            if previous is not None and self.connected and self.control_enabled:
                self.log('JOG','STOP 요청 · 2000 전송 예정')
                self.command_status.config(text='JOG STOP · 2000 전송 대기',fg=ORANGE)
            return
        self.sim.v = self.sim.w = self.sim.lease = 0

    def focus_guard(self):
        if self.focus_displayof() is None:
            if hasattr(self,'studio_runner') and self.studio_runner.active:
                actions=self.studio_runner.actions
                if actions and actions[self.studio_runner.index]['type'] in ('Translation','Rotation'):
                    self.studio_runner.pause(time.monotonic())
            self.release_drive()
            self.manual.set(False)

    def _transform(self):
        values = [(n['x'],n['y']) for n in self.map.nodes.values()]
        values += [(w[i],w[i+1]) for w in self.map.walls for i in (0,2)]
        cloud=getattr(self.map,'cloud',[])
        values += cloud[::max(1,len(cloud)//2000)]
        if self.real and self.connected and all(type(self.live.get(k)) in (float,int) for k in ('x','y')):
            values.append((self.live['x'],self.live['y']))
        if not values:values=[(-5,-5),(5,5)]
        xs,ys = zip(*values)
        lo_x,hi_x,lo_y,hi_y = min(xs)-1,max(xs)+1,min(ys)-1,max(ys)+1
        width,height = max(1,self.canvas.winfo_width()),max(1,self.canvas.winfo_height())
        scale = min(width/(hi_x-lo_x),height/(hi_y-lo_y))*.90*self.zoom
        return scale, width/2-(lo_x+hi_x)/2*scale+self.pan[0],height/2+(lo_y+hi_y)/2*scale+self.pan[1]

    def xy(self,x,y):
        if self.canvas is self.operation_canvas and hasattr(self,'world3d') and self.view_mode.get()=='3D':return self.world3d.map_xy(x,y)
        s,ox,oy = getattr(self,'_view_transform',None) or self._transform()
        return ox+x*s,oy-y*s

    def world(self,x,y):
        if self.canvas is self.operation_canvas and hasattr(self,'world3d') and self.view_mode.get()=='3D':return self.world3d.map_world(x,y)
        s,ox,oy = getattr(self,'_view_transform',None) or self._transform()
        return (x-ox)/s,(oy-y)/s

    def change_zoom(self,factor):
        if self.canvas is self.operation_canvas and self.view_mode.get()=="3D":
            self.world3d.zoom(1/factor);return
        self.zoom = min(6,max(.3,self.zoom*factor)); self.draw_map()

    def fit_map(self):
        if self.canvas is self.operation_canvas and self.view_mode.get()=="3D":
            self.world3d.fit();return
        self.zoom,self.pan = 1,[0,0]; self.draw_map()

    def pan_start(self,e):
        self.drag = (e.x,e.y,self.pan[:])

    def pan_move(self,e):
        if self.drag:
            x,y,p = self.drag
            self.pan = [p[0]+e.x-x,p[1]+e.y-y]
            self.draw_map()

    def _select_node(self,key):
        if key not in self.map.nodes:
            return False
        self.selected=key
        self.editor_selection_kind='node'
        if hasattr(self,'editor_lists') and self.tabs.select()==str(self.nodes_page):self.editor_lists.select(0)
        self.target.set(key)
        n=self.map.nodes[key]
        if hasattr(self,'selected_node_label'):
            source='ROBOT' if key in self.robot_stations else 'LOCAL/DRAFT'
            r=self._finite_value(n.get('r',n.get('angle')))
            angle_text=f'  ·  θ {math.degrees(r):.1f}°' if r is not None else ''
            self.selected_node_label.config(text=f'{key}  ·  X {n["x"]:.3f}  Y {n["y"]:.3f}{angle_text}  ·  {source}')
            if hasattr(self,'selected_node_options'):
                self.selected_node_options.config(text=self._station_option_summary(n) or '추가 목적지 옵션 없음')
        if hasattr(self,'node_prop_text'):
            r=self._finite_value(n.get('r',n.get('angle')))
            deg=f'{math.degrees(r):.1f}°' if r is not None else '-'
            self.node_prop_text.config(text=f"ID: {key}\nType: {n.get('type',n.get('kind','-'))}\nX/Y: {n['x']:.3f}, {n['y']:.3f} m\nHeading: {deg}\nSpin: {n.get('spin','-')}\nDesc: {n.get('desc','')}")
        if hasattr(self,'node_tree') and self.node_tree.exists(key):
            if key not in self.node_tree.selection():
                self.node_tree.selection_set(key)
            self.node_tree.see(key)
        return True

    def show_selected_station_options(self):
        key=self.selected if self.selected in self.map.nodes else self.target.get()
        if key not in self.map.nodes:
            messagebox.showinfo('Station 옵션','선택된 노드가 없습니다.',parent=self); return
        node=self.map.nodes[key]
        win=tk.Toplevel(self); win.title(f'{key} · RoboShop Station 전체 옵션'); self._fit_dialog(win,720,620,560,460)
        text=tk.Text(win,bg='#ffffff',fg='#263445',insertbackground=INK,font=('Consolas',10),wrap='none')
        text.pack(fill='both',expand=True,padx=8,pady=8)
        display={k:v for k,v in node.items() if k not in ('smap_raw','station_raw')}
        display['3051_payload']=self._station_nav_payload(node)
        if isinstance(node.get('station_raw'),dict): display['1301_raw']=node['station_raw']
        if isinstance(node.get('smap_raw'),dict): display['smap_raw']=node['smap_raw']
        text.insert('1.0',json.dumps(display,ensure_ascii=False,indent=2,default=str)); text.config(state='disabled')

    def target_changed(self,event=None):
        if self._select_node(self.target.get()):
            self.draw_map()

    def node_tree_select(self,event=None):
        keys=self.node_tree.selection()
        if len(keys)==1 and self._select_node(keys[0]):
            self.draw_map()

    def navigate_selected_from_tree(self):
        keys=self.node_tree.selection()
        if len(keys)!=1:return
        key=keys[0]
        self._select_node(key)
        if self.real and key not in self.robot_stations:
            messagebox.showwarning('목적지 이동',f'{key}은 로봇에서 조회된 실제 Station이 아닙니다.')
            return
        if messagebox.askyesno('목적지 이동',f'{key} 노드로 이동할까요?',parent=self):
            self.navigate()

    def map_click(self,e):
        (self.world3d if self.canvas is self.operation_canvas and self.view_mode.get()=='3D' else self.canvas).focus_set()
        x,y = self.world(e.x,e.y)
        if self.canvas is self.operation_canvas and not self.reloc_mode:
            if self.map.nodes:
                key=min(self.map.nodes,key=lambda k:math.hypot(e.x-self.xy(self.map.nodes[k]['x'],self.map.nodes[k]['y'])[0],e.y-self.xy(self.map.nodes[k]['x'],self.map.nodes[k]['y'])[1]))
                px,py=self.xy(self.map.nodes[key]['x'],self.map.nodes[key]['y'])
                if math.hypot(e.x-px,e.y-py)<=22:self._select_node(key)
            self.draw_map()
            return
        if hasattr(self,'studio_runner') and not self.reloc_mode and self._studio_map_click(e):return
        if self.edit_mode.get()=='곡선 편집' and not self.reloc_mode:
            self.guarded(lambda:self._curve_click(e))
            return

        # Relocation is a dedicated map interaction mode and takes precedence over
        # station/path editing. Manual: click+drag gives x/y/angle. Auto: click only
        # gives x/y and intentionally omits angle so RBK searches the full 2π range.
        if self.reloc_mode=='auto':
            self.reloc_candidate=(x,y,None)
            self.draw_map()
            self.after_idle(lambda xx=x,yy=y:self._send_reloc(xx,yy,None))
            self.reloc_mode=None
            return
        if self.reloc_mode=='manual':
            self.reloc_drag_start=(x,y)
            self.reloc_candidate=(x,y,0.0)
            self.draw_map(); return

        # Selection priority is always higher than map-edit drawing tools.
        # Even while Advanced Area mode is active, clicking an existing station
        # must select the station instead of starting an area rectangle.
        for key,n in self.map.nodes.items():
            heading=self._finite_value(n.get('r',n.get('angle')))
            if heading is None: continue
            cx,cy=self.xy(n['x'],n['y']); hx=cx+28*math.cos(heading); hy=cy-28*math.sin(heading)
            if math.hypot(hx-e.x,hy-e.y)<=10:
                self.heading_drag_key=key; self._select_node(key); self.draw_map(); return

        nearest=None
        if self.map.nodes:
            nearest=self.map.nearest(x,y)
            n=self.map.nodes[nearest]
            nx,ny=self.xy(n['x'],n['y'])
            if math.hypot(nx-e.x,ny-e.y)<=22:
                self._select_node(nearest)
                if self.edit_mode.get()=='경로 연결':
                    if self.pending_link and self.pending_link != nearest:
                        first=self.pending_link; self.pending_link=None
                        self.guarded(lambda:self.link(first,nearest))
                    else:
                        self.pending_link=nearest
                self.draw_map(); return

        # Point creation is explicit and only happens after existing-node hit testing.
        if self.edit_mode.get() == 'Point 추가':
            self.create_point_at(x,y); return

        # Advanced Area creation starts only on empty map space.
        if self.edit_mode.get() == 'Advanced Area':
            self.area_drag_start=(x,y); self.area_drag_current=(x,y); self.draw_map(); return

        # Pointer can select an Advanced Area directly on the map.
        if self.edit_mode.get()=='선택':
            for area in reversed(getattr(self.map,'area_records',[])):
                if self._point_in_polygon(x,y,area.get('points',[])):
                    self.selected_area=area.get('id'); self._select_area_tree(self.selected_area); self.draw_map(); return
            candidates=[]
            for index,record in enumerate(getattr(self.map,'path_records',[])):
                geometry=path_record_geometry(record.get('raw',{}))
                screen=[self.xy(*p) for p in geometry]
                if len(screen)>1:
                    candidates.append((min(point_segment_distance(e.x,e.y,a,b) for a,b in zip(screen,screen[1:])),index))
            if candidates:
                distance,index=min(candidates)
                if distance<=10 and self.path_tree.exists(f'path_{index}'):
                    self.path_tree.selection_set(f'path_{index}')
                    self.path_tree.see(f'path_{index}')
                    self._path_tree_select()
                    return

        self.draw_map()

    def map_drag(self,e):
        if hasattr(self,'operation_canvas') and self.canvas is self.operation_canvas and not self.reloc_mode:return
        if hasattr(self,'studio_runner') and self._studio_map_drag(e):return
        if self.curve_drag_index is not None:
            def move_control():
                self.editable()
                rec=self.curve_edit_record
                if not any(r is rec for r in getattr(self.map,'path_records',[])):return
                controls=list(rec['controls'])
                controls[self.curve_drag_index]=self.world(e.x,e.y)
                self._set_curve_controls(rec,controls)
                self.draw_map()
            self.guarded(move_control)
            return
        if self.reloc_mode=='manual' and self.reloc_drag_start is not None:
            x0,y0=self.reloc_drag_start; x1,y1=self.world(e.x,e.y)
            ang=math.atan2(y1-y0,x1-x0)
            self.reloc_candidate=(x0,y0,ang); self.draw_map(); return
        if self.area_drag_start is not None and self.edit_mode.get()=='Advanced Area':
            self.area_drag_current=self.world(e.x,e.y); self.draw_map(); return
        key=self.heading_drag_key
        if not key or key not in self.map.nodes:return
        if hasattr(self,'studio_runner') and not self._studio_can_edit():return
        n=self.map.nodes[key]
        wx,wy=self.world(e.x,e.y)
        n['r']=math.atan2(wy-n['y'],wx-n['x'])
        n['angle']=n['r']
        n['spin']=True
        self._select_node(key); self.draw_map()

    def map_release(self,e=None):
        if hasattr(self,'studio_runner') and self._studio_map_release():return
        if self.curve_drag_index is not None:
            self.curve_drag_index=None
            self.map.curves=[path_record_geometry(r.get('raw',{})) for r in self.map.path_records]
            self.log('EDIT','Path 곡선 조절점 변경')
            self.draw_map()
            return
        if self.reloc_mode=='manual' and self.reloc_drag_start is not None:
            x0,y0=self.reloc_drag_start
            cand=self.reloc_candidate or (x0,y0,0.0)
            self.reloc_drag_start=None; self.reloc_mode=None
            self.draw_map()
            self.after_idle(lambda c=cand:self._send_reloc(c[0],c[1],c[2]))
            return
        if self.area_drag_start is not None and self.edit_mode.get()=='Advanced Area':
            a=self.area_drag_start; b=self.area_drag_current or a
            self.area_drag_start=None; self.area_drag_current=None
            if abs(a[0]-b[0])>0.05 and abs(a[1]-b[1])>0.05:
                self.guarded(lambda:self.create_area_rect(a,b))
            else:
                self.draw_map()
            return
        if self.heading_drag_key:
            key=self.heading_drag_key; self.heading_drag_key=None
            n=self.map.nodes.get(key,{})
            r=self._finite_value(n.get('r'))
            if r is not None:
                self.map_dirty=True; self.log('EDIT',f'{key} 도착 방향 변경 → {math.degrees(r):.1f}°')

    def map_double_click(self,e):
        if self.canvas is self.editor_canvas:
            if self.edit_mode.get()!='선택':return
            self.map_click(e)
            wx,wy=self.world(e.x,e.y)
            kind=self.editor_selection_kind
            if kind=='node' and self.selected in self.map.nodes:
                n=self.map.nodes[self.selected];px,py=self.xy(n['x'],n['y'])
                if math.hypot(e.x-px,e.y-py)<=22:self.edit_node()
            elif kind=='area':
                rec=self._find_area()
                if rec and self._point_in_polygon(wx,wy,rec.get('points',[])):self.edit_selected_area()
            elif kind=='path':
                rec=self._selected_path_record()
                screen=[self.xy(*p) for p in path_record_geometry(rec.get('raw',{}))] if rec else []
                if len(screen)>1 and min(point_segment_distance(e.x,e.y,a,b) for a,b in zip(screen,screen[1:]))<=10:self.edit_selected_path()
            return
        if self.reloc_mode:return
        wx,wy=self.world(e.x,e.y)
        self.map_click(e)
        target=self.selected
        if not target or target not in self.map.nodes:
            return
        n=self.map.nodes[target];px,py=self.xy(n['x'],n['y'])
        if math.hypot(e.x-px,e.y-py)>22:return
        if self.real and target not in self.robot_stations:
            messagebox.showwarning('목적지 이동',f'{target}은 로봇에서 조회된 실제 Station이 아닙니다.\n로봇 맵에 저장된 노드만 주행할 수 있습니다.')
            return
        if messagebox.askyesno('목적지 이동',f'{target} 노드로 이동할까요?',parent=self):
            self.target.set(target)
            self.navigate()

    def _lidar_mount_file(self):
        return USER_DIR/'lidar_mounts.json'

    def _load_lidar_mounts(self):
        try:
            data=json.loads(self._lidar_mount_file().read_text(encoding='utf-8'))
            host=data.get(self.host.get().strip(),{}) if isinstance(data,dict) else {}
            mounts=host.get('mounts') if isinstance(host,dict) else None
            if isinstance(mounts,list):
                out=[]
                for m in mounts:
                    if isinstance(m,(list,tuple)) and len(m)>=3:
                        out.append((float(m[0]),float(m[1]),float(m[2])))
                    else: out.append(None)
                self.lidar_mount_overrides=out
                if any(m is not None for m in out):
                    self.log('LASER',f'저장된 센서 장착 보정값 로드 · {out}')
        except Exception:
            pass

    def _save_lidar_mounts(self, mounts):
        USER_DIR.mkdir(parents=True,exist_ok=True)
        path=self._lidar_mount_file()
        try:
            data=json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
        except Exception:data={}
        data[self.host.get().strip()]={'mounts':[list(m) if m is not None else None for m in mounts],
                                      'updated':datetime.now().isoformat(timespec='seconds')}
        path.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')

    @staticmethod
    def _grid_scan_score(map_cells, pts, rx, ry, ra, mount, cell=0.12):
        dx,dy,da=mount
        ca,sa=math.cos(da),math.sin(da)
        cr,sr=math.cos(ra),math.sin(ra)
        hits=0; total=0
        for lx,ly in pts:
            bx=dx+ca*lx-sa*ly; by=dy+sa*lx+ca*ly
            wx=rx+cr*bx-sr*by; wy=ry+sr*bx+cr*by
            gx=int(round(wx/cell)); gy=int(round(wy/cell)); total+=1
            found=False
            for ox in (-1,0,1):
                for oy in (-1,0,1):
                    if (gx+ox,gy+oy) in map_cells:
                        found=True; break
                if found: break
            if found:hits+=1
        return hits/max(1,total)

    def _schedule_lidar_autocalibration(self, scans, pose):
        if self._lidar_calibration_running or not scans or pose is None:return
        if self._lidar_calibrated_for_map == (self.current_robot_map or self.map.name):return
        unknown=[i for i,sc in enumerate(scans) if sc.get('mount') is None and sc.get('local_points')]
        cloud=getattr(self.map,'cloud',[]) or []
        if not unknown or len(cloud)<300:return
        self._lidar_calibration_running=True
        map_name=self.current_robot_map or self.map.name
        cloud_sample=cloud[::max(1,len(cloud)//12000)]
        scan_copies=[list(sc.get('local_points') or []) for sc in scans]
        rx,ry,ra=map(float,pose)
        def work():
            try:
                cell=.12
                cells={(int(round(x/cell)),int(round(y/cell))) for x,y in cloud_sample}
                results=list(self.lidar_mount_overrides)
                while len(results)<len(scans):results.append(None)
                reports=[]
                for i in unknown:
                    raw=scan_copies[i]
                    sample=raw[::max(1,len(raw)//140)]
                    best=(-1.0,(0.0,0.0,0.0))
                    # Coarse search: chassis-sized translation and full yaw range.
                    for ix in range(-8,9):
                        dx=ix*.08
                        for iy in range(-8,9):
                            dy=iy*.08
                            for ia in range(24):
                                da=-math.pi+ia*(2*math.pi/24)
                                sc=self._grid_scan_score(cells,sample,rx,ry,ra,(dx,dy,da),cell)
                                if sc>best[0]:best=(sc,(dx,dy,da))
                    bscore,(bdx,bdy,bda)=best
                    # Fine search around the coarse optimum.
                    fine=best
                    for ix in range(-4,5):
                        dx=bdx+ix*.02
                        for iy in range(-4,5):
                            dy=bdy+iy*.02
                            for ia in range(-4,5):
                                da=bda+math.radians(ia*2.0)
                                sc=self._grid_scan_score(cells,sample,rx,ry,ra,(dx,dy,da),cell)
                                if sc>fine[0]:fine=(sc,(dx,dy,da))
                    score,m=fine
                    # Only accept a plausible match; otherwise keep unknown rather than
                    # force a bad transform.
                    if score>=0.18:
                        results[i]=m
                    reports.append((i,round(score,3),tuple(round(v,4) for v in m)))
                self.events.put(('lidar_calibration_done',self.generation,(map_name,results,reports)))
            except Exception as e:
                self.events.put(('lidar_calibration_error',self.generation,str(e)))
        threading.Thread(target=work,daemon=True,name='seer-lidar-autocal').start()


    def _mapping_accumulate_live_scan(self):
        if not (self.real and self.connected and self.real_mapping_active):
            return
        pts=list(self.real_laser_points or [])
        if pts:
            cell=0.05
            cloud=self.real_mapping_cloud
            seen=self._real_mapping_cells
            step=max(1,len(pts)//1600)
            for x,y in pts[::step]:
                gx=int(round(x/cell)); gy=int(round(y/cell))
                key=(gx,gy)
                if key in seen:
                    continue
                seen.add(key)
                cloud.append((x,y))
        st=self.current_state()
        rx=self._finite_value(st.get('x')); ry=self._finite_value(st.get('y'))
        if rx is not None and ry is not None:
            trace=self.real_mapping_trace
            if not trace or (trace[-1][0]-rx)**2 + (trace[-1][1]-ry)**2 > 0.04:
                trace.append((rx,ry))
                if len(trace)>4000:
                    del trace[:1000]

    def _draw_real_mapping_canvas(self,c,w,h):
        # Blank work area during live SLAM creation, RoboShop-like progressive build.
        c.create_rectangle(0,0,w,h,fill='#f5f7fa',outline='')
        if self.layers['그리드'].get():
            step=self._transform()[0]
            if step>12:
                ox,oy=self.xy(0,0)
                for i in range(int(w/step)+2):
                    x=ox%step+i*step; c.create_line(x,0,x,h,fill='#e6ebef')
                for i in range(int(h/step)+2):
                    y=oy%step+i*step; c.create_line(0,y,w,y,fill='#e6ebef')
        self._mapping_accumulate_live_scan()
        trace=self.real_mapping_trace
        if len(trace)>1:
            c.create_line(*[v for p in trace for v in self.xy(*p)],fill='#98a8ba',width=2)
        cloud=self.real_mapping_cloud
        if cloud:
            step=max(1,len(cloud)//5000)
            for x,y in cloud[::step]:
                px,py=self.xy(x,y)
                c.create_rectangle(px,py,px+1,py+1,fill='#4d5966',outline='')
        scans=getattr(self,'real_laser_scans',[]) or []
        if scans:
            # Draw only VALID reflected beams. Their line length is the actual range:
            # nearby obstacles produce short rays, distant walls long rays. Max-range /
            # no-return samples were already filtered by extract_laser_scans().
            for scan in scans:
                pts=list(scan.get('points') or []); origin=scan.get('origin')
                if not pts: continue
                ray_step=max(1,len(pts)//260); point_step=max(1,len(pts)//700)
                if isinstance(origin,(list,tuple)) and len(origin)>=2:
                    ox,oy=self.xy(float(origin[0]),float(origin[1]))
                    for x,y in pts[::ray_step]:
                        px,py=self.xy(x,y)
                        c.create_line(ox,oy,px,py,fill='#cfeaf2',width=1)
                for x,y in pts[::point_step]:
                    px,py=self.xy(x,y)
                    c.create_rectangle(px-1.4,py-1.4,px+1.4,py+1.4,fill='#45c96a',outline='')
        st=self.current_state()
        rx=self._finite_value(st.get('x')); ry=self._finite_value(st.get('y')); ra=self._finite_value(st.get('theta')) or 0.0
        if rx is not None and ry is not None:
            x,y=self.xy(rx,ry)
            c.create_oval(x-10,y-10,x+10,y+10,fill='#2764e7',outline='#153b8f',width=2)
            hx=x+26*math.cos(ra); hy=y-26*math.sin(ra)
            c.create_line(x,y,hx,hy,fill='#153b8f',width=3,arrow='last')
        c.create_text(14,14,anchor='nw',text='실시간 SLAM · 장애물 LiDAR Hit Point만 누적 표시',fill='#4a5662',font=(self.font,10,'bold'))
        c.create_text(14,34,anchor='nw',text=f'누적 장애물점 {len(self.real_mapping_cloud)} · 현재 Hit {len(self.real_laser_points)} pts',fill='#6a7581',font=(self.font,9))

    def _map_notice(self,text,tag,priority=0,bold=False):
        self._map_notices.append((priority,text,tag,bold))

    def _draw_map_notices(self,c):
        y=10
        for _,text,tag,bold in sorted(self._map_notices,key=lambda item:item[0]):
            item=c.create_text(12,y,anchor='nw',text=text,fill=INK,font=(self.font,9,'bold' if bold else 'normal'),
                               width=max(100,c.winfo_width()-24),tags=('map_notice',tag))
            box=c.bbox(item)
            if box:
                backing=c.create_rectangle(box[0]-4,box[1]-2,box[2]+4,box[3]+2,fill=WHITE,outline='',tags='map_notice_background')
                c.tag_lower(backing,item)
                y=box[3]+7
        self._map_notice_bottom=y

    def _draw_map_label(self,c,x,y,text,color,tag,offsets):
        """Place readable labels without covering another label or node marker."""
        item=c.create_text(x,y,text=text,fill=color,font=(self.font,9,'bold'),width=max(80,min(240,c.winfo_width()-16)),tags=('map_label',tag))
        for dx,dy in offsets:
            c.coords(item,x+dx,y+dy)
            box=c.bbox(item)
            if not box:continue
            shift_x=max(0,5-box[0])-max(0,box[2]-(c.winfo_width()-5))
            shift_y=max(0,self._map_notice_bottom+3-box[1])-max(0,box[3]-(c.winfo_height()-5))
            if shift_x or shift_y:
                c.move(item,shift_x,shift_y);box=c.bbox(item)
            padded=(box[0]-3,box[1]-2,box[2]+3,box[3]+2)
            if any(padded[0]<other[2] and padded[2]>other[0] and padded[1]<other[3] and padded[3]>other[1]
                   for other in self._map_label_boxes+self._map_marker_boxes):continue
            self._map_label_boxes.append(padded)
            backing=c.create_rectangle(*padded,fill=WHITE,outline='',tags='map_label_background')
            c.tag_lower(backing,item)
            return item
        c.delete(item)
        return None

    def draw_map(self):
        if not hasattr(self,'canvas'):
            return
        if hasattr(self,'operation_sensor_label'):
            if self.real:
                count=len(self.real_laser_points);age=time.monotonic()-self.last_laser_rx
                sensor=f'LiDAR 수신 {count}점 · {age:.1f}s 전' if count and age<=2 else ('LiDAR 수신 지연' if count else 'LiDAR 수신 대기')
                blocked=self.live.get('blocked');obstacle='정지 감지' if blocked is True else ('정상' if blocked is False else '상태 미수신')
            else:
                sensor=f'LiDAR SIM · {len(self.sim.scan()) if self.connected and self.sim_powered else 0}점'
                obstacle=self.sim.block_reason or getattr(self.sim,'detected_obstacle','') or ('정지 감지' if self.sim.state.blocked else '정상')
            if not self.connected:sensor='LiDAR 미연결';obstacle='연결 대기'
            self.operation_sensor_label.configure(text=f'{sensor}  |  장애물: {obstacle}'+('  |  LiDAR 표시 OFF' if not self.layers['LiDAR'].get() else ''))
        if self.canvas is self.operation_canvas and hasattr(self,"world3d") and self.view_mode.get()=="3D":
            self._spatial_render();return
        c = self.canvas
        self._view_transform=self._transform()
        c.delete('all')
        self._map_label_boxes=[]
        self._map_marker_boxes=[]
        self._map_notices=[]
        self._map_notice_bottom=10
        node_labels=[]
        robot_label=None
        w,h = c.winfo_width(),c.winfo_height()
        if self.real and self.connected and self.real_mapping_active:
            self._draw_real_mapping_canvas(c,w,h)
            return
        if self.layers['그리드'].get():
            step = self._transform()[0]
            if step>12:
                ox,oy = self.xy(0,0)
                for i in range(int(w/step)+2):
                    x = ox%step+i*step; c.create_line(x,0,x,h,fill=GRID)
                for i in range(int(h/step)+2):
                    y = oy%step+i*step; c.create_line(0,y,w,y,fill=GRID)
        if self.layers['벽'].get():
            cloud=getattr(self.map,'cloud',[])
            # Loaded SMAP point cloud: denser, darker and slightly larger so wall
            # outlines stay readable on laptop / mini-PC displays.
            sample_step=max(1,len(cloud)//5200)
            dot=max(1.4,min(2.6,1.7*self.zoom))
            for px,py in cloud[::sample_step]:
                x,y=self.xy(px,py)
                c.create_oval(x-dot,y-dot,x+dot,y+dot,fill='#3f4750',outline='')
            for x1,y1,x2,y2 in self.map.walls:
                c.create_line(*self.xy(x1,y1),*self.xy(x2,y2),fill='#3f4750',width=5)
        if self.layers.get('영역') is not None and self.layers['영역'].get():
            for area in getattr(self.map,'area_records',[]):
                pts=area.get('points',[])
                if len(pts)<3: continue
                coords=[v for pt in pts for v in self.xy(*pt)]
                props=area.get('properties') or {}
                selected=(area.get('id')==self.selected_area)
                forbidden=self._bool_value(props.get('forbidden')) is True
                outline='#ff5c70' if forbidden else '#e7a13b' if selected else '#4ba7b7'
                fill='#5a2730' if forbidden else '#51452a' if selected else '#28434a'
                c.create_polygon(*coords,fill=fill,outline=outline,width=3 if selected else 2,stipple='gray25')
                cx=sum(p[0] for p in pts)/len(pts); cy=sum(p[1] for p in pts)/len(pts)
                sx,sy=self.xy(cx,cy)
                speed=props.get('maxspeed','-'); dec=props.get('obsDecDist','-'); stop=props.get('obsStopDist','-')
                c.create_text(sx,sy,text=f"{area.get('id')}\nV≤{speed}  DEC {dec}  STOP {stop}",fill='#d9eef1',font=(self.font,8,'bold'),justify='center')
            if self.area_drag_start is not None and self.area_drag_current is not None:
                x1,y1=self.area_drag_start; x2,y2=self.area_drag_current
                draft=[(x1,y1),(x2,y1),(x2,y2),(x1,y2)]
                coords=[v for pt in draft for v in self.xy(*pt)]
                c.create_polygon(*coords,fill='#5d4a25',outline='#f1b54b',width=2,stipple='gray25',dash=(5,3))
        if self.layers['경로'].get():
            # Render RoboShop topology: a reverse record means bidirectional; a single
            # AdvancedCurve means one-way. Draw one geometry per physical pair with arrows.
            records=getattr(self.map,'path_records',[])
            handled=set()
            for rec in records:
                a,b=rec.get('a'),rec.get('b')
                if not a or not b: continue
                pair=frozenset((a,b))
                if pair in handled: continue
                rev=next((r for r in records if r is not rec and r.get('a')==b and r.get('b')==a),None)
                raw=rec.get('raw',{}) if isinstance(rec.get('raw'),dict) else {}
                geom=path_record_geometry(raw)
                coords=[v for p in geom for v in self.xy(*p)]
                if len(coords)>=4:
                    arrow='both' if rev else 'last'
                    c.create_line(*coords,fill='#6f879d',width=2,arrow=arrow,arrowshape=(9,11,4),smooth=False)
                handled.add(pair)
            if self.real and self.connected:
                route=points(self.live.get('path',[]))
                if not route:
                    route=[(self.map.nodes[k]['x'],self.map.nodes[k]['y']) for k in self.live.get('unfinished_path',[]) if k in self.map.nodes]
                if len(route)>1:c.create_line(*[v for p in route for v in self.xy(*p)],fill=GREEN,width=4)
            # `edges` is the route graph.  Real SMAP paths are already rendered above
            # from AdvancedCurve geometry; drawing every edge again would put a false
            # straight line on top of a RoboShop Bezier path.
            curved_pairs={frozenset((r.get('a'),r.get('b'))) for r in getattr(self.map,'path_records',[]) if r.get('a') and r.get('b')}
            for a,b in self.map.edges:
                if frozenset((a,b)) in curved_pairs:continue
                n,m = self.map.nodes[a],self.map.nodes[b]
                c.create_line(*self.xy(n['x'],n['y']),*self.xy(m['x'],m['y']),fill='#9aaab8',width=2,dash=(5,5))
            if not self.real and self.connected:
                route_points=self.sim.navigation_points()
                if len(route_points)>1:
                    c.create_line(*[v for p in route_points for v in self.xy(*p)],fill='#51d8b6',width=4,arrow='last')
        rec=self.curve_edit_record
        if self.canvas is self.editor_canvas and self.edit_mode.get()=='곡선 편집' and any(r is rec for r in getattr(self.map,'path_records',[])):
            coords=[v for p in path_record_geometry(rec['raw']) for v in self.xy(*p)]
            c.create_line(*coords,fill='#ffb347',width=4)
            for i,key in enumerate(('a','b')):
                n=self.map.nodes[rec[key]]; px,py=self.xy(*rec['controls'][i])
                c.create_line(*self.xy(n['x'],n['y']),px,py,fill='#ffb347',dash=(4,3))
                c.create_rectangle(px-7,py-7,px+7,py+7,fill='#ffb347',outline='#ffffff',width=2)
                c.create_text(px+12,py-12,text=f'C{i+1}',fill=INK,font=(self.font,10,'bold'))
            self._map_notice(f"{rec['a']} → {rec['b']} · 주황색 □ 드래그: 곡선 변경",'curve_edit_notice',10,True)
        if self.layers['LiDAR'].get() and not self.real and self.connected:
            lidar_points = self.sim.cloud if self.sim.mapping else self.scan_points
            for x,y in lidar_points[::max(1,len(lidar_points)//2200)]:
                px,py = self.xy(x,y)
                c.create_oval(px-1.5,py-1.5,px+1.5,py+1.5,fill='#d33757',outline='')
        if self.real and self.connected and self.layers['LiDAR'].get():
            # Normal map view uses the exact same obstacle-hit policy as SLAM:
            # no sensor rays, no max-range/no-return beams, only real returns.
            # Apply the saved LiDAR-to-map alignment only for display so the hit
            # cloud overlays the 4011 map contour without changing robot control.
            live_laser = self.real_laser_points or points(self.live.get('laser',[]))
            scans=getattr(self,'real_laser_scans',[]) or []
            if scans:
                for scan in scans:
                    pts=list(scan.get('points') or []); origin=scan.get('origin')
                    if not pts: continue
                    official_world=bool(scan.get('official_world'))
                    ray_step=max(1,len(pts)//260); point_step=max(1,len(pts)//900)
                    # Do not draw rays. RoboShop-style operator view shows only
                    # physical return/hit points; rays make the scan look like a starburst.
                    for x,y in pts[::point_step]:
                        if not official_world:
                            x,y=self._lidar_map_xy(x,y)
                        px,py=self.xy(x,y)
                        c.create_rectangle(px-1.4,py-1.4,px+1.4,py+1.4,fill='#45c96a',outline='')
            elif live_laser:
                step=max(1,len(live_laser)//1200)
                official=('laser_beams(WORLD)' in self.lidar_source)
                for x,y in live_laser[::step]:
                    if not official:
                        x,y=self._lidar_map_xy(x,y)
                    px,py=self.xy(x,y)
                    c.create_rectangle(px-1.4,py-1.4,px+1.4,py+1.4,fill='#45c96a',outline='')
        # RoboShop-style obstacle layer: every VALID return from the merged dual-LiDAR
        # is a currently sensed obstacle surface.  Static map-matching returns (walls,
        # columns, equipment) MUST remain visible; map-novel returns are highlighted as
        # dynamic/unmapped.  `blocked` is only a controller stop state, not visibility.
        if self.real and self.connected and self.layers.get('장애물') is not None and self.layers['장애물'].get():
            scan_pts=list(self.real_laser_points or [])  # already dual-LiDAR merged in WORLD/MAP frame
            map_pts=list(getattr(self.map,'cloud',[]) or [])
            st=self.current_state(); rx=self._finite_value(st.get('x')); ry=self._finite_value(st.get('y'))
            static_hits=[]; dynamic=[]
            if scan_pts:
                # Classify only for display emphasis.  Never discard static hits: these are
                # exactly the returns used to perceive/map walls and fixed equipment.
                cell=.11
                mcells={(int(round(mx/cell)),int(round(my/cell))) for mx,my in map_pts} if map_pts else set()
                max_r2=12.0*12.0
                for qx,qy in scan_pts:
                    if rx is not None and ry is not None and (qx-rx)**2+(qy-ry)**2>max_r2:
                        continue
                    if not mcells:
                        static_hits.append((qx,qy)); continue
                    gx=int(round(qx/cell)); gy=int(round(qy/cell)); near=False
                    for ox in (-2,-1,0,1,2):
                        for oy in (-2,-1,0,1,2):
                            if (gx+ox,gy+oy) in mcells:
                                near=True; break
                        if near: break
                    (static_hits if near else dynamic).append((qx,qy))
            # Remove isolated reflections only from the dynamic/unmapped emphasis layer.
            # Static returns are intentionally never density-filtered away.
            if dynamic:
                ccell=.18; bins={}
                for q in dynamic:
                    k=(int(round(q[0]/ccell)),int(round(q[1]/ccell)))
                    bins.setdefault(k,[]).append(q)
                keep=[]
                for k,vals in bins.items():
                    count=sum(len(bins.get((k[0]+ox,k[1]+oy),())) for ox in (-1,0,1) for oy in (-1,0,1))
                    if count>=4: keep.extend(vals)
                dynamic=keep
            # Fixed/static obstacle surfaces: amber, fine points.
            step=max(1,len(static_hits)//1000) if static_hits else 1
            for qx,qy in static_hits[::step]:
                qpx,qpy=self.xy(qx,qy)
                c.create_rectangle(qpx-1.8,qpy-1.8,qpx+1.8,qpy+1.8,fill='#f0a43a',outline='')
            # Dynamic / not-yet-in-map obstacle surfaces: red and slightly larger.
            step=max(1,len(dynamic)//600) if dynamic else 1
            for qx,qy in dynamic[::step]:
                qpx,qpy=self.xy(qx,qy)
                c.create_oval(qpx-2.7,qpy-2.7,qpx+2.7,qpy+2.7,fill='#ff4f5e',outline='')
            # Controller-confirmed blocking point remains a separate stop marker.
            blocked=self._bool_value(self.live.get('blocked')) is True
            bx=self._finite_value(self.live.get('block_x')); by=self._finite_value(self.live.get('block_y'))
            if blocked and bx is not None and by is not None:
                opx,opy=self.xy(bx,by)
                c.create_oval(opx-11,opy-11,opx+11,opy+11,outline='#d71920',width=3)
                c.create_line(opx-15,opy,opx+15,opy,fill='#d71920',width=2)
                c.create_line(opx,opy-15,opx,opy+15,fill='#d71920',width=2)
                reasons={0:'ULTRASONIC',1:'LASER',2:'FALLING',3:'COLLISION',4:'INFRARED',5:'LOCK',6:'API OBSTACLE',7:'VIRTUAL POINT'}
                reason=self.live.get('block_reason')
                c.create_text(opx+16,opy-16,anchor='sw',text=f'BLOCK · {reasons.get(reason,reason)}',fill='#d71920',font=(self.font,9,'bold'))
        if self.reloc_candidate is not None:
            rx0,ry0,ra0=self.reloc_candidate
            px,py=self.xy(rx0,ry0)
            c.create_oval(px-12,py-12,px+12,py+12,outline='#d78313',width=3,dash=(4,2))
            if ra0 is not None:
                hx=px+42*math.cos(ra0); hy=py-42*math.sin(ra0)
                c.create_line(px,py,hx,hy,fill='#d78313',width=4,arrow='last')
            c.create_text(px+15,py-18,anchor='w',text='RELOC 후보',fill='#d78313',font=(self.font,9,'bold'))
        if self.layers['노드'].get():
            for key,n in self.map.nodes.items():
                x,y = self.xy(n['x'],n['y'])
                is_robot = key in self.robot_stations if self.real else True
                color = '#f5a45d' if key==self.selected else '#55c98f' if n.get('kind')=='dock' else '#e9a06f' if is_robot and self.real else '#7b8ee8'
                radius = 10 if key==self.selected else 8
                c.create_rectangle(x-radius,y-radius,x+radius,y+radius,fill=color,outline='#30363e',width=2)
                # Draw the RoboShop station heading. Canvas Y is inverted, hence -sin.
                heading=self._finite_value(n.get('r',n.get('angle')))
                if heading is not None:
                    hx=x+24*math.cos(heading); hy=y-24*math.sin(heading)
                    hcolor='#c93043' if self._bool_value(n.get('spin')) is True else '#596f86'
                    c.create_line(x,y,hx,hy,fill=hcolor,width=2)
                    # Direction handle similar to RoboShop's red arrival-orientation triangle.
                    a=heading; tip=(hx,hy); left=(hx-8*math.cos(a)-5*math.sin(a),hy+8*math.sin(a)-5*math.cos(a)); right=(hx-8*math.cos(a)+5*math.sin(a),hy+8*math.sin(a)+5*math.cos(a))
                    c.create_polygon(tip[0],tip[1],left[0],left[1],right[0],right[1],fill='#d83a48',outline='#8e1e28')
                self._map_marker_boxes.append((x-12,y-12,x+12,y+12))
                label_color=BLUE if key==self.selected else '#087750' if n.get('kind')=='dock' else '#465775'
                node_labels.append((key,x,y,label_color))
        if self.canvas is self.editor_canvas and getattr(self,'editor_selection_kind','node')=='path':
            record=self._selected_path_record()
            if record:
                geometry=path_record_geometry(record.get('raw',{}))
                if len(geometry)>1:
                    c.create_line(*[v for p in geometry for v in self.xy(*p)],fill=BLUE,width=5,arrow='last',tags='selected_editor_path')
        state = self.current_state()
        if self.connected and all(isinstance(state.get(k),(float,int)) and math.isfinite(state[k]) for k in ('x','y','theta')):
            sx,sy=float(state['x']),float(state['y']); a=float(state['theta'])
            if self.real:
                sx,sy=self._lidar_map_xy(sx,sy)
                a += self.lidar_map_alignment[2]
            x,y = self.xy(sx,sy)
            vertices = [(x+18*math.cos(a),y-18*math.sin(a)),
                        (x+13*math.cos(a+2.45),y-13*math.sin(a+2.45)),
                        (x+13*math.cos(a-2.45),y-13*math.sin(a-2.45))]
            c.create_oval(x-23,y-23,x+23,y+23,outline='#287be0',width=2)
            c.create_polygon(*sum(([vx,vy] for vx,vy in vertices),[]),fill='#4e9bff',outline=WHITE,width=2)
            location=self.location_display
            if location:
                caption=('현재 '+location['at']) if location.get('at') else ('직전 '+(location.get('last') or '—'))
                if location.get('next'):caption+=' → '+location['next']
                if not location.get('valid'):caption+=' · 갱신 대기'
                robot_label=(x,y,caption)
        self._map_notice(('REAL TELEMETRY / ROBOT MAP (4011)' if self.current_robot_map else 'REAL TELEMETRY / LOCAL MAP (좌표 일치 확인)') if self.real else 'SIMULATION  /  SYNTHETIC LiDAR','telemetry_notice')
        s,_,_ = self._transform()
        c.create_line(18,h-23,18+2*s,h-23,fill='#526a80',width=2)
        self._studio_draw(c)
        self._draw_map_notices(c)
        # Render labels after geometry and diagnostics, keeping IDs legible.
        for key,x,y,color in sorted(node_labels,key=lambda item:item[0]!=self.selected):
            self._draw_map_label(c,x,y,key,color,'node_label',[(0,-23),(0,25),(38,-23),(-38,-23),(38,25),(-38,25),(0,-45),(0,47)])
        if robot_label:
            x,y,caption=robot_label
            self._draw_map_label(c,x,y,caption,BLUE,'robot_location_caption',[(0,-48),(0,49),(75,-35),(-75,-35),(0,-72),(0,73)])
        c.create_text(18,h-38,anchor='w',text='2 m',fill='#526a80',font=(self.font,9))
        lidar_n=len(self.real_laser_points) if self.real else len(self.scan_points)
        lidar_age=(time.monotonic()-self.last_laser_rx) if self.real and self.last_laser_rx else None
        lidar_diag=(f' · {self.lidar_source} · RX {lidar_age:.1f}s' if self.real and lidar_age is not None else (' · LiDAR WAIT' if self.real else ''))
        obs_txt=''
        if self.real and self.live.get('blocked'):
            obs_txt=f' · BLOCK ({self.live.get("block_x","?")},{self.live.get("block_y","?")})'
        self._map_info_details=(f'{self.map.name}  |  {len(self.map.nodes)} nodes · {len(self.map.edges)} paths  |  '
                            f'{len(getattr(self.map,"area_records",[]))} areas · {self.zoom*100:.0f}% · LiDAR {lidar_n} pts{lidar_diag}{obs_txt} · Align {self.lidar_alignment_score:.2f} · Hit Point only  |  휠: 확대 · 우클릭 드래그: 이동 · 클릭: 노드 선택 · 더블클릭: 목적지 이동')
        self.map_info.config(text=f'{self.map.name[:28]} · 노드 {len(self.map.nodes)} / 경로 {len(self.map.edges)} · {self.zoom*100:.0f}% · LiDAR {lidar_n}점')

    def editable(self):
        if self.map_push_in_progress:
            raise ValueError('맵 업데이트가 진행 중입니다.')
        if self.task_running or self.held or self.sim.route or self.sim.mapping:
            raise ValueError('주행/수동조작/맵 생성을 종료한 뒤 편집하세요.')
        if self.real:
            if not hasattr(self.map,'smap_source'):
                raise ValueError('실기 맵 편집 전 Pull Map이 필요합니다.')
            task=str(self.live.get('task','')).upper()
            if task in ('RUNNING','WAITING','SUSPENDED'):
                raise ValueError('AMR Task가 동작 중입니다. 취소/완료 후 맵을 편집하세요.')
            try: speed=float(self.live.get('speed') or 0.0)
            except (TypeError,ValueError): speed=0.0
            if abs(speed) > 0.02:
                raise ValueError('AMR가 움직이는 동안 맵을 편집할 수 없습니다.')
        if hasattr(self,'studio_history') and getattr(self,'_studio_edit_pending',None) is None:
            self._studio_edit_pending=self.studio_history.capture(self.map)

    @staticmethod
    def _point_in_polygon(x,y,pts):
        if not isinstance(pts,list) or len(pts)<3:return False
        inside=False; j=len(pts)-1
        for i in range(len(pts)):
            xi,yi=pts[i]; xj,yj=pts[j]
            if ((yi>y)!=(yj>y)):
                xin=(xj-xi)*(y-yi)/(yj-yi if yj!=yi else 1e-12)+xi
                if x < xin: inside=not inside
            j=i
        return inside

    def _next_area_id(self):
        used={str(a.get('id')) for a in getattr(self.map,'area_records',[])}
        i=1
        while f'Area{i}' in used:i+=1
        return f'Area{i}'

    def create_area_rect(self,a,b):
        self.editable()
        x1,y1=a; x2,y2=b
        pts=[(min(x1,x2),min(y1,y2)),(max(x1,x2),min(y1,y2)),(max(x1,x2),max(y1,y2)),(min(x1,x2),max(y1,y2))]
        aid=self._next_area_id()
        template=None
        existing=getattr(self.map,'area_records',[])
        if existing and isinstance(existing[0].get('raw'),dict):template=existing[0]['raw']
        props={'maxspeed':0.2,'maxacc':0.3,'maxdec':0.3,'obsDecDist':1.0,'obsStopDist':0.5,'obsExpansion':0.1}
        raw=make_area_record(aid,pts,props,template)
        rec={'id':aid,'className':'AdvancedArea','points':pts,'properties':props,'raw':raw,'draft':True}
        if not hasattr(self.map,'area_records'):self.map.area_records=[]
        self.map.area_records.append(rec);self.selected_area=aid;self.map_dirty=True;self.edit_mode.set('선택')
        self.refresh_nodes();self.open_area_properties(rec)

    def _select_area_tree(self,aid):
        if not hasattr(self,'area_tree'):return
        if self.area_tree.exists(aid):
            self.area_tree.selection_set(aid);self.area_tree.see(aid)

    def _area_tree_select(self):
        if not hasattr(self,'area_tree'):return
        sel=self.area_tree.selection()
        if sel:
            self.selected_area=sel[0]
            self.editor_selection_kind='area'
            if self.tabs.select()==str(self.nodes_page):self.editor_lists.select(2)
            rec=self._find_area()
            if rec:self.node_prop_text.configure(text=f"AREA {rec.get('id')}\n"+json.dumps(rec.get('properties',{}),ensure_ascii=False,indent=2))
            self.draw_map()

    def _find_area(self,aid=None):
        aid=aid or self.selected_area
        return next((a for a in getattr(self.map,'area_records',[]) if str(a.get('id'))==str(aid)),None)

    def edit_selected_area(self):
        rec=self._find_area()
        if rec:self.open_area_properties(rec)

    def delete_selected_area(self):
        rec=self._find_area()
        if not rec:return
        if not messagebox.askyesno('Advanced Area 삭제',f"{rec.get('id')} 영역을 삭제할까요?",parent=self):return
        try:self.editable()
        except ValueError as e:messagebox.showwarning('삭제',str(e),parent=self);return
        self.map.area_records=[a for a in getattr(self.map,'area_records',[]) if a is not rec]
        self.selected_area=None;self.map_dirty=True;self.refresh_nodes()

    def open_area_properties(self,rec):
        if not isinstance(rec,dict):return
        win=tk.Toplevel(self);win.title(f"Advanced Area · {rec.get('id')}");self._fit_dialog(win,520,720,460,500);win.configure(bg=PANEL);win.transient(self)
        frm=tk.Frame(win,bg=PANEL);frm.pack(fill='both',expand=True,padx=16,pady=14)
        aid=tk.StringVar(value=str(rec.get('id') or self._next_area_id()))
        props=dict(rec.get('properties') or {})
        fields=['maxspeed','maxacc','maxdec','maxrot','maxrotacc','maxrotdec','obsDecDist','obsStopDist','obsExpansion','weight','collisionPointThreshold']
        vars_={k:tk.StringVar(value=str(props.get(k,''))) for k in fields}
        bools={k:tk.BooleanVar(value=self._bool_value(props.get(k)) is True) for k in ['forbidden','ultrasonic','fallingdown','infrared']}
        self.label(frm,'RoboShop Advanced Area',13,INK,True).pack(anchor='w')
        self.label(frm,'영역 내부에서는 Path/전역 설정보다 더 보수적인 속성이 적용됩니다.',9,'#98600a',wraplength=470,justify='left').pack(anchor='w',pady=(3,10))
        def row(label,var):
            r=tk.Frame(frm,bg=PANEL);r.pack(fill='x',pady=3);self.label(r,label,9,MUTED).pack(side='left');ttk.Entry(r,textvariable=var,width=24).pack(side='right')
        row('Area ID',aid)
        sep=tk.Frame(frm,bg='#46505d',height=1);sep.pack(fill='x',pady=7)
        row('maxspeed (m/s)',vars_['maxspeed']);row('maxacc (m/s²)',vars_['maxacc']);row('maxdec (m/s²)',vars_['maxdec'])
        row('maxrot (deg/s)',vars_['maxrot']);row('maxrotacc',vars_['maxrotacc']);row('maxrotdec',vars_['maxrotdec'])
        self.label(frm,'장애물 대응 거리',10,INK,True).pack(anchor='w',pady=(8,2))
        row('obsDecDist · 감속 시작거리 (m)',vars_['obsDecDist']);row('obsStopDist · 정지거리 (m)',vars_['obsStopDist']);row('obsExpansion · 충돌확장 (m)',vars_['obsExpansion'])
        row('weight',vars_['weight']);row('collisionPointThreshold',vars_['collisionPointThreshold'])
        self.label(frm,'영역 기능',10,INK,True).pack(anchor='w',pady=(8,2))
        for k,label in [('forbidden','진입 금지 (free navigation)'),('ultrasonic','Ultrasonic 사용'),('fallingdown','Falling-down 센서 사용'),('infrared','Infrared 사용')]:
            tk.Checkbutton(frm,text=label,variable=bools[k],bg=PANEL,activebackground=PANEL,fg=INK,selectcolor=BG).pack(anchor='w')
        self.label(frm,'예: 경로 maxspeed=0.4, Area maxspeed=0.1이면 영역 안에서는 0.1 m/s.\nobsStopDist는 더 큰(안전한) 값이 우선 적용됩니다.',9,MUTED,justify='left',wraplength=470).pack(anchor='w',pady=(10,4))
        def parse(txt):
            txt=txt.strip()
            if not txt:return None
            try:return float(txt)
            except Exception:return txt
        def save():
            new_id=aid.get().strip()
            if not new_id:messagebox.showwarning('Advanced Area','Area ID가 필요합니다.',parent=win);return
            if new_id!=rec.get('id') and any(a.get('id')==new_id for a in getattr(self.map,'area_records',[])):messagebox.showwarning('Advanced Area','같은 Area ID가 있습니다.',parent=win);return
            rec['id']=new_id; rec['className']='AdvancedArea'
            p2=dict(rec.get('properties') or {})
            for k,v in vars_.items():
                val=parse(v.get())
                if val is None:p2.pop(k,None)
                else:p2[k]=val
            for k,v in bools.items():p2[k]=bool(v.get())
            rec['properties']=p2;rec['raw']=make_area_record(new_id,rec.get('points',[]),p2,rec.get('raw'));rec['draft']=True
            self.selected_area=new_id;self.map_dirty=True;self.refresh_nodes();win.destroy()
        self.button(frm,'적용',save,ORANGE).pack(fill='x',pady=(14,4))

    def _refresh_area_tree(self):
        if not hasattr(self,'area_tree'):return
        self.area_tree.delete(*self.area_tree.get_children())
        for area in getattr(self.map,'area_records',[]):
            p=area.get('properties') or {};aid=str(area.get('id'))
            self.area_tree.insert('','end',iid=aid,values=(aid,p.get('maxspeed','-'),p.get('obsDecDist','-'),p.get('obsStopDist','-'),p.get('obsExpansion','-'),p.get('forbidden','-')))
        if self.selected_area and self.area_tree.exists(self.selected_area):self.area_tree.selection_set(self.selected_area)

    def refresh_nodes(self):
        self.node_tree.delete(*self.node_tree.get_children())
        for key,n in self.map.nodes.items():
            r=self._finite_value(n.get('r',n.get('angle')))
            angle=f'{math.degrees(r):.1f}°' if r is not None else '-'
            spin=self._bool_value(n.get('spin')) if 'spin' in n else '-'
            typ=n.get('type') or n.get('station_type') or n.get('className') or n.get('kind','station')
            summary=self._station_option_summary(n)
            self.node_tree.insert('','end',iid=key,values=(key,f"{n['x']:.3f}",f"{n['y']:.3f}",angle,str(spin),typ,summary))
        values = list(self.map.nodes)
        self.target_combo['values'] = values
        if hasattr(self,'task_combo'):
            self.task_combo['values'] = values
        for var in (self.target,self.task_goal):
            if var.get() not in values:
                var.set(values[0] if values else '')
        self._refresh_path_tree()
        self._refresh_area_tree()
        self.draw_map()

    def _next_lm_id(self):
        nums=[]
        for key in self.map.nodes:
            if key.upper().startswith('LM'):
                try: nums.append(int(key[2:]))
                except ValueError: pass
        return f'LM{max(nums,default=0)+1}'

    def create_point_at(self,x,y):
        try:self.editable()
        except ValueError as e:
            messagebox.showwarning('Point 추가',str(e),parent=self); return
        if self.task_running or self.held:
            messagebox.showwarning('Point 추가','주행/수동 조작을 먼저 종료하세요.',parent=self); return
        if self.real and not hasattr(self.map,'smap_source'):
            messagebox.showwarning('Point 추가','먼저 Pull Map으로 실제 로봇 지도를 표시하세요.',parent=self); return
        key=self._next_lm_id()
        kind='LocationMark'
        self.map.nodes[key]=dict(id=key,x=float(x),y=float(y),r=0.0,angle=0.0,spin=True,type=kind,kind='draft' if self.real else 'station',draft=bool(self.real),desc='')
        self.selected=key; self.target.set(key); self.edit_mode.set('선택'); self.map_dirty=True
        self.refresh_nodes(); self.open_point_properties(key,is_new=True)

    def add_node_dialog(self,x=0,y=0):
        # Kept for compatibility with older buttons; no X/Y typing is used anymore.
        self.edit_mode.set('Point 추가')
        messagebox.showinfo('Point 추가','Map/Control 탭에서 원하는 위치를 클릭하세요.\n좌표는 클릭 위치에서 자동 생성됩니다.',parent=self)

    def open_point_properties(self,key,is_new=False):
        """Edit a RoboShop Point with a stable, directly editable layout.

        v0.9.6: every Entry/Combobox is created with its own row frame as the
        Tk parent.  Older builds created the widgets with ``frm`` as parent and
        then tried to pack them into a different row, which caused the controls
        to collapse at the bottom of the dialog on Windows.
        """
        if key not in self.map.nodes:
            return
        n=self.map.nodes[key]
        win=tk.Toplevel(self)
        win.title(f'Point 속성 · {key}')
        self._fit_dialog(win,680,650,560,500)
        win.configure(bg=PANEL)
        win.transient(self)
        win.resizable(True,True)

        # Header + scrollable form + fixed footer.  The footer always stays
        # visible even on a 1366x768 mini-PC display.
        header=tk.Frame(win,bg=PANEL)
        header.pack(fill='x',padx=22,pady=(18,8))
        self.label(header,'RoboShop Point 설정',15,INK,True).pack(side='left')
        self.label(header,'기존 Point 직접 수정',9,MUTED).pack(side='right',pady=(5,0))

        body=tk.Frame(win,bg=PANEL)
        body.pack(fill='both',expand=True,padx=16,pady=(0,8))
        canvas=tk.Canvas(body,bg=PANEL,highlightthickness=0)
        sb=ttk.Scrollbar(body,orient='vertical',command=canvas.yview)
        canvas.configure(yscrollcommand=sb.set)
        sb.pack(side='right',fill='y')
        canvas.pack(side='left',fill='both',expand=True)
        frm=tk.Frame(canvas,bg=PANEL)
        window_id=canvas.create_window((0,0),window=frm,anchor='nw')
        frm.bind('<Configure>',lambda e:canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.bind('<Configure>',lambda e:canvas.itemconfigure(window_id,width=e.width))
        def _wheel(e):
            try: canvas.yview_scroll(int(-1*(e.delta/120)),'units')
            except Exception: pass
        canvas.bind('<MouseWheel>',_wheel)

        idv=tk.StringVar(value=key)
        typev=tk.StringVar(value=str(n.get('type') or 'LocationMark'))
        spinv=tk.BooleanVar(value=self._bool_value(n.get('spin')) is not False)
        descv=tk.StringVar(value=str(n.get('desc','')))
        xv=tk.StringVar(value=f"{float(n.get('x',0.0)):.4f}")
        yv=tk.StringVar(value=f"{float(n.get('y',0.0)):.4f}")
        heading0=self._finite_value(n.get('r',n.get('angle'))) or 0.0
        anglev=tk.StringVar(value=f"{math.degrees(heading0):.2f}")

        # IMPORTANT: controls are created with the row itself as parent.
        def entry_row(label_text,var):
            r=tk.Frame(frm,bg=PANEL)
            r.pack(fill='x',padx=10,pady=7)
            self.label(r,label_text,10,MUTED,width=18,anchor='w').pack(side='left')
            e=ttk.Entry(r,textvariable=var)
            e.pack(side='left',fill='x',expand=True,padx=(12,6),ipady=4)
            return e

        name_entry=entry_row('Name / Node ID',idv)

        r=tk.Frame(frm,bg=PANEL)
        r.pack(fill='x',padx=10,pady=7)
        self.label(r,'Point Type',10,MUTED,width=18,anchor='w').pack(side='left')
        cb=ttk.Combobox(r,textvariable=typev,state='readonly',
                        values=['OrdinaryMark','LocationMark','SpecialLocation','ParkPoint','ChargePoint'])
        cb.pack(side='left',fill='x',expand=True,padx=(12,6),ipady=3)

        desc_entry=entry_row('Description',descv)

        self.label(frm,'위치 / 방향',11,INK,True).pack(anchor='w',padx=10,pady=(18,4))
        x_entry=entry_row('X (m)',xv)
        y_entry=entry_row('Y (m)',yv)
        angle_entry=entry_row('Angle (deg)',anglev)

        r=tk.Frame(frm,bg=PANEL)
        r.pack(fill='x',padx=10,pady=8)
        self.label(r,'Spin / 도착 방향',10,MUTED,width=18,anchor='w').pack(side='left')
        spin_check=tk.Checkbutton(r,text='사용',variable=spinv,bg=PANEL,fg=INK,
                                  selectcolor='#ffffff',activebackground=PANEL,
                                  activeforeground=INK)
        spin_check.pack(side='left',padx=(12,6))

        info=tk.Frame(frm,bg=PANEL,highlightbackground='#ccd5e0',highlightthickness=1)
        info.pack(fill='x',padx=10,pady=(18,10))
        current_type=str(n.get('type') or 'LocationMark')
        self.label(info,f'현재 Point : {key}   |   Type : {current_type}',10,INK,True,bg='#282e35').pack(anchor='w',padx=14,pady=(12,4))
        self.label(info,f'현재 위치 : X={float(n.get("x",0.0)):.3f} m   Y={float(n.get("y",0.0)):.3f} m   Angle={math.degrees(heading0):.1f}°',
                   9,'#98600a',True,bg='#282e35').pack(anchor='w',padx=14,pady=(0,10))

        self.label(frm,'수정 방법',10,INK,True).pack(anchor='w',padx=10,pady=(8,3))
        self.label(frm,
            '위 값을 직접 수정한 뒤 [적용]을 누르세요. Name을 변경하면 연결된 Path의 시작/끝 Node ID도 같이 변경됩니다. '
            'X/Y를 변경하면 연결 Curve 끝점도 Point 위치를 따라 이동합니다. Angle은 degree 단위입니다.',
            9,MUTED,wraplength=590,justify='left').pack(anchor='w',padx=10,pady=(0,14))

        def save():
            new=idv.get().strip()
            if not new:
                messagebox.showerror('Point','Name이 필요합니다.',parent=win); return
            if new!=key and new in self.map.nodes:
                messagebox.showerror('Point','중복 Name입니다.',parent=win); return
            try:
                nx=float(xv.get().strip())
                ny=float(yv.get().strip())
                nang=math.radians(float(anglev.get().strip()))
                if not all(math.isfinite(v) for v in (nx,ny,nang)):
                    raise ValueError
            except Exception:
                messagebox.showerror('Point','X/Y/Angle 값이 올바르지 않습니다.',parent=win); return
            oldnode=self.map.nodes[key]
            oldx=float(oldnode.get('x',0.0)); oldy=float(oldnode.get('y',0.0))
            if new!=key:
                self.map.nodes[new]=self.map.nodes.pop(key)
                self.map.nodes[new]['id']=new
                self.map.edges=[[new if a==key else a,new if b==key else b] for a,b in self.map.edges]
                for rec in getattr(self.map,'path_records',[]):
                    if rec.get('a')==key: rec['a']=new
                    if rec.get('b')==key: rec['b']=new
                    if rec.get('a')==new or rec.get('b')==new:
                        rec['instanceName']=f"{rec.get('a')}-{rec.get('b')}"
            node=self.map.nodes[new]
            node.update(x=nx,y=ny,r=nang,angle=nang,type=typev.get(),
                        spin=bool(spinv.get()),desc=descv.get())
            self._sync_paths_after_point_edit(key,new,oldx,oldy,nx,ny)
            self.selected=new
            self.target.set(new)
            self.map_dirty=True
            self.refresh_nodes()
            self.draw_map()
            if self.real:
                self.log('EDIT',f'로컬 Point {new} 수정 · X={nx:.3f} Y={ny:.3f} Angle={math.degrees(nang):.1f}° · 실제 로봇 반영 전 SMAP Push 필요')
            win.destroy()

        def remove_point():
            target=idv.get().strip() or key
            actual=target if target in self.map.nodes else key
            linked=[r for r in getattr(self.map,'path_records',[]) if r.get('a')==actual or r.get('b')==actual]
            if linked:
                messagebox.showwarning('Point 삭제',f'{actual}에 연결된 Path {len(linked)}개가 있습니다.\n먼저 연결 Path를 삭제하세요.',parent=win)
                return
            if not messagebox.askyesno('Point 삭제',f'{actual} Point를 삭제할까요?',parent=win):
                return
            self.map.nodes.pop(actual,None)
            self.map_dirty=True
            if self.selected==actual:self.selected=None
            self.refresh_nodes(); self.draw_map(); win.destroy()

        footer=tk.Frame(win,bg='#252a31',highlightbackground='#46515e',highlightthickness=1)
        footer.pack(fill='x',side='bottom')
        self.button(footer,'적용',save,BLUE).pack(side='right',padx=(6,14),pady=12)
        self.button(footer,'취소',win.destroy).pack(side='right',padx=6,pady=12)
        self.button(footer,'Point 삭제',remove_point,RED).pack(side='left',padx=14,pady=12)

        # Friendly initial focus and Enter shortcut.
        name_entry.focus_set()
        win.bind('<Return>',lambda e:save())
        win.bind('<Escape>',lambda e:win.destroy())

    def _sync_paths_after_point_edit(self,old_key,new_key,oldx,oldy,newx,newy):
        """Keep directed AdvancedCurve records attached when a Point is renamed/moved."""
        dx=float(newx)-float(oldx); dy=float(newy)-float(oldy)
        for rec in getattr(self.map,'path_records',[]):
            raw=rec.get('raw') if isinstance(rec.get('raw'),dict) else {}
            touched_start=(rec.get('a')==new_key)
            touched_end=(rec.get('b')==new_key)
            if not (touched_start or touched_end): continue
            rec['instanceName']=f"{rec.get('a')}-{rec.get('b')}"
            raw['instanceName']=rec['instanceName']
            if touched_start:
                sp=raw.setdefault('startPos',{}); sp['instanceName']=new_key
                pos=sp.setdefault('pos',{}); pos['x']=float(newx); pos['y']=float(newy)
                cp=raw.get('controlPos1')
                if isinstance(cp,dict): cp['x']=float(cp.get('x',oldx))+dx; cp['y']=float(cp.get('y',oldy))+dy
            if touched_end:
                ep=raw.setdefault('endPos',{}); ep['instanceName']=new_key
                pos=ep.setdefault('pos',{}); pos['x']=float(newx); pos['y']=float(newy)
                cp=raw.get('controlPos2')
                if isinstance(cp,dict): cp['x']=float(cp.get('x',oldx))+dx; cp['y']=float(cp.get('y',oldy))+dy
            rec['raw']=raw
            c1=raw.get('controlPos1') or {'x':self.map.nodes[rec['a']]['x'],'y':self.map.nodes[rec['a']]['y']}
            c2=raw.get('controlPos2') or {'x':self.map.nodes[rec['b']]['x'],'y':self.map.nodes[rec['b']]['y']}
            rec['controls']=[(float(c1.get('x')),float(c1.get('y'))),(float(c2.get('x')),float(c2.get('y')))]
        self.map.edges=[[r.get('a'),r.get('b')] for r in getattr(self.map,'path_records',[]) if r.get('a') in self.map.nodes and r.get('b') in self.map.nodes]
        self.map.curves=[path_record_geometry(r.get('raw',{})) for r in getattr(self.map,'path_records',[])]

    def edit_node(self):
        key=self.selected if self.selected in self.map.nodes else None
        if not key and hasattr(self,'node_tree'):
            sel=self.node_tree.selection(); key=sel[0] if len(sel)==1 else None
        if not key:
            messagebox.showinfo('Point 속성','맵 또는 목록에서 Point 하나를 선택하세요.',parent=self); return
        self.open_point_properties(key)

    def delete_node(self):
        def delete():
            self.editable()
            keys = self.node_tree.selection()
            if not keys: return
            if len(keys)>=len(self.map.nodes):raise ValueError('노드 한 개 이상 남겨야 합니다.')
            if any(t['goal'] in keys for t in self.tasks):raise ValueError('Task에서 참조 중인 노드는 삭제할 수 없습니다.')
            if not messagebox.askyesno('노드 삭제','선택 노드와 연결 경로를 삭제할까요?'):return
            for key in keys:self.map.nodes.pop(key)
            self.map.edges = [[a,b] for a,b in self.map.edges if a not in keys and b not in keys]
            if hasattr(self.map,'path_records'):
                self.map.path_records=[r for r in self.map.path_records if r.get('a') not in keys and r.get('b') not in keys]
                self.map.curves=[path_record_geometry(r.get('raw',{})) if isinstance(r.get('raw'),dict) else [] for r in self.map.path_records]
            self.pending_link = None; self.map_dirty=True
            self.refresh_nodes()
        self.guarded(delete)

    def _path_reverse(self,rec):
        if not isinstance(rec,dict): return None
        a,b=rec.get('a'),rec.get('b')
        return next((r for r in getattr(self.map,'path_records',[]) if r is not rec and r.get('a')==b and r.get('b')==a),None)

    def _path_traffic_label(self,rec):
        if not isinstance(rec,dict): return '-'
        a,b=rec.get('a'),rec.get('b')
        return f'{a} ↔ {b} 양방향' if self._path_reverse(rec) else f'{a} → {b} 단방향'

    def _drive_label(self,rec):
        val=(rec.get('properties') or {}).get('direction') if isinstance(rec,dict) else None
        try: iv=int(val)
        except Exception: return str(val) if val not in (None,'') else '기본'
        return '후진' if iv==1 else '전진' if iv==0 else str(iv)

    def _refresh_path_tree(self):
        if not hasattr(self,'path_tree'): return
        self.path_tree.delete(*self.path_tree.get_children())
        records=getattr(self.map,'path_records',[])
        directed=0; paired=0
        for i,rec in enumerate(records):
            a,b=rec.get('a'),rec.get('b')
            if not a or not b: continue
            rev=self._path_reverse(rec)
            traffic=f'{a} → {b}' + ('  (역방향 있음)' if rev else '  (단방향)')
            props=rec.get('properties') or {}
            iid=f'path_{i}'
            self.path_tree.insert('', 'end', iid=iid, values=(rec.get('instanceName') or f'{a}-{b}',traffic,self._drive_label(rec),props.get('movestyle',''),props.get('maxspeed','')))
            directed+=1
            if rev: paired+=1
        self.edge_text.config(text=f'방향별 Path {directed}개 · 왕복 쌍 {paired//2}개   |   각 행을 개별 편집: A→B 전진 / B→A 후진처럼 서로 다른 설정 가능')

    def _selected_path_record(self):
        if not hasattr(self,'path_tree'): return None
        sel=self.path_tree.selection()
        if not sel:return None
        try:i=int(sel[0].split('_',1)[1])
        except Exception:return None
        records=getattr(self.map,'path_records',[])
        return records[i] if 0<=i<len(records) else None

    def _set_curve_controls(self,rec,controls):
        rec['controls']=[(number(x),number(y)) for x,y in controls]
        rec['className']='BezierPath'; rec['draft']=True
        raw=rec['raw'];raw['className']='BezierPath'
        for i,(x,y) in enumerate(rec['controls'],1):raw[f'controlPos{i}']={'x':x,'y':y}
        # The map displays one physical geometry for a round-trip pair.
        for reverse in getattr(self.map,'path_records',[]):
            if reverse is rec or reverse.get('a')!=rec['b'] or reverse.get('b')!=rec['a']:continue
            reverse['controls']=list(reversed(rec['controls']))
            reverse['className']='BezierPath';reverse['draft']=True
            reverse['raw']['className']='BezierPath'
            for i,(x,y) in enumerate(reverse['controls'],1):
                reverse['raw'][f'controlPos{i}']={'x':x,'y':y}
        self.map_dirty=True

    def _curve_click(self,e):
        self.editable()
        rec=self.curve_edit_record
        records=getattr(self.map,'path_records',[])
        if any(r is rec for r in records):
            for i,p in enumerate(rec['controls']):
                px,py=self.xy(*p)
                if math.hypot(px-e.x,py-e.y)<=12:
                    self.curve_drag_index=i;return
        def segment_distance(a,b):
            ax,ay=self.xy(*a);bx,by=self.xy(*b);dx,dy=bx-ax,by-ay
            t=max(0,min(1,((e.x-ax)*dx+(e.y-ay)*dy)/(dx*dx+dy*dy))) if dx or dy else 0
            return math.hypot(e.x-ax-t*dx,e.y-ay-t*dy)
        candidates=[]
        for r in records:
            pts=path_record_geometry(r.get('raw',{}))
            if len(pts)>1:candidates.append((min(segment_distance(a,b) for a,b in zip(pts,pts[1:])),r))
        represented={frozenset((r['a'],r['b'])) for r in records}
        for a,b in self.map.edges:
            if frozenset((a,b)) in represented:continue
            na,nb=self.map.nodes[a],self.map.nodes[b]
            candidates.append((segment_distance((na['x'],na['y']),(nb['x'],nb['y'])),(a,b)))
        if not candidates:return
        distance,rec=min(candidates,key=lambda item:item[0])
        if distance>12:return
        if isinstance(rec,tuple):
            a,b=rec
            if not hasattr(self.map,'path_records'):self.map.path_records=[]
            # Convert all demo edges so route() retains its original connectivity.
            for src,dst in self.map.edges:
                for u,v in ((src,dst),(dst,src)):
                    if any(r['a']==u and r['b']==v for r in self.map.path_records):continue
                    raw=make_path_record(self.map.nodes[u],self.map.nodes[v])
                    self.map.path_records.append(dict(a=u,b=v,className='BezierPath',raw=raw,
                        controls=[(raw[k]['x'],raw[k]['y']) for k in ('controlPos1','controlPos2')],properties={},instanceName=f'{u}-{v}'))
            rec=next(r for r in self.map.path_records if r['a']==a and r['b']==b)
        controls=rec.get('controls')
        if not controls:
            raw=make_path_record(self.map.nodes[rec['a']],self.map.nodes[rec['b']])
            controls=[(raw[k]['x'],raw[k]['y']) for k in ('controlPos1','controlPos2')]
        self._set_curve_controls(rec,controls)
        self.curve_edit_record=rec
        self.refresh_nodes();self.draw_map()

    def _path_tree_select(self):
        rec=self._selected_path_record()
        if not rec:return
        self.editor_selection_kind='path'
        if self.tabs.select()==str(self.nodes_page):self.editor_lists.select(1)
        props=rec.get('properties') or {}
        self.node_prop_text.config(text=(f"PATH {rec.get('a')} → {rec.get('b')}\n"
            f"통행: {self._path_traffic_label(rec)}\n주행: {self._drive_label(rec)}\n"
            f"Type: {rec.get('className')}\nMoveStyle: {props.get('movestyle','-')}\n"
            f"MaxSpeed: {props.get('maxspeed','-')}  MaxAcc: {props.get('maxacc','-')}\n"
            f"ReachDist: {props.get('reachdist','-')}  ReachAngle: {props.get('reachangle','-')}"))
        self.draw_map()

    def edit_selected_path(self):
        rec=self._selected_path_record()
        if rec:self.open_path_properties(rec)

    def delete_selected_path(self):
        if hasattr(self,'studio_runner') and not self._studio_can_edit():return
        rec=self._selected_path_record()
        if not rec:return
        if not messagebox.askyesno('단방향 Path 삭제',f"{rec.get('a')} → {rec.get('b')} 경로만 삭제할까요?\n역방향 Path는 유지됩니다.",parent=self):return
        self.map.path_records=[r for r in self.map.path_records if r is not rec]
        self.map.edges=[[r.get('a'),r.get('b')] for r in self.map.path_records if r.get('a') in self.map.nodes and r.get('b') in self.map.nodes]
        self.map.curves=[path_record_geometry(r.get('raw',{})) for r in self.map.path_records]
        self.map_dirty=True;self.refresh_nodes()

    def add_reverse_selected_path(self):
        if hasattr(self,'studio_runner') and not self._studio_can_edit():return
        rec=self._selected_path_record()
        if not rec:return
        if self._path_reverse(rec):
            messagebox.showinfo('역방향 Path',f"{rec.get('b')} → {rec.get('a')} Path가 이미 있습니다. 목록에서 해당 행을 선택해 독립적으로 수정하세요.",parent=self);return
        rev=self._clone_reverse_path(rec)
        self.map.path_records.append(rev)
        self.map.edges.append([rev.get('a'),rev.get('b')])
        self.map.curves.append(path_record_geometry(rev.get('raw',{})))
        self.map_dirty=True; self.refresh_nodes()
        # Open the newly created reverse direction immediately so forward/backward can differ.
        self.open_path_properties(rev)

    def _clone_reverse_path(self,rec):
        import copy
        a,b=rec.get('a'),rec.get('b')
        raw=copy.deepcopy(rec.get('raw') or {})
        raw['instanceName']=f'{b}-{a}'
        raw['startPos'],raw['endPos']=copy.deepcopy(raw.get('endPos',{})),copy.deepcopy(raw.get('startPos',{}))
        # Reverse a cubic Bezier without changing its physical geometry.
        cp1=copy.deepcopy(raw.get('controlPos1')); cp2=copy.deepcopy(raw.get('controlPos2'))
        if cp1 is not None or cp2 is not None:
            raw['controlPos1']=cp2 if cp2 is not None else copy.deepcopy(raw.get('endPos',{}).get('pos',{}))
            raw['controlPos2']=cp1 if cp1 is not None else copy.deepcopy(raw.get('startPos',{}).get('pos',{}))
        c1=raw.get('controlPos1') or {'x':self.map.nodes[b]['x'],'y':self.map.nodes[b]['y']}
        c2=raw.get('controlPos2') or {'x':self.map.nodes[a]['x'],'y':self.map.nodes[a]['y']}
        return {'a':b,'b':a,'className':rec.get('className','BezierPath'),'instanceName':f'{b}-{a}',
                'controls':[(float(c1.get('x',self.map.nodes[b]['x'])),float(c1.get('y',self.map.nodes[b]['y']))),
                            (float(c2.get('x',self.map.nodes[a]['x'])),float(c2.get('y',self.map.nodes[a]['y'])))],
                'properties':dict(rec.get('properties') or {}),'raw':raw,'draft':True}

    def _apply_path_traffic(self,rec,mode):
        # mode: AB / BA / BOTH. Physical bidirection is represented by two opposite
        # AdvancedCurve records, matching RoboShop SMAP exports.
        records=self.map.path_records
        rev=self._path_reverse(rec)
        if mode=='BOTH':
            if not rev: records.append(self._clone_reverse_path(rec))
        elif mode=='AB':
            if rev: records.remove(rev)
        elif mode=='BA':
            if rev:
                # Keep reverse and delete current record.
                records.remove(rec); rec=rev
            else:
                new=self._clone_reverse_path(rec); records.remove(rec);records.append(new);rec=new
        self.map.edges=[[r.get('a'),r.get('b')] for r in records if r.get('a') in self.map.nodes and r.get('b') in self.map.nodes]
        self.map.curves=[path_record_geometry(r.get('raw',{})) for r in records]
        return rec

    def link(self,a,b):
        self.editable()
        if a not in self.map.nodes or b not in self.map.nodes:raise ValueError('노드가 없습니다.')
        if a==b:raise ValueError('같은 Point끼리는 연결할 수 없습니다.')
        if any(r.get('a')==a and r.get('b')==b for r in getattr(self.map,'path_records',[])):
            raise ValueError('같은 방향의 Path가 이미 있습니다.')
        if not hasattr(self.map,'path_records'):self.map.path_records=[]
        # New RoboShop-style connection defaults to cubic Bezier.  For firmware-specific
        # properties use the first existing path as a template and preserve its property schema.
        template=None; template_props={}
        for r in self.map.path_records:
            if isinstance(r.get('raw'),dict):
                template=r['raw']; template_props=dict(r.get('properties') or {}); break
        raw=make_path_record(self.map.nodes[a],self.map.nodes[b],'BezierPath',template_props,template)
        rec={'a':a,'b':b,'className':'BezierPath','instanceName':f'{a}-{b}',
             'controls':[(raw['controlPos1']['x'],raw['controlPos1']['y']),(raw['controlPos2']['x'],raw['controlPos2']['y'])],
             'properties':template_props,'raw':raw,'draft':True}
        self.map.path_records.append(rec); self.map.edges.append([a,b])
        self.map.curves.append(path_record_geometry(raw)); self.map_dirty=True
        self.refresh_nodes(); self.open_path_properties(rec)

    def connect_nodes(self):
        try:self.editable()
        except ValueError as e:
            messagebox.showwarning('Path 연결',str(e),parent=self);return
        keys=list(self.node_tree.selection())
        if len(keys)!=2:
            messagebox.showinfo('Path 연결','Ctrl+클릭으로 노드 두 개를 선택하세요.',parent=self);return
        a,b=keys[0],keys[1]
        win=tk.Toplevel(self);win.title(f'방향별 Path 생성 · {a} / {b}');self._fit_dialog(win,520,390,460,360);win.configure(bg=PANEL);win.transient(self)
        frm=tk.Frame(win,bg=PANEL);frm.pack(fill='both',expand=True,padx=20,pady=18)
        self.label(frm,'두 노드 사이 방향별 Path 설정',14,INK,True).pack(anchor='w',pady=(0,8))
        self.label(frm,'각 방향은 서로 독립된 AdvancedCurve입니다. 전진/후진도 방향별로 따로 저장됩니다.',9,MUTED,wraplength=470,justify='left').pack(anchor='w',pady=(0,12))
        ab=tk.BooleanVar(value=not any(r.get('a')==a and r.get('b')==b for r in getattr(self.map,'path_records',[])))
        ba=tk.BooleanVar(value=not any(r.get('a')==b and r.get('b')==a for r in getattr(self.map,'path_records',[])))
        abd=tk.StringVar(value='전진 (0)'); bad=tk.StringVar(value='후진 (1)')
        def prow(title,var,drive):
            r=tk.Frame(frm,bg='#282e35');r.pack(fill='x',pady=5)
            tk.Checkbutton(r,text=title,variable=var,bg='#282e35',fg=INK,selectcolor=PANEL,activebackground='#282e35',activeforeground=INK).pack(side='left',padx=8,pady=8)
            ttk.Combobox(r,textvariable=drive,values=['전진 (0)','후진 (1)'],state='readonly',width=15).pack(side='right',padx=8)
        prow(f'{a} → {b}',ab,abd); prow(f'{b} → {a}',ba,bad)
        self.label(frm,'예: A→B 전진 + B→A 후진을 선택하면 같은 두 노드 사이에 단방향 Path 2개가 생성됩니다.',9,'#98600a',wraplength=470,justify='left').pack(anchor='w',pady=10)
        def create_dir(src,dst,drive_value):
            if any(r.get('a')==src and r.get('b')==dst for r in getattr(self.map,'path_records',[])):return
            if not hasattr(self.map,'path_records'):self.map.path_records=[]
            template=None; template_props={}
            for r in self.map.path_records:
                if isinstance(r.get('raw'),dict):template=r['raw'];template_props=dict(r.get('properties') or {});break
            template_props=dict(template_props);template_props['direction']=1 if drive_value.startswith('후진') else 0
            raw=make_path_record(self.map.nodes[src],self.map.nodes[dst],'BezierPath',template_props,template)
            rec={'a':src,'b':dst,'className':'BezierPath','instanceName':f'{src}-{dst}',
                 'controls':[(raw['controlPos1']['x'],raw['controlPos1']['y']),(raw['controlPos2']['x'],raw['controlPos2']['y'])],
                 'properties':template_props,'raw':raw,'draft':True}
            self.map.path_records.append(rec)
        def apply():
            if not ab.get() and not ba.get():
                messagebox.showerror('Path 연결','최소 한 방향을 선택하세요.',parent=win);return
            create_dir(a,b,abd.get()) if ab.get() else None
            create_dir(b,a,bad.get()) if ba.get() else None
            self.map.edges=[[r.get('a'),r.get('b')] for r in self.map.path_records if r.get('a') in self.map.nodes and r.get('b') in self.map.nodes]
            self.map.curves=[path_record_geometry(r.get('raw',{})) for r in self.map.path_records]
            self.map_dirty=True;self.refresh_nodes();win.destroy()
        self.button(frm,'방향별 Path 생성',apply,BLUE).pack(fill='x',pady=(14,4))

    def unlink_nodes(self):
        def unlink():
            self.editable()
            keys = self.node_tree.selection()
            if len(keys)!=2:raise ValueError('노드 두 개를 선택하세요.')
            self.map.edges = [e for e in self.map.edges if set(e)!=set(keys)]
            if hasattr(self.map,'path_records'):
                self.map.path_records=[r for r in self.map.path_records if {r.get('a'),r.get('b')}!=set(keys)]
                self.map.curves=[path_record_geometry(r.get('raw',{})) if isinstance(r.get('raw'),dict) else [] for r in self.map.path_records]
            self.map_dirty=True; self.refresh_nodes()
        self.guarded(unlink)

    def open_path_properties(self,rec):
        if hasattr(self,'studio_runner') and not self._studio_can_edit():return
        if not isinstance(rec,dict):return
        a0,b0=rec.get('a'),rec.get('b')
        win=tk.Toplevel(self); win.title(f"단방향 Path 속성 · {a0} → {b0}"); self._fit_dialog(win,540,760,500,520); win.configure(bg=PANEL); win.transient(self)
        shell=tk.Frame(win,bg=PANEL);shell.pack(fill='both',expand=True)
        cvs=tk.Canvas(shell,bg=PANEL,highlightthickness=0);sb=ttk.Scrollbar(shell,orient='vertical',command=cvs.yview);cvs.configure(yscrollcommand=sb.set)
        sb.pack(side='right',fill='y');cvs.pack(side='left',fill='both',expand=True)
        frm=tk.Frame(cvs,bg=PANEL);wid=cvs.create_window((0,0),window=frm,anchor='nw')
        frm.bind('<Configure>',lambda e:cvs.configure(scrollregion=cvs.bbox('all')));cvs.bind('<Configure>',lambda e:cvs.itemconfigure(wid,width=e.width))
        inner=tk.Frame(frm,bg=PANEL);inner.pack(fill='both',expand=True,padx=18,pady=14)
        typ=tk.StringVar(value=str(rec.get('className') or 'BezierPath'))
        props=dict(rec.get('properties') or {})
        drive=tk.StringVar(value='후진 (1)' if str(props.get('direction','0'))=='1' else '전진 (0)')
        vars_={k:tk.StringVar(value=str(props.get(k,''))) for k in ['movestyle','maxspeed','maxacc','maxdec','maxrot','maxrotacc','maxrotdec','reachdist','reachangle','width','length','holdDir','obsStopDist','obsDecDist','obsExpansion']}
        self.label(inner,f"{a0}  →  {b0}",15,INK,True).pack(anchor='w',pady=(0,3))
        rev=self._path_reverse(rec)
        self.label(inner,f"독립 단방향 Path · 역방향 {'있음' if rev else '없음'}",9,'#98600a').pack(anchor='w',pady=(0,10))
        def field(label,var,values=None):
            r=tk.Frame(inner,bg=PANEL);r.pack(fill='x',pady=3);self.label(r,label,9,MUTED).pack(side='left')
            w=ttk.Combobox(r,textvariable=var,values=values,state='readonly',width=28) if values else ttk.Entry(r,textvariable=var,width=30)
            w.pack(side='right');return w
        field('Path Type',typ,['BezierPath','StraightPath'])
        field('주행 방향 direction',drive,['전진 (0)','후진 (1)'])
        sep=tk.Frame(inner,bg='#46505d',height=1);sep.pack(fill='x',pady=8)
        field('movestyle',vars_['movestyle']); field('maxspeed (m/s)',vars_['maxspeed']); field('maxacc (m/s²)',vars_['maxacc']); field('maxdec (m/s²)',vars_['maxdec'])
        field('maxrot (deg/s)',vars_['maxrot']); field('maxrotacc',vars_['maxrotacc']); field('maxrotdec',vars_['maxrotdec'])
        field('reachdist (m)',vars_['reachdist']); field('reachangle (deg)',vars_['reachangle'])
        field('width (m)',vars_['width']); field('length (m)',vars_['length']); field('holdDir',vars_['holdDir'])
        field('obsStopDist (m)',vars_['obsStopDist']); field('obsDecDist (m)',vars_['obsDecDist']); field('obsExpansion (m)',vars_['obsExpansion'])
        self.label(inner,'이 값들은 이 방향에만 적용됩니다. 역방향 Path는 목록에서 별도로 선택해 전진/후진/속도/장애물 거리를 다르게 설정하세요.',9,MUTED,wraplength=490,justify='left').pack(anchor='w',pady=(10,4))
        def parseval(txt):
            txt=txt.strip()
            if txt=='':return None
            try:v=float(txt);return int(v) if v.is_integer() else v
            except Exception:return txt
        def save():
            if hasattr(self,'studio_runner') and not self._studio_can_edit():return
            rec['className']=typ.get();props2=dict(rec.get('properties') or {});props2['direction']=1 if drive.get().startswith('후진') else 0
            for k,var in vars_.items():
                val=parseval(var.get())
                if val is None:props2.pop(k,None)
                else:props2[k]=val
            rec['properties']=props2;rec['draft']=True
            raw=make_path_record(self.map.nodes[rec['a']],self.map.nodes[rec['b']],rec['className'],props2,rec.get('raw'))
            if typ.get()=='BezierPath' and rec.get('controls'):
                for i,(x,y) in enumerate(rec['controls'],1):raw[f'controlPos{i}']={'x':x,'y':y}
            rec['raw']=raw;rec['instanceName']=f"{rec['a']}-{rec['b']}";rec['controls']=[(raw['controlPos1']['x'],raw['controlPos1']['y']),(raw['controlPos2']['x'],raw['controlPos2']['y'])]
            self.map.edges=[[r.get('a'),r.get('b')] for r in self.map.path_records if r.get('a') in self.map.nodes and r.get('b') in self.map.nodes]
            self.map.curves=[path_record_geometry(r.get('raw',{})) for r in self.map.path_records]
            self.map_dirty=True;self.refresh_nodes();win.destroy()
        self.button(inner,'이 방향 Path 적용',save,BLUE).pack(fill='x',pady=(20,6))
        if not rev:
            def add_rev():
                save()
                # save closes window; use current record reference and open reverse after refresh
                if not self._path_reverse(rec):
                    r=self._clone_reverse_path(rec);self.map.path_records.append(r);self.map_dirty=True;self.refresh_nodes();self.open_path_properties(r)
            # Separate button intentionally omitted here to avoid save/close ordering ambiguity; use list button '역방향 Path 추가'.

    def validate_map_editor(self,show=True):
        errors=validate_smap_edit(self.map)
        if errors:
            if show:messagebox.showerror('맵 검증 실패','\n'.join('• '+e for e in errors[:12]),parent=self)
            return False
        path_count=len(getattr(self.map,'path_records',[]))
        if show:messagebox.showinfo('맵 검증',f'검증 OK\nPoint {len(self.map.nodes)}개\nPath {path_count}개\nAdvanced Area {len(getattr(self.map,"area_records",[]))}개\n현재 맵: {self.current_robot_map or self.map.name}',parent=self)
        return True

    @staticmethod
    def _require_robot_ok(response, api_name):
        if isinstance(response,dict):
            code=response.get('ret_code',0)
            try: code_i=int(code)
            except Exception: code_i=code
            if code_i not in (0,None):
                msg=response.get('err_msg') or response.get('message') or 'unknown controller error'
                raise ValueError(f'{api_name} 거부 · ret_code={code} · {msg}')
        return response

    def push_map_to_robot(self):
        if not self.real or not self.connected or not self.control_enabled:
            messagebox.showwarning('AMR 업데이트','실기 · 제어 모드로 연결하세요.',parent=self);return
        try:self.editable()
        except ValueError as e:messagebox.showwarning('AMR 업데이트',str(e),parent=self);return
        if not self.validate_map_editor(show=False):
            self.validate_map_editor(show=True);return
        if not self.map_dirty:
            if not messagebox.askyesno('AMR 업데이트','로컬 변경 표시가 없습니다. 그래도 현재 SMAP을 다시 업로드할까요?',parent=self):return
        if not messagebox.askyesno('AMR 맵 업데이트',
            f"현재 맵 [{self.current_robot_map or self.map.name}]을 수정본으로 업데이트합니다.\n\n"
            "원본은 PC에 자동 백업하고 4010으로 전체 SMAP을 업로드한 뒤 4011로 다시 내려받아 검증합니다.\n"
            + ('업로드 후 2022로 현재 맵을 재적용합니다.\n' if self.auto_apply_uploaded_map.get() else '')+
            "AMR가 정지 상태인지 확인한 뒤 진행하세요.",parent=self):return
        self.map_push_in_progress=True
        self.connection_text.config(text='AMR 맵 업데이트 중…')
        generation=self.generation;host=self.host.get().strip();apply_now=bool(self.auto_apply_uploaded_map.get())
        try:profile_data=json.loads(self.profile.read_text(encoding='utf-8-sig'))
        except Exception as e:
            self.map_push_in_progress=False;messagebox.showerror('AMR 업데이트',str(e));return
        model=self.map
        def work():
            try:
                started=time.monotonic()
                smap=build_smap_from_model(model)
                map_name=str(smap.get('header',{}).get('mapName') or self.current_robot_map or model.name)
                expected_nodes=set(model.nodes)
                expected_paths={(r.get('a'),r.get('b')) for r in getattr(model,'path_records',[])}
                draft_nodes=sorted(k for k,n in model.nodes.items() if n.get('draft'))
                self.events.put(('map_push_stage',generation,
                    f'업로드 데이터 준비 · map={map_name} · Point={len(expected_nodes)} · Path={len(expected_paths)} · 신규/수정 Point={draft_nodes[:12]}'))
                USER_DIR.mkdir(parents=True,exist_ok=True);backup_dir=USER_DIR/'map_backups';backup_dir.mkdir(parents=True,exist_ok=True)
                stamp=datetime.now().strftime('%Y%m%d_%H%M%S')
                safe=''.join(ch if ch.isalnum() or ch in '._-' else '_' for ch in map_name).strip('._') or 'map'
                backup=backup_dir/f'{safe}_{stamp}_before_push.smap'
                backup.write_text(json.dumps(model.smap_source,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
                edited=backup_dir/f'{safe}_{stamp}_edited_to_push.smap'
                edited.write_text(json.dumps(smap,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
                client=SeerClient(host,profile_data);client.timeout=20.0
                self.events.put(('map_push_stage',generation,f'4010 Upload 시작 · {map_name} · bytes={edited.stat().st_size}'))
                ack=client.upload_map(smap)
                self._require_robot_ok(ack,'4010 Upload Map')
                self.events.put(('map_push_stage',generation,f'4010 ACK OK · {ack}'))
                verify=client.download_map(map_name)
                # Verify against the raw SMAP JSON rather than the display parser.  Live
                # firmware variants may use LocationMark vs LandMark class names, which
                # previously caused a false 'all nodes missing' result even after 4010 ACK.
                raw_verify=verify
                if isinstance(raw_verify,str):
                    raw_verify=json.loads(raw_verify)
                if isinstance(raw_verify,dict) and not isinstance(raw_verify.get('header'),dict):
                    for wk in ('map','data','map_data','content'):
                        vv=raw_verify.get(wk)
                        if isinstance(vv,str):
                            try: vv=json.loads(vv)
                            except Exception: vv=None
                        if isinstance(vv,dict) and isinstance(vv.get('header'),dict):
                            raw_verify=vv; break
                if not isinstance(raw_verify,dict):
                    raise ValueError('4011 검증 응답이 SMAP JSON이 아닙니다.')
                actual_nodes={str(v.get('instanceName')) for v in raw_verify.get('advancedPointList',[]) if isinstance(v,dict) and v.get('instanceName') is not None}
                actual_paths=set()
                for cv in raw_verify.get('advancedCurveList',[]):
                    if not isinstance(cv,dict): continue
                    sp=cv.get('startPos') if isinstance(cv.get('startPos'),dict) else {}
                    ep=cv.get('endPos') if isinstance(cv.get('endPos'),dict) else {}
                    a=sp.get('instanceName'); b=ep.get('instanceName')
                    if a is not None and b is not None: actual_paths.add((str(a),str(b)))
                missing=sorted(expected_nodes-actual_nodes)
                missing_paths=sorted(expected_paths-actual_paths)
                # Save the robot round-trip copy even on verification failure, so field
                # differences can be inspected without guessing.
                verify_file=backup_dir/f'{safe}_{stamp}_4011_roundtrip.smap'
                verify_file.write_text(json.dumps(raw_verify,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
                if missing or missing_paths:
                    # Some RBK firmware acknowledges 4010 for an existing map name but keeps
                    # the old map unchanged.  In that case create a new map name and upload
                    # the edited SMAP as a clone, then switch to it after verification.
                    self.events.put(('map_push_stage',generation,
                        f'기존 mapName 덮어쓰기 미반영 감지 · missing nodes={missing[:8]} paths={missing_paths[:8]} · 새 이름으로 재업로드'))
                    clone_stamp=datetime.now().strftime('%Y%m%d_%H%M%S')
                    base=''.join(ch if ch.isalnum() or ch in '_-' else '_' for ch in map_name)[:42].strip('_-') or 'map'
                    clone_name=f'{base}_edit_{clone_stamp}'
                    clone_smap=json.loads(json.dumps(smap))
                    clone_smap.setdefault('header',{})['mapName']=clone_name
                    clone_file=backup_dir/f'{clone_name}_edited_to_push.smap'
                    clone_file.write_text(json.dumps(clone_smap,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
                    self.events.put(('map_push_stage',generation,f'4010 Clone Upload 시작 · {clone_name} · bytes={clone_file.stat().st_size}'))
                    clone_ack=client.upload_map(clone_smap)
                    self._require_robot_ok(clone_ack,'4010 Clone Upload Map')
                    self.events.put(('map_push_stage',generation,f'4010 Clone ACK OK · {clone_ack}'))
                    time.sleep(0.25)
                    clone_verify=client.download_map(clone_name)
                    if isinstance(clone_verify,str): clone_verify=json.loads(clone_verify)
                    if isinstance(clone_verify,dict) and not isinstance(clone_verify.get('header'),dict):
                        for wk in ('map','data','map_data','content'):
                            vv=clone_verify.get(wk)
                            if isinstance(vv,str):
                                try: vv=json.loads(vv)
                                except Exception: vv=None
                            if isinstance(vv,dict) and isinstance(vv.get('header'),dict):
                                clone_verify=vv; break
                    if not isinstance(clone_verify,dict):
                        raise ValueError('새 mapName 4011 검증 응답이 SMAP JSON이 아닙니다.')
                    clone_nodes={str(v.get('instanceName')) for v in clone_verify.get('advancedPointList',[]) if isinstance(v,dict) and v.get('instanceName') is not None}
                    clone_paths=set()
                    for cv in clone_verify.get('advancedCurveList',[]):
                        if not isinstance(cv,dict): continue
                        sp=cv.get('startPos') if isinstance(cv.get('startPos'),dict) else {}
                        ep=cv.get('endPos') if isinstance(cv.get('endPos'),dict) else {}
                        a=sp.get('instanceName'); b=ep.get('instanceName')
                        if a is not None and b is not None: clone_paths.add((str(a),str(b)))
                    cm=sorted(expected_nodes-clone_nodes); cp=sorted(expected_paths-clone_paths)
                    clone_roundtrip=backup_dir/f'{clone_name}_4011_roundtrip.smap'
                    clone_roundtrip.write_text(json.dumps(clone_verify,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
                    if cm or cp:
                        raise ValueError(f'4010 새 mapName 업로드도 저장되지 않음 · missing nodes={cm[:8]} paths={cp[:8]} · roundtrip={clone_roundtrip}')
                    map_name=clone_name
                    smap=clone_smap
                    raw_verify=clone_verify
                    actual_nodes=clone_nodes; actual_paths=clone_paths
                    verify_file=clone_roundtrip
                    self.events.put(('map_push_stage',generation,f'새 mapName 저장 검증 OK · {clone_name} · Point={len(actual_nodes)} · Path={len(actual_paths)}'))
                verified=load_smap_data(raw_verify,map_name)
                self.events.put(('map_push_stage',generation,
                    f'4011 저장 검증 OK · Point={len(actual_nodes)} · Path={len(actual_paths)} · roundtrip={verify_file.name}'))
                load_ack=None
                station_check=None
                if apply_now:
                    self.events.put(('map_push_stage',generation,f'2022 현재맵 재적용 · {map_name}'))
                    load_ack=client.request(dict(port=19205,request_type=2022,response_type=12022,payload={'map_name':map_name},accept_any_response=True))
                    self._require_robot_ok(load_ack,'2022 Load Map')
                    self.events.put(('map_push_stage',generation,f'2022 ACK OK · {load_ack}'))
                    time.sleep(0.8)
                    try:
                        station_check=client.query(1301,{})
                        self._require_robot_ok(station_check,'1301 Station Query')
                        st=station_check.get('stations',[]) if isinstance(station_check,dict) else []
                        station_ids={str(v.get('id')) for v in st if isinstance(v,dict) and v.get('id') is not None}
                        active_missing=sorted(expected_nodes-station_ids)
                        if active_missing:
                            raise ValueError(f'2022 후 1301 인식 검증 실패 · 활성 맵에 없는 Point={active_missing[:8]}')
                        self.events.put(('map_push_stage',generation,f'1301 활성맵 검증 OK · Station={len(station_ids)}'))
                    except Exception as e:
                        raise ValueError('맵 파일 저장은 성공했지만 현재 AMR 적용 검증 실패 · '+str(e))
                elapsed=(time.monotonic()-started)*1000.0
                self.events.put(('map_push_done',generation,(map_name,verify,backup,ack,load_ack,elapsed)))
            except Exception as e:self.events.put(('map_push_error',generation,str(e)))
        threading.Thread(target=work,daemon=True,name='seer-map-push').start()

    def load_map(self):
        def load():
            if not self.real and (self.task_running or self.sim.route):raise ValueError('주행 종료 후 맵을 변경하세요.')
            path = filedialog.askopenfilename(filetypes=[('Console map','*.json')])
            if not path:return
            model = MapModel.load(path)
            self.map,self.sim = model,Simulator(model)
            self.tasks=[]; self.task_running=False; self.pending_link=None
            self.scan_points=[]; self.last_scan=0
            self.map_path = Path(path)
            self.refresh_tasks(); self.refresh_nodes(); self.fit_map()
            self.log('INFO','맵 로드: '+path)
        self.guarded(load)

    def save_map(self):
        if hasattr(self.map,'smap_source'):
            path=filedialog.asksaveasfilename(defaultextension='.smap',initialfile=(self.current_robot_map or 'edited_map')+'.smap',filetypes=[('SEER SMAP','*.smap'),('JSON','*.json')])
            if path:
                def save_smap():Path(path).write_text(json.dumps(build_smap_from_model(self.map),ensure_ascii=False,indent=2),encoding='utf-8')
                self.guarded(save_smap)
            return
        path = filedialog.asksaveasfilename(defaultextension='.json',initialfile='floor_map.json',filetypes=[('Console map','*.json')])
        if path:self.guarded(lambda:self.map.save(path))

    def save_scan(self):
        path = filedialog.asksaveasfilename(defaultextension='.csv',initialfile='simulated_scan.csv')
        def save():
            with open(path,'w',newline='',encoding='utf-8-sig') as f:
                writer=csv.writer(f);writer.writerow(['x_m','y_m'])
                writer.writerows(points(self.live.get('laser',[])) if self.real else self.sim.cloud or self.scan_points)
        if path:self.guarded(save)

    def _directed_path_exists(self,a,b):
        records=getattr(self.map,'path_records',[]) or []
        if records:return any(rec.get('a')==a and rec.get('b')==b for rec in records)
        return any(tuple(edge) in ((a,b),(b,a)) for edge in getattr(self.map,'edges',[]))

    def quick_loop_mission(self):
        nodes=list(self.map.nodes)
        if not nodes:
            messagebox.showerror('순환 미션','현재 맵에 Point가 없습니다.',parent=self); return
        win=tk.Toplevel(self); win.title('순환 미션 만들기'); self._fit_dialog(win,1100,690,940,560); win.configure(bg=BG); win.transient(self); win.grab_set()
        self.label(win,'순환 미션 빠른 생성',16,INK,True,bg=BG).pack(anchor='w',padx=18,pady=(16,4))
        self.label(win,'정지할 목적지만 선택하세요. 중간 노드는 정지 없이 통과합니다. 우회는 제외 노드로 설정하며, 마지막 목적지는 시작 Point와 같아야 합니다.',9,MUTED,bg=BG).pack(anchor='w',padx=18,pady=(0,8))
        policy=tk.StringVar(value='최단 거리')
        excluded=tk.StringVar(value='')
        policybar=tk.Frame(win,bg=BG);policybar.pack(fill='x',padx=18,pady=(0,5))
        self.label(policybar,'경로 선택',9,INK,bg=BG).pack(side='left',padx=(0,6))
        ttk.Combobox(policybar,textvariable=policy,values=['최단 거리','예상 시간 우선','우회 (제외 노드)'],state='readonly',width=19).pack(side='left')
        self.label(policybar,'우회 제외 노드',9,INK,bg=BG).pack(side='left',padx=(15,6))
        ttk.Entry(policybar,textvariable=excluded,width=25).pack(side='left')
        self.label(policybar,'쉼표로 구분',8,MUTED,bg=BG).pack(side='left',padx=5)
        summary=tk.StringVar(value='실제 통과 경로: —')
        self.label(win,'',9,INK,bg=BG,textvariable=summary,wraplength=1030,justify='left',anchor='w').pack(fill='x',padx=18,pady=(0,5))
        body=tk.Frame(win,bg=BG); body.pack(fill='both',expand=True,padx=18)
        preview_panel=tk.Frame(body,bg=PANEL,width=400)
        preview_panel.pack(side='left',fill='both',expand=True,padx=(0,10))
        self.label(preview_panel,'순환 경로 미리보기',11,INK,True).pack(anchor='w',padx=10,pady=(10,4))
        self.label(preview_panel,'지도 노드 클릭: 선택 · 더블클릭: 순서에 추가\n노드의 [숫자]는 방문 순서 · 휠 확대 / 우클릭 이동',8,MUTED,justify='left').pack(fill='x',padx=10,pady=(0,5))
        lists=tk.Frame(body,bg=BG);lists.pack(side='left',fill='both',expand=True)
        body=lists
        left=tk.Frame(body,bg=PANEL); left.pack(side='left',fill='both',expand=True,padx=(0,8))
        mid=tk.Frame(body,bg=BG,width=90); mid.pack(side='left',fill='y',padx=4)
        right=tk.Frame(body,bg=PANEL); right.pack(side='left',fill='both',expand=True,padx=(8,0))
        self.label(left,'사용 가능한 Point',11,INK,True).pack(anchor='w',padx=10,pady=(10,5))
        avail=tk.Listbox(left,bg='#ffffff',fg=INK,selectbackground='#2376cf',relief='flat',exportselection=False,width=12)
        for n in nodes: avail.insert('end',n)
        avail.pack(fill='both',expand=True,padx=10,pady=(0,10))
        self.label(right,'정지할 목적지 순서',11,INK,True).pack(anchor='w',padx=10,pady=(10,5))
        route=tk.Listbox(right,bg='#ffffff',fg=INK,selectbackground='#2376cf',relief='flat',exportselection=False,width=12)
        route.pack(fill='both',expand=True,padx=10,pady=(0,10))
        def vals(): return list(route.get(0,'end'))
        cache={}
        def planned_route():
            excluded_nodes=[key.strip() for key in excluded.get().replace('，',',').split(',') if key.strip()] if policy.get()=='우회 (제외 노드)' else []
            key=(tuple(vals()),policy.get(),tuple(excluded_nodes))
            if cache.get('key')!=key:
                cache['key']=key
                cache['plan']=plan_stops(self.map,vals(),'time' if policy.get()=='예상 시간 우선' else 'distance',excluded_nodes)
                plan=cache['plan']
                summary.set('경로 확인: '+plan['errors'][0] if plan['errors'] else '실제 통과 경로: '+(' → '.join(plan['nodes']) or '—'))
            return cache['plan']
        def add_node(event=None):
            sel=avail.curselection()
            if sel: route.insert('end',avail.get(sel[0]))
            preview.redraw()
        def remove_node():
            sel=route.curselection()
            if sel: route.delete(sel[0])
            preview.redraw()
        def move(delta):
            sel=route.curselection()
            if not sel:return
            i=sel[0]; j=i+delta
            if j<0 or j>=route.size():return
            v=route.get(i); route.delete(i); route.insert(j,v); route.selection_set(j)
            preview.redraw()
        def close_loop():
            v=vals()
            if v and (len(v)==1 or v[-1]!=v[0]): route.insert('end',v[0])
            preview.redraw()
        def selected_point():
            sel=route.curselection()
            if sel:return route.get(sel[0])
            sel=avail.curselection()
            return avail.get(sel[0]) if sel else ''
        def select_point(key):
            route.selection_clear(0,'end')
            avail.selection_clear(0,'end')
            index=nodes.index(key)
            avail.selection_set(index);avail.see(index)
        def add_point(key):
            route.insert('end',key);route.see('end');preview.redraw()
        def toggle_excluded():
            key=selected_point()
            if not key:return
            values=[value.strip() for value in excluded.get().split(',') if value.strip()]
            if key in values:values.remove(key)
            else:values.append(key)
            policy.set('우회 (제외 노드)');excluded.set(', '.join(values));preview.redraw()
        self.button(policybar,'선택 Point 제외 / 해제',toggle_excluded).pack(side='left',padx=6)
        def preview_pose():
            if not self.connected:return {}
            state=dict(self.current_state())
            if self.real and all(type(state.get(k)) in (int,float) and math.isfinite(state[k]) for k in ('x','y')):
                state['x'],state['y']=self._lidar_map_xy(state['x'],state['y'])
            return state
        preview=MissionMapPreview(preview_panel,self.map,vals,selected_point,preview_pose,select_point,add_point,self._directed_path_exists,get_plan=planned_route,width=390,height=300)
        preview.pack(fill='both',expand=True,padx=8,pady=(0,5))
        self.button(preview_panel,'지도 맞춤',preview.fit).pack(anchor='w',padx=8,pady=(0,8))
        win.preview=preview;win.route_list=route;win.available_list=avail
        win.route_policy=policy;win.excluded_nodes=excluded;win.planned_route=planned_route
        policy.trace_add('write',lambda *args:preview.redraw())
        excluded.trace_add('write',lambda *args:preview.redraw())
        def list_selected(event):
            (route if event.widget is avail else avail).selection_clear(0,'end')
            preview.redraw()
        avail.bind('<<ListboxSelect>>',list_selected)
        route.bind('<<ListboxSelect>>',list_selected)
        self.button(mid,'추가 →',add_node,BLUE).pack(fill='x',pady=(85,5))
        self.button(mid,'← 제거',remove_node).pack(fill='x',pady=5)
        self.button(mid,'위로',lambda:move(-1)).pack(fill='x',pady=(28,5))
        self.button(mid,'아래로',lambda:move(1)).pack(fill='x',pady=5)
        self.button(mid,'시작점으로 닫기',close_loop,GREEN).pack(fill='x',pady=(28,5))
        avail.bind('<Double-Button-1>',add_node)
        if self.selected in self.map.nodes: route.insert('end',self.selected)
        preview.redraw()
        opts=tk.Frame(win,bg=BG); opts.pack(fill='x',padx=18,pady=(10,4))
        self.label(opts,'반복 횟수',9,INK,bg=BG).pack(side='left')
        rep=tk.StringVar(value='0' if self.tc_loop.get() else self.tc_repeat_count.get())
        tk.Spinbox(opts,from_=0,to=999,textvariable=rep,width=6,bg=PANEL,fg=INK,insertbackground=INK,buttonbackground='#46515e').pack(side='left',padx=(5,14))
        self.label(opts,'0 = 무한 반복',9,MUTED,bg=BG).pack(side='left')
        self.label(opts,'도착 후 대기(ms)',9,INK,bg=BG).pack(side='left',padx=(28,5))
        dwell=tk.StringVar(value='0')
        tk.Spinbox(opts,from_=0,to=600000,increment=500,textvariable=dwell,width=9,bg=PANEL,fg=INK,insertbackground=INK,buttonbackground='#46515e').pack(side='left')
        start_move=tk.BooleanVar(value=True)
        tk.Checkbutton(opts,text='필요 시 시작 Point로 먼저 이동',variable=start_move,bg=BG,fg=INK,selectcolor=PANEL,activebackground=BG,activeforeground=INK).pack(side='right')
        info=self.label(win,'Point를 순서대로 더블클릭해 추가하세요.',9,MUTED,bg=BG,anchor='w'); info.pack(fill='x',padx=18,pady=4)
        buttons=tk.Frame(win,bg=BG); buttons.pack(fill='x',padx=18,pady=(6,16))
        self.button(buttons,'취소',win.destroy).pack(side='right',padx=(6,0))
        def create():
            v=vals()
            if len(v)<3:
                info.config(text='최소 3개 항목(시작 → 경유/목적 → 시작)이 필요합니다.',fg=RED); return
            if v[0]!=v[-1]:
                info.config(text='마지막 Point가 시작 Point와 같아야 합니다. [시작점으로 닫기]를 누르세요.',fg=RED); return
            plan=planned_route()
            if plan['errors']:
                info.config(text=plan['errors'][0],fg=RED); return
            try:
                repeat=int(rep.get()); wait=max(0,int(dwell.get()))
                if repeat<0: raise ValueError
            except Exception:
                info.config(text='반복 횟수/대기시간 값을 확인하세요.',fg=RED); return
            actions=plan_actions(plan,wait)
            if not actions:
                info.config(text='서로 다른 방문 Point를 추가하세요.',fg=RED);return
            chain={'name':f'Loop {v[0]} ({"∞" if repeat==0 else str(repeat)+"회"})',
                   'tasks':[{'checked':True,'groups':[{'checked':True,'actions':actions}]}],
                   'loop_route':plan['nodes'],'loop_stops':v,'routing_policy':policy.get(),'routing_format':2,'excluded_nodes':plan['excluded'],
                   'planned_distance_m':plan['distance'],'estimated_drive_seconds':plan['seconds'],
                   'repeat_count':repeat,'dwell_ms':wait,'move_to_start':bool(start_move.get())}
            self.task_chains.append(chain); self.active_chain=len(self.task_chains)-1; self.active_task=0; self.active_group=0
            self.tc_loop.set(repeat==0)
            if repeat>0:self.tc_repeat_count.set(str(repeat))
            self.tc_refresh(); self.tc_chain_list.selection_clear(0,'end'); self.tc_chain_list.selection_set(self.active_chain); self.tc_task_listbox.selection_set(0); self.tc_group_list.selection_set(0)
            self.log('MISSION','순환 미션 생성 · '+' → '.join(plan['nodes'])+f' · {policy.get()} · repeat={repeat if repeat else "∞"} · dwell={wait}ms')
            win.destroy(); self.tabs.select(self.tasks_page)
        self.button(buttons,'순환 미션 생성',create,GREEN).pack(side='right')

    def _mission_active_chain(self):
        if self.active_chain is None or self.active_chain>=len(self.task_chains): return {}
        return self.task_chains[self.active_chain]

    def _mission_station_distance(self,goal):
        st=self.robot_stations.get(goal) or self.map.nodes.get(goal)
        if not isinstance(st,dict):return None
        x=self._finite_value(st.get('x')); y=self._finite_value(st.get('y'))
        cur=self.current_state(); rx=self._finite_value(cur.get('x')); ry=self._finite_value(cur.get('y'))
        if None in (x,y,rx,ry):return None
        return math.hypot(x-rx,y-ry)

    def _mission_send_current_real_goal(self):
        if not self.task_running or self.task_index is None or not (0<=self.task_index<len(self.tasks)):return
        goal=self.tasks[self.task_index]['goal']
        if goal not in self.robot_stations:
            self.log('MISSION',f'중단 · 로봇 Station에 {goal} 없음'); self.cancel_tasks(); return
        station=self.robot_stations[goal]; payload=self._station_nav_payload(station)
        self.tasks[self.task_index]['status']='실행 중'
        self.target.set(goal); self._select_node(goal)
        self.mission_seen_active=False; self.mission_goal_sent_at=time.monotonic(); self.mission_wait_until=0.0; self.mission_pending_send=False
        self.command_status.config(text=f'순환 미션 · 3051 → {goal}',fg=ORANGE)
        self.log('MISSION',f'Cycle {self.mission_repeat_done+1} · Step {self.task_index+1}/{len(self.tasks)} · 3051 → {goal}')
        self.send_command('navigate',payload); self.refresh_tasks()

    def _mission_advance_real(self,now):
        if not self.task_running or self.task_index is None:return
        if self.mission_wait_until:
            if now<self.mission_wait_until:return
            self.mission_wait_until=0.0; self.mission_pending_send=True
        if self.mission_pending_send:
            self._mission_send_current_real_goal(); return
        api1020=self.raw.get('api_1020',{}) if isinstance(self.raw,dict) and isinstance(self.raw.get('api_1020'),dict) else {}
        status=self.live.get('task_status')
        if status is None:
            status=api1020.get('task_status',api1020.get('nav_status',api1020.get('navigation_status')))
        task_name=str(self.live.get('task','')).upper()
        if task_name in ('','UNKNOWN') and isinstance(status,int) and 0<=status<=6:
            task_name=('NONE','WAITING','RUNNING','SUSPENDED','COMPLETED','FAILED','CANCELED')[status]
        if status in (1,2,3) or task_name in ('WAITING','RUNNING','SUSPENDED'):
            self.mission_seen_active=True; return
        if status in (5,6) or task_name in ('FAILED','CANCELED'):
            self.log('MISSION',f'중단 · {self.tasks[self.task_index]["goal"]} task={task_name or status}'); self.cancel_tasks(); return
        if not (status==4 or task_name=='COMPLETED'):return
        goal=self.tasks[self.task_index]['goal']; dist=self._mission_station_distance(goal)
        if not self.mission_seen_active:
            if now-self.mission_goal_sent_at<0.8:return
            if dist is None or dist>0.45:return
        self.tasks[self.task_index]['status']='완료'
        delay=max(0,int(self.tasks[self.task_index].get('delay_ms',0)))/1000.0
        self.log('MISSION',f'{goal} 도착 완료'+(f' · {delay:.1f}s 대기' if delay else ''))
        self.task_index+=1
        if self.task_index>=len(self.tasks):
            self.mission_repeat_done+=1; target=self.mission_repeat_target
            if target and self.mission_repeat_done>=target:
                self.task_running=False; self.task_index=None
                self.mission_status.config(text=f'미션 완료 · {self.mission_repeat_done}회',fg=GREEN)
                self.command_status.config(text=f'순환 미션 완료 · {self.mission_repeat_done}회',fg=GREEN)
                self.log('MISSION',f'순환 미션 완료 · {self.mission_repeat_done}회'); self.refresh_tasks(); return
            cycle_start=1 if self.tasks and self.tasks[0].get('setup_move') else 0
            for i,t in enumerate(self.tasks):
                t['status']='완료' if i<cycle_start else '대기'
            self.task_index=cycle_start; self.log('MISSION',f'한 바퀴 완료 · {self.mission_repeat_done}회 · 다음 사이클 시작')
        self.mission_wait_until=now+delay if delay else 0.0; self.mission_pending_send=True; self.mission_seen_active=False
        self.refresh_tasks()
        if not delay:self._mission_send_current_real_goal()

    def refresh_tasks(self):
        # Legacy flat task list is retained as the execution projection of Path Nav actions.
        if hasattr(self,'task_tree'):
            self.task_tree.delete(*self.task_tree.get_children())
            for i,t in enumerate(self.tasks):self.task_tree.insert('','end',iid=str(i),values=(i+1,t['goal'],t['status']))

    def tc_refresh(self):
        if not hasattr(self,'tc_chain_list'):return
        self.tc_chain_list.delete(0,'end')
        for c in self.task_chains:self.tc_chain_list.insert('end',c['name'])
        self.tc_task_listbox.delete(0,'end'); self.tc_group_list.delete(0,'end'); self.tc_group_actions.delete(0,'end')
        if self.active_chain is not None and self.active_chain < len(self.task_chains):
            c=self.task_chains[self.active_chain]
            for i,t in enumerate(c['tasks']):self.tc_task_listbox.insert('end',f'Task {i+1}')
            if self.active_task is not None and self.active_task < len(c['tasks']):
                t=c['tasks'][self.active_task]
                for i,g in enumerate(t['groups']):self.tc_group_list.insert('end',f'Group {i+1}')
                if self.active_group is not None and self.active_group < len(t['groups']):
                    for a in t['groups'][self.active_group]['actions']:
                        suffix=f" → {a.get('goal')}" if a.get('goal') else ''
                        self.tc_group_actions.insert('end',a['type']+suffix)
        if self.active_chain is not None and self.active_chain < len(self.task_chains):
            meta=self.task_chains[self.active_chain]
            repeat=meta.get('repeat_count')
            if isinstance(repeat,int):
                self.tc_loop.set(repeat==0)
                if repeat>0:self.tc_repeat_count.set(str(repeat))
        self._project_taskchain()

    def tc_new(self):
        name=f'TaskChain {len(self.task_chains)+1}'
        self.task_chains.append({'name':name,'tasks':[]}); self.active_chain=len(self.task_chains)-1; self.active_task=None; self.active_group=None
        if hasattr(self,'tc_chain_list'):self.tc_refresh(); self.tc_chain_list.selection_set(self.active_chain)

    def tc_delete_chain(self):
        if self.active_chain is None:return
        self.task_chains.pop(self.active_chain); self.active_chain=0 if self.task_chains else None; self.active_task=None; self.active_group=None; self.tc_refresh()

    def tc_select_chain(self):
        sel=self.tc_chain_list.curselection()
        if sel:self.active_chain=sel[0]; self.active_task=None; self.active_group=None; self.tc_refresh(); self.tc_chain_list.selection_set(self.active_chain)

    def tc_new_task(self):
        if self.active_chain is None:self.tc_new()
        c=self.task_chains[self.active_chain]; c['tasks'].append({'checked':True,'groups':[]}); self.active_task=len(c['tasks'])-1; self.active_group=None; self.tc_refresh(); self.tc_task_listbox.selection_set(self.active_task)

    def tc_delete_task(self):
        if self.active_chain is None or self.active_task is None:return
        self.task_chains[self.active_chain]['tasks'].pop(self.active_task); self.active_task=None; self.active_group=None; self.tc_refresh()

    def tc_select_task(self):
        sel=self.tc_task_listbox.curselection()
        if sel:self.active_task=sel[0]; self.active_group=None; self.tc_refresh(); self.tc_task_listbox.selection_set(self.active_task)

    def tc_add_group(self):
        if self.active_task is None:self.tc_new_task()
        t=self.task_chains[self.active_chain]['tasks'][self.active_task]; t['groups'].append({'checked':True,'actions':[]}); self.active_group=len(t['groups'])-1; self.tc_refresh(); self.tc_group_list.selection_set(self.active_group)

    def tc_delete_group(self):
        if self.active_chain is None or self.active_task is None or self.active_group is None:return
        self.task_chains[self.active_chain]['tasks'][self.active_task]['groups'].pop(self.active_group); self.active_group=None; self.tc_refresh()

    def tc_select_group(self):
        sel=self.tc_group_list.curselection()
        if sel:self.active_group=sel[0]; self.tc_refresh(); self.tc_group_list.selection_set(self.active_group)

    def tc_add_action(self):
        if self.active_group is None:self.tc_add_group()
        sel=self.tc_action_list.curselection(); typ=self.tc_action_list.get(sel[0]) if sel else 'Path Nav'
        a={'type':typ,**ACTION_DEFAULTS.get(typ,{})}
        if typ=='Path Nav':
            nodes=list(self.map.nodes)
            if not nodes:return
            win=tk.Toplevel(self); win.title('Path Nav 목적지 선택'); self._fit_dialog(win,360,180,340,180); win.configure(bg=PANEL); win.transient(self)
            goal=tk.StringVar(value=self.selected if self.selected in self.map.nodes else nodes[0])
            self.label(win,'Target Point',10,INK,True).pack(anchor='w',padx=15,pady=(15,5)); ttk.Combobox(win,textvariable=goal,values=nodes,state='readonly').pack(fill='x',padx=15)
            def ok():
                a['goal']=goal.get(); self._append_tc_action(a); win.destroy()
            self.button(win,'추가',ok,BLUE).pack(fill='x',padx=15,pady=15); return
        self._append_tc_action(a)

    def _append_tc_action(self,a):
        g=self.task_chains[self.active_chain]['tasks'][self.active_task]['groups'][self.active_group]; g['actions'].append(a); self.tc_refresh(); self.tc_group_list.selection_set(self.active_group)

    def tc_action_selected(self):
        sel=self.tc_group_actions.curselection(); self.tc_prop.delete('1.0','end')
        if not sel:return
        a=self.task_chains[self.active_chain]['tasks'][self.active_task]['groups'][self.active_group]['actions'][sel[0]]
        self.tc_prop.insert('1.0',json.dumps(a,ensure_ascii=False,indent=2))

    def tc_edit_action(self):
        sel=self.tc_group_actions.curselection()
        if not sel:return
        a=self.task_chains[self.active_chain]['tasks'][self.active_task]['groups'][self.active_group]['actions'][sel[0]]
        if hasattr(self,'studio_runner'):
            if self.studio_runner.active:
                messagebox.showinfo('Action 편집','미션 종료 후 편집하세요.',parent=self);return
            def apply(data):
                # Branch targets are validated against the complete chain at execution time.
                if data.get('type')!='Branch DI':validate_actions([data])
                a.clear();a.update(data);self.tc_refresh()
            self._studio_json_dialog('Action 속성 (시간 제한 / I/O / 로봇팔)',a,apply)
            return
        if a['type']=='Path Nav':
            nodes=list(self.map.nodes); goal=simpledialog.askstring('Path Nav','Target Point ID',initialvalue=a.get('goal',''),parent=self)
            if goal in self.map.nodes:a['goal']=goal
            delay=simpledialog.askinteger('Attributes','도착 후 대기시간 (ms)',initialvalue=int(a.get('delay_ms',0)),minvalue=0,parent=self)
            if delay is not None:a['delay_ms']=delay
        elif a['type']=='Translation':
            a['distance_m']=simpledialog.askfloat('Translation','Distance (m)',initialvalue=a.get('distance_m',1.0),parent=self) or a.get('distance_m',1.0)
            a['speed_mps']=simpledialog.askfloat('Translation','Speed (m/s)',initialvalue=a.get('speed_mps',0.1),parent=self) or a.get('speed_mps',0.1)
        elif a['type']=='Rotation':
            a['angle_deg']=simpledialog.askfloat('Rotation','Rotation Angle (deg)',initialvalue=a.get('angle_deg',90.0),parent=self) or a.get('angle_deg',90.0)
            a['speed_dps']=simpledialog.askfloat('Rotation','Angular Speed (deg/s)',initialvalue=a.get('speed_dps',30.0),parent=self) or a.get('speed_dps',30.0)
        self.tc_refresh(); self.tc_group_list.selection_set(self.active_group)

    def tc_delete_action(self):
        sel=self.tc_group_actions.curselection()
        if not sel:return
        self.task_chains[self.active_chain]['tasks'][self.active_task]['groups'][self.active_group]['actions'].pop(sel[0]); self.tc_refresh(); self.tc_group_list.selection_set(self.active_group)

    def _project_taskchain(self):
        tasks=[]
        if self.active_chain is not None and self.active_chain < len(self.task_chains):
            for t in self.task_chains[self.active_chain]['tasks']:
                for g in t['groups']:
                    for a in g['actions']:
                        if a.get('type')=='Path Nav' and a.get('goal') in self.map.nodes:tasks.append({'goal':a['goal'],'status':'대기','delay_ms':a.get('delay_ms',0)})
        self.tasks=tasks

    def add_task(self):
        self.tc_add_action()

    def remove_task(self):
        self.tc_delete_action()

    def tc_task_list(self):
        if hasattr(self,'studio_runner'):
            from .studio_core import flatten_actions
            chain=self._mission_active_chain() or {}
            actions=flatten_actions(chain)
            messagebox.showinfo('Task List','\n'.join(f"{i+1}. {a['type']} {a.get('goal',a.get('operation',''))}" for i,a in enumerate(actions)) or 'Action 없음',parent=self)
            return
        chain=self._mission_active_chain(); route=chain.get('loop_route') if isinstance(chain,dict) else None
        extra=('\n순환 경로: '+' → '.join(route)) if isinstance(route,list) and route else ''
        repeat=chain.get('repeat_count') if isinstance(chain,dict) else None
        if repeat is not None:extra+=f'\n반복: {"무한" if repeat==0 else str(repeat)+"회"}'
        messagebox.showinfo('Task List',f'현재 편집 중 Taskchain: {len(self.task_chains)}개\nPath Nav 실행 항목: {len(self.tasks)}개'+extra,parent=self)

    def tc_send_selected(self):
        self.run_tasks()

    def run_tasks(self):
        if hasattr(self,'studio_runner'):return self._studio_run_tasks()
        def run():
            self._project_taskchain()
            if self.task_running:raise ValueError('이미 미션이 실행 중입니다.')
            if not self.tasks:raise ValueError('Path Nav Action을 추가하거나 [순환 미션 만들기]를 사용하세요.')
            chain=self._mission_active_chain()
            repeat_meta=chain.get('repeat_count') if isinstance(chain,dict) else None
            if isinstance(repeat_meta,int): repeat=repeat_meta
            elif self.tc_loop.get(): repeat=0
            else: repeat=max(1,int(self.tc_repeat_count.get()))
            self.mission_repeat_target=repeat; self.mission_repeat_done=0; self.mission_seen_active=False; self.mission_wait_until=0.0; self.mission_pending_send=False
            self.mission_cycle_name=chain.get('name','Taskchain') if isinstance(chain,dict) else 'Taskchain'
            for t in self.tasks:t['status']='대기'
            if self.real:
                if not (self.connected and self.control_enabled):raise ValueError('실기 · 제어 모드로 연결하고 제어권을 확보하세요.')
                missing=[t['goal'] for t in self.tasks if t['goal'] not in self.robot_stations]
                if missing:raise ValueError('로봇 Station에 없는 Point: '+', '.join(sorted(set(missing))))
                route=chain.get('loop_route') if isinstance(chain,dict) else None
                if isinstance(route,list) and route and chain.get('move_to_start',True):
                    start_goal=route[0]; d=self._mission_station_distance(start_goal)
                    if d is not None and d>0.45:self.tasks.insert(0,{'goal':start_goal,'status':'대기','delay_ms':0,'setup_move':True})
                self.task_index=0; self.task_running=True
                self.mission_status.config(text=f'실행 중 · 1/{"∞" if repeat==0 else repeat}회',fg=GREEN)
                self.log('MISSION',f'미션 시작 · {self.mission_cycle_name} · repeat={"∞" if repeat==0 else repeat}')
                self._mission_send_current_real_goal(); return
            self.sim_required()
            if self.sim.route or self.held:raise ValueError('현재 주행을 종료하세요.')
            start=self.map.nearest(self.sim.state.x,self.sim.state.y)
            for t in self.tasks:self.map.route(start,t['goal']);start=t['goal']
            self.sim._ready(); self.task_index=0; self.sim.navigate(self.tasks[0]['goal']); self.task_running=True; self.tasks[0]['status']='실행 중'
            self.mission_status.config(text=f'SIM 실행 중 · 1/{"∞" if repeat==0 else repeat}회',fg=GREEN)
        self.guarded(run)

    def cancel_tasks(self):
        if hasattr(self,'studio_runner') and self.studio_runner.active:
            self.studio_runner.cancel();self.task_running=False;self.task_index=None
            self.mission_status.config(text='미션 정지');self.refresh_tasks();return
        if self.task_running:
            for t in self.tasks:
                if t['status'] in ('대기','실행 중'):t['status']='취소됨'
            if self.real and self.connected and self.control_enabled:
                try:self.send_command('cancel',{})
                except Exception:pass
            self.log('MISSION','미션 정지 요청')
        self.task_running=False;self.task_index=None
        self.mission_seen_active=False;self.mission_wait_until=0.0;self.mission_pending_send=False
        if hasattr(self,'mission_status'):self.mission_status.config(text='미션 정지',fg=MUTED)
        self.refresh_tasks()

    def save_tasks(self):
        path=filedialog.asksaveasfilename(defaultextension='.json',initialfile='taskchain.json')
        if path:self.guarded(lambda:Path(path).write_text(json.dumps({'taskchains':self.task_chains},ensure_ascii=False,indent=2),encoding='utf-8'))

    def load_tasks(self):
        path=filedialog.askopenfilename(filetypes=[('Taskchain JSON','*.json')])
        if not path:return
        def load():
            data=json.loads(Path(path).read_text(encoding='utf-8-sig'))
            chains=data.get('taskchains')
            if not isinstance(chains,list):raise ValueError('Taskchain JSON 형식이 아닙니다.')
            self.task_chains=chains; self.active_chain=0 if chains else None; self.active_task=None; self.active_group=None; self.tc_refresh()
        self.guarded(load)

    def current_state(self):
        return self.live if self.real else self.sim.status()

    def _update_location_display(self, state, now):
        raw=self.raw if isinstance(self.raw,dict) else {}
        sources=[raw.get(k,{}) for k in ('api1020','all2_1101','all_1100')]
        # Snapshot response names vary by profile; include available flat responses.
        sources.extend(v for v in raw.values() if isinstance(v,dict))
        tracked=dict(state)
        if self.real and all(type(state.get(k)) in (int,float) and math.isfinite(state[k]) for k in ('x','y')):
            tracked['x'],tracked['y']=self._lidar_map_xy(state['x'],state['y'])
        valid=self.connected and (not self.real or now-self.last_state<=3.) and (self.real or self.sim_powered)
        context=(self.real,self.host.get() if self.real else 'SIM',self.generation,id(self.map),self.current_robot_map)
        if getattr(self,'location_pose_context',None)!=context:
            self.location_pose_context=context
            self.location_last_pose={}
        if valid and all(type(state.get(k)) in (int,float) and math.isfinite(state[k]) for k in ('x','y','theta')):
            self.location_last_pose=dict(state)
        location=self.location_tracker.update(self.map.nodes,tracked,context=context,now=now,valid=valid,
                                              real=self.real,sources=sources,route=self.sim.route if not self.real else ())
        self.location_display=location
        last=location['last'] or '—';nxt=location['next'] or '—'
        next_label='목표' if location['next_is_goal'] else '다음'
        self.location_nodes.configure(text=f'직전 {last}  →  {next_label} {nxt}')
        display_state=state if valid else self.location_last_pose
        def formatted(key):
            value=display_state.get(key)
            return f'{value:.2f}' if type(value) in (int,float) and math.isfinite(value) else '—'
        theta=display_state.get('theta')
        heading=f'{math.degrees(theta):.1f}°' if type(theta) in (int,float) and math.isfinite(theta) else '—'
        self.location_coordinates.configure(text=f"X {formatted('x')} m  Y {formatted('y')} m  θ {heading}")
        if not valid:
            detail='연결 끊김 · 마지막 수신 위치' if not self.connected else '위치 갱신 대기 · 마지막 수신 위치'
        else:
            detail=('현재 '+location['at']) if location['at'] else '노드 사이 이동 중' if location['next'] else '현재 좌표 유지'
            if location['goal']:detail+=' · 최종 목표 '+location['goal']
            if location['distance'] is not None:detail+=f"\n{next_label} 노드까지 직선거리 {location['distance']:.2f} m"
            if self.real:detail+='\n'+('좌표 기반 도착 추정' if location['estimated'] else '로봇 보고 노드')+f' · 수신 {max(0.,now-self.last_state):.1f}초 전'
        self.location_detail.configure(text=detail)
        return f'{last} → {nxt}'

    def save_snapshot(self):
        path=filedialog.asksaveasfilename(defaultextension='.json',initialfile='state.json')
        data=dict(source='REAL' if self.real else 'SIMULATION',connected=self.connected,
                  timestamp=datetime.now().isoformat(),state=self.current_state(),raw=self.raw)
        if path:self.guarded(lambda:Path(path).write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8'))

    def export_logs(self):
        path=filedialog.asksaveasfilename(defaultextension='.csv',initialfile='events.csv')
        def save():
            with open(path,'w',encoding='utf-8-sig',newline='') as f:
                writer=csv.writer(f);writer.writerow(['timestamp','level','message']);writer.writerows(self.logs)
        if path:self.guarded(save)

    def _sim_obstacle_change(self):
        self.studio_config['sim_obstacle_policy']=self._obstacle_policy_key()
        self.sim.obstacle_policy=self.studio_config['sim_obstacle_policy'];self.sim._avoid_next=0.
        self._studio_save_settings()

    def tick(self):
        now=time.monotonic();dt=now-self.last_tick;self.last_tick=now
        self.sim.prefer_graph_routes=self.prefer_graph_routes.get()
        self._auto_apply_settings()
        self.auto_class_label.configure(text='자동 판단: '+self.sim.auto_obstacle_status)
        self.sim.obstacle_policy=self._obstacle_policy_key()
        self.sim.reroute_wait_s=float(self.studio_config.get('reroute_wait_s',5))
        self.sim.reroute_attempt_limit=int(self.studio_config.get('reroute_attempts',3))
        self.map.dynamic_paused=self.dynamic_paused.get()
        body=getattr(getattr(self,'world3d',None),'body_asset',None)
        cfg=self.map.robot_model
        visual_radius=math.hypot(.9582,.6314)/2+.03 if body and body.name.startswith('SEER AMB-CSW04-CE') else math.hypot(cfg['length'],cfg['width'])/2
        self.sim.collision_radius=max(cfg['radius'],visual_radius)
        self.sim_avoidance_label.configure(text=(self.sim.avoidance_status or '주행 준비')+f' · 충돌 반경 {self.sim.collision_radius:.2f} m')
        records=self.sim.obstacle_tracker.records()
        self.tracking_label.configure(text='신규 추적: '+str(sum(r['state']=='추적 중' for r in records))+' / 미관측: '+str(sum(r['state']=='미관측' for r in records)))
        goals=list(self.sim.skipped_goals)
        self.skipped_label.configure(text='패스 목적지: '+(', '.join(goals[:6])+(' …' if len(goals)>6 else '') if goals else '없음'))
        while True:
            try:kind,generation,payload=self.events.get_nowait()
            except queue.Empty:break
            if kind=='probe':
                self.diagnostic.config(text='\n'.join(f'{port}: {status}'+(f' ({ms} ms)' if ms is not None else '') for port,status,ms in payload))
                self.log('INFO','TCP 진단 완료');continue
            if generation!=self.generation:continue
            if kind=='state':
                new_live,new_raw,self.last_state=payload
                # Preserve slower per-API RoboShop operation telemetry between 1100 snapshots.
                old_extra={k:v for k,v in self.raw.items() if isinstance(k,str) and k.startswith('api_')} if isinstance(self.raw,dict) else {}
                self.live,self.raw=new_live,new_raw
                if isinstance(self.raw,dict): self.raw.update(old_extra)
                self.connected=True
                self.connection_text.config(text='실기 연결됨 · '+('제어 활성' if self.control_enabled else '조회 전용'))
                self._startup_pose_recovery_check()
                self._maybe_auto_confirm_recovery()
            elif kind=='maps':self.show_robot_maps(payload[1])
            elif kind=='maps_for_pull':
                data=payload[1];name=data.get('current_map')
                self.current_robot_map=str(name or '')
                try:self._queue_map_download(name)
                except (ValueError,KeyError,TypeError) as e:self.log('ERROR',str(e))
            elif kind=='fast_map_downloaded':
                map_name,data,t1300,t4011,network_total=payload
                self.current_robot_map=map_name
                self.log('INFO',f'FAST PULL 수신 완료 · map={map_name} · 1300={t1300:.0f} ms · 4011={t4011:.0f} ms · network={network_total:.0f} ms')
                self.connection_text.config(text='Fast Pull · 수신 완료 · 지도 파싱 중…')
                try:self._prepare_robot_map_async(map_name,data)
                except Exception as e:
                    self.downloading_map=False
                    self.log('ERROR','Fast Pull 파싱 시작 실패: '+str(e))
            elif kind=='fast_map_error':
                self.downloading_map=False
                self.connection_text.config(text='실기 연결됨 · Fast Pull 실패')
                self.log('ERROR','FAST PULL 실패: '+payload)
            elif kind=='download_map':
                # 4011 network receive is complete. Keep Tk responsive while parsing/saving the large SMAP.
                try:self._prepare_robot_map_async(payload[2]['map_name'],payload[1])
                except Exception as e:
                    self.downloading_map=False
                    self.log('ERROR','지도 파싱 작업 시작 실패: '+str(e))
                    self.connection_text.config(text='실기 연결됨 · 지도 표시 실패')
            elif kind=='map_prepared':
                token,map_name,model,target,elapsed=payload
                if token!=self._map_prepare_token:continue
                self.downloading_map=False
                try:self._install_prepared_robot_map(map_name,model,target,elapsed)
                except Exception as e:
                    self.log('ERROR','지도 화면 적용 실패: '+str(e))
                    self.connection_text.config(text='실기 연결됨 · 지도 표시 실패')
            elif kind=='map_prepare_error':
                token,error=payload
                if token!=self._map_prepare_token:continue
                self.downloading_map=False
                self.log('ERROR','지도 다운로드 응답 파싱 실패: '+error)
                self.connection_text.config(text='실기 연결됨 · 지도 표시 실패')
            elif kind=='map_loaded':
                self.loading_map=False
                self.map.name=payload+' (로봇 지도 로드됨)'
                self.current_robot_map=payload
                self.log('INFO','로봇 지도 로드 확인: '+payload+' · 배경 다운로드 요청')
                try:self._queue_map_download(payload)
                except ValueError as e:self.log('ERROR',str(e))
            elif kind=='stations':
                stations=payload[1].get('stations',[])
                try:
                    # Keep the COMPLETE 1301 station object.  v0.4.8 only kept
                    # id/x/y/type, which silently discarded heading r, spin and
                    # destination-specific navigation options configured in RoboShop.
                    merged={}
                    smap_nodes=dict(getattr(self.map,'nodes',{}))
                    for raw in stations:
                        key=str(raw['id'])
                        base=dict(smap_nodes.get(key,{}))
                        base.update(dict(raw))  # 1301 is authoritative for live station settings
                        base.update(id=key,x=number(raw['x']),y=number(raw['y']),
                                    kind='dock' if raw.get('type')=='ChargePoint' else base.get('kind','station'),
                                    station_raw=dict(raw))
                        r=self._finite_value(raw.get('r',base.get('r')))
                        if r is not None: base['r']=r
                        if 'spin' in raw: base['spin']=self._bool_value(raw.get('spin'))
                        merged[key]=base
                    self.robot_stations=merged
                    if hasattr(self.map,'smap_source'):
                        # Do not overwrite a local map-edit session with periodic 1301 data.
                        # Before edits, 1301 remains authoritative for live station settings.
                        if not self.map_dirty:
                            local_only={k:v for k,v in self.map.nodes.items() if v.get('draft')}
                            self.map.nodes=dict(self.robot_stations); self.map.nodes.update(local_only)
                    else:
                        self.map.nodes=dict(self.robot_stations); self.map.edges=[]
                    self.refresh_nodes();self.fit_map()
                    angled=sum(1 for n in self.robot_stations.values() if self._finite_value(n.get('r',n.get('angle'))) is not None)
                    spun=sum(1 for n in self.robot_stations.values() if self._bool_value(n.get('spin')) is True)
                    self.log('INFO',f'로봇 노드 {len(stations)}개 수신 · 방향각 {angled}개 · spin 활성 {spun}개 · Station 전체 옵션 보존/3051 반영')
                except (ValueError,KeyError,TypeError) as e:self.log('ERROR',str(e))
            elif kind=='map_push_stage':
                self.log('MAP_PUSH',str(payload)); self.command_status.config(text=str(payload),fg=ORANGE)
            elif kind=='map_push_done':
                map_name,verify,backup,ack,load_ack,elapsed=payload
                self.map_push_in_progress=False;self.map_dirty=False
                self.log('MAP_PUSH',f'AMR 업데이트/검증 완료 · {map_name} · {elapsed:.0f} ms · backup={backup}')
                if load_ack is not None:self.log('MAP_PUSH','2022 재적용 ACK · '+str(load_ack))
                self.connection_text.config(text='실기 연결됨 · 맵 업데이트 완료')
                self.command_status.config(text=f'MAP PUSH OK · {map_name} · {elapsed:.0f} ms',fg=GREEN)
                self._prepare_robot_map_async(map_name,verify)
                messagebox.showinfo('AMR 업데이트',f'맵 업데이트 완료\n{map_name}\nPoint {len(self.map.nodes)}개 / Path {len(getattr(self.map,"path_records",[]))}개\n백업: {backup}',parent=self)
            elif kind=='map_push_error':
                self.map_push_in_progress=False
                self.connection_text.config(text='실기 연결됨 · 맵 업데이트 실패')
                self.log('ERROR','MAP PUSH 실패: '+str(payload)); self.command_status.config(text='MAP PUSH 실패 · '+str(payload),fg=RED)
                messagebox.showerror('AMR 업데이트 실패',str(payload),parent=self)
            elif kind=='command_io':
                direction,command,port,api,request_payload,response,elapsed=payload
                if direction=='TX':
                    msg=f'TX {command} · port={port} api={api} payload={request_payload or {}}'
                    self.log('WIRE',msg)
                    self.command_status.config(text=msg,fg=ORANGE)
                elif direction=='RX':
                    msg=f'RX {command} · api={api} · {elapsed:.0f} ms · {response}'
                    self.log('WIRE',msg)
                    self.command_status.config(text=msg,fg=GREEN)
                else:
                    msg=f'{direction} · api={api} · {response}'
                    self.log('WIRE',msg)
                    self.command_status.config(text=msg,fg=RED)
            elif kind=='nav_probe':
                self.log('NAV','3051 직후 1020 Task 상태: '+str(payload))
            elif kind=='jog_io':
                direction,request_payload,detail=payload
                now_report=time.monotonic()
                if direction in ('ERROR','ERROR_STOP'):
                    self.pad_enabled.set(False);self._pad_stop();self.pad_gate.reset()
                    self.studio_motion_error=str(detail)
                    if '40020' in str(detail) or 'preempted' in str(detail):
                        msg='JOG 거부 · 제어권 선점 상태(40020) · [제어권 가져오기 (4005)]를 먼저 누르세요'
                        self.log('CONTROL',msg+' · 원문: '+str(detail))
                    else:
                        msg=f'JOG {direction} · {detail}'
                        self.log('ERROR',msg)
                    self.command_status.config(text=msg,fg=RED)
                elif direction=='STOP':
                    result,elapsed=detail
                    msg=f'JOG STOP RX · 2000 · {elapsed:.0f} ms · {result}'
                    self.log('WIRE',msg); self.command_status.config(text=msg,fg=GREEN)
                elif direction=='RX' and now_report-self._jog_last_report>.8:
                    result,elapsed=detail
                    msg=f'JOG RX · 2010 · {elapsed:.0f} ms · {result}'
                    self.log('WIRE',msg); self.command_status.config(text=msg,fg=GREEN)
                    self._jog_last_report=now_report
                elif direction=='TX' and now_report-self._jog_last_report>.8:
                    self.log('WIRE',f'JOG TX · 2010 · {request_payload}')
            elif kind=='operation_info':
                if isinstance(payload,dict):
                    if not isinstance(self.raw,dict): self.raw={}
                    for api,data in payload.items():
                        if isinstance(data,dict): self.raw[f'api_{api}']=data
                    # Promote authoritative battery fields for the rest of the UI.
                    bat=payload.get(1007,{})
                    if isinstance(bat,dict):
                        if isinstance(bat.get('battery_level'),(int,float)): self.live['battery']=float(bat['battery_level'])*100.0
                        if 'charging' in bat: self.live['charging']=bat.get('charging')
                self.connected=True
            elif kind=='operation_api_error':
                api,msg=payload
                self.log('API',f'운영정보 API {api} 미지원/조회 실패 · {msg}')
            elif kind=='sensor1101':
                if isinstance(payload,dict):
                    self.last_sensor1101_rx=time.monotonic()
                    if isinstance(self.raw,dict): self.raw['all2_1101']=payload
                    # Merge the high-rate fields without discarding slower 1100 telemetry.
                    for src,dst in (('x','x'),('y','y'),('angle','theta'),('vx','speed'),
                                    ('blocked','blocked'),('block_reason','block_reason'),
                                    ('block_x','block_x'),('block_y','block_y'),
                                    ('emergency','emergency'),('task_status','task_status'),
                                    ('path','path'),('area_ids','area_ids')):
                        if src in payload:
                            self.live[dst]=payload.get(src)
                    pose=(self.live.get('x'),self.live.get('y'),self.live.get('theta'))
                    valid_pose=pose if all(type(v) in (int,float) for v in pose) else None
                    scans=extract_laser_scans(payload,valid_pose,self.lidar_mount_overrides)
                    official_world=bool(scans and scans[0].get('official_world'))
                    # Do not auto-fit LiDAR to map. RoboShop uses each sensor's configured
                    # install_info; map-fitting can visibly distort a dual-LiDAR merge.
                    # If install_info is unavailable, scans stay at chassis origin rather
                    # than inventing a transform.
                    pass
                    if self.real_mapping_active and time.monotonic()-self._mapping_laser_diag_at>3.0:
                        self._mapping_laser_diag_at=time.monotonic()
                        diag=[]
                        for i,sc in enumerate(scans):
                            stt=sc.get('stats') or {}
                            diag.append(f'L{i}: raw={stt.get("raw","?")} hit={stt.get("hits",len(sc.get("points") or []))} drop={stt.get("dropped","?")} min={stt.get("min")} max={stt.get("max")} cutoff={stt.get("cutoff")}')
                        self.log('SLAM_LASER',' · '.join(diag) if diag else 'scan 없음')
                    beams=[pt for scan in scans for pt in scan.get('points',[])]
                    source=' + '.join(scan.get('source','?') for scan in scans[:4])
                    if beams:
                        self.real_laser_scans=scans
                        self.real_laser_points=beams
                        self.last_laser_rx=time.monotonic()
                        self.live['laser']=beams
                        mounts=[scan.get('mount') for scan in scans]
                        self.lidar_source=('1101.laser_beams(WORLD)' if official_world else '1101:'+source)
                        self.lidar_raw_count=len(beams)
                        if not getattr(self,'_laser_mount_logged',False):
                            self._laser_mount_logged=True
                            infos=[]
                            for li,lv in enumerate(payload.get('lasers',[]) if isinstance(payload.get('lasers'),list) else []):
                                if isinstance(lv,dict): infos.append({'i':li,'install_info':lv.get('install_info'),'device_info':lv.get('device_info')})
                            self.log('LASER',f'듀얼 LiDAR MERGE · scans={len(scans)} · mounts={mounts} · info={infos}')
                    self.block_status={k:payload.get(k) for k in ('blocked','block_reason','block_x','block_y','block_di','block_ultrasonic_id') if k in payload}
                    now=time.monotonic()
                    if (not self._sensor1101_logged) or now-self.lidar_last_diag>2.0:
                        self._sensor1101_logged=True; self.lidar_last_diag=now
                        keys=list(payload.keys())[:24]
                        
                        if not beams and 'lasers' in payload:
                            self.log('SENSOR',f'1101 lasers shape={_shape_summary(payload.get("lasers"))}')
                        self.log('SENSOR',f'1101 RX keys={keys} · source={source or "NONE"} · LiDAR={len(beams)} pts · blocked={payload.get("blocked")}')
                        if isinstance(payload.get('lasers'),list) and payload.get('lasers'):
                            samples=[]
                            for li,lv in enumerate(payload.get('lasers')[:3]):
                                if isinstance(lv,dict):
                                    bb=lv.get('beams')
                                    samples.append({'i':li,'keys':list(lv.keys())[:16],'beam0':(bb[0] if isinstance(bb,list) and bb else None)})
                            if samples: self.log('LASER_RAW','1101 laser sample='+str(samples))
            elif kind=='lidar_calibration_done':
                map_name,mounts,reports=payload
                self._lidar_calibration_running=False
                self.lidar_mount_overrides=mounts
                self._lidar_calibrated_for_map=map_name
                self._save_lidar_mounts(mounts)
                self.log('LASER',f'RoboShop 정합 자동보정 완료 · {reports} · mounts={mounts}')
            elif kind=='lidar_calibration_error':
                self._lidar_calibration_running=False
                self.log('LASER','센서 장착 자동보정 실패 · '+str(payload))
            elif kind=='sensor1101_error':
                self.log('SENSOR','1101 All2 조회 실패 · 1009 fallback 사용: '+str(payload))
            elif kind=='laser':
                pose=(self.live.get('x'),self.live.get('y'),self.live.get('theta'))
                valid_pose=pose if all(type(v) in (int,float) for v in pose) else None
                scans=extract_laser_scans(payload,valid_pose,self.lidar_mount_overrides)
                official_world=bool(scans and scans[0].get('official_world'))
                pts=[pt for scan in scans for pt in scan.get('points',[])]
                source=' + '.join(scan.get('source','?') for scan in scans[:4])
                if pts and (time.monotonic()-self.last_laser_rx > 0.8 or not self.real_laser_points):
                    self.real_laser_scans=scans
                    self.real_laser_points=pts
                    self.last_laser_rx=time.monotonic()
                    self.lidar_source=('1009.laser_beams(WORLD)' if official_world else '1009:'+source)
                    self.lidar_raw_count=len(pts)
                if isinstance(self.raw,dict): self.raw['laser1009']=payload
                now=time.monotonic()
                if (not self._laser_shape_logged) or (not self.real_laser_points and now-self.lidar_last_diag>2.0):
                    self._laser_shape_logged=True
                    
                    if isinstance(payload,dict) and not pts and 'lasers' in payload:
                        self.log('LASER',f'1009 lasers shape={_shape_summary(payload.get("lasers"))}')
                    self.log('LASER',f'1009 RX keys={list(payload.keys())[:24] if isinstance(payload,dict) else type(payload).__name__} · source={source or "NONE"} · parsed={len(pts)}')
            elif kind=='laser_error':
                self.log('LASER','1009 조회 실패: '+str(payload))
            elif kind=='lidar_map_alignment_done':
                map_name,tr,score,auto=payload
                self._lidar_alignment_running=False
                if map_name==(self.current_robot_map or self.map.name):
                    self.lidar_map_alignment=tuple(tr); self.lidar_alignment_score=float(score); self._lidar_alignment_map=map_name
                    self._save_lidar_alignment(); self.draw_map()
                    self.log('LASER',f'LiDAR↔Map 정합 완료 · dx={tr[0]:.3f} dy={tr[1]:.3f} yaw={math.degrees(tr[2]):.2f}° score={score:.3f}')
                    if not auto:messagebox.showinfo('LiDAR 정합',f'정합 완료\nscore={score:.3f}\ndx={tr[0]:.3f} m, dy={tr[1]:.3f} m, yaw={math.degrees(tr[2]):.2f}°',parent=self)
            elif kind=='lidar_map_alignment_error':
                self._lidar_alignment_running=False; self.log('LASER','LiDAR↔Map 정합 실패: '+str(payload))
            elif kind=='command':
                command,result,request_payload = payload if isinstance(payload,tuple) and len(payload)==3 else ('command',payload,None)
                self.log('ACK',str(payload)+' · 완료 여부는 Task 상태에서 확인')
                if command=='lock_control':
                    self.command_status.config(text='제어권 확보됨 · 4005 ACK · 이제 수동/목적지 명령 가능',fg=GREEN)
                    self.log('CONTROL','4005 Preempt Control ACK · 제어권 확보')
                elif command=='unlock_control':
                    self.command_status.config(text='제어권 해제됨 · 4006 ACK',fg=MUTED)
                    self.log('CONTROL','4006 Release Control ACK')
                elif command=='relocate':
                    self.command_status.config(text='2002 Reloc ACK · 재배치 계산 중 · Laser Match/Confidence 확인',fg=GREEN)
                    self.reloc_info.config(text='재배치 계산 중 · Localization Confidence와 LiDAR/맵 정합을 확인한 뒤 위치 확정(2003)을 누르세요.',fg=ORANGE)
                    self.log('RELOC',f'2002 Reloc ACK · {result}')
                elif command=='confirm_loc':
                    self.command_status.config(text='2003 ConfirmLoc ACK · 위치 확정',fg=GREEN)
                    self.reloc_info.config(text='위치 확정 완료 · 현재 Localization Confidence를 확인하세요.',fg=GREEN)
                    self.reloc_candidate=None; self.draw_map()
                    self.log('RELOC',f'2003 ConfirmLoc ACK · {result}')
                elif command=='cancel_reloc':
                    self.command_status.config(text='2004 CancelReloc ACK · 재배치 취소',fg=MUTED)
                    self.reloc_info.config(text='재배치 취소됨',fg=MUTED)
                    self.reloc_candidate=None; self.draw_map()
                    self.log('RELOC',f'2004 CancelReloc ACK · {result}')
                elif command=='slam_start':
                    # Already switched to blank mapping view at button press. Do not
                    # clear points again when ACK arrives; keep early scan data.
                    self.real_mapping_active=True
                    self.mapping_status.config(text='SLAM: SCANNING',fg=GREEN)
                    self.command_status.config(text='맵 생성 시작 · 6100 ACK · 빈 화면에 실시간 맵 누적 표시',fg=GREEN)
                    self.log('SLAM','6100 Start Map Scanning ACK · 실제 맵 생성 시작 · 장애물 Hit Point 전용 누적 모드')
                    self.draw_map()
                elif command=='slam_stop':
                    self.real_mapping_active=False
                    self.mapping_status.config(text='SLAM: 종료됨 · 지도 갱신 대기',fg=MUTED)
                    self.command_status.config(text='맵 생성 종료 · 6101 ACK · 현재 지도 재조회',fg=GREEN)
                    self.log('SLAM','6101 End SLAM ACK · 1300/4011로 생성된 지도 재조회 예정')
                    self.after(1200,self.pull_robot_map)
            elif kind=='download_map_error':
                self.downloading_map=False
                self.connection_text.config(text='실기 연결됨 · 지도 다운로드 실패')
                self.log('ERROR','지도 다운로드 실패: '+payload+' · 상태 모니터링은 계속합니다.')
            elif kind=='command_error':
                self.studio_command_error=str(payload)
                self.downloading_map=False
                # A command socket timeout must not invalidate the independent status heartbeat.
                # Never auto-retry motion/navigation commands: the robot may already have accepted them.
                if '40020' in str(payload) or 'preempted' in str(payload):
                    msg='제어 명령 거부(40020) · 다른 클라이언트가 제어권을 선점 중 · 4005 제어권 가져오기 필요'
                    self.log('CONTROL',msg+' · 원문: '+str(payload))
                    self.command_status.config(text=msg,fg=RED)
                else:
                    self.log('ERROR',payload+' · 명령 결과 불확실/실패. 상태 연결은 유지하며 자동 재전송하지 않습니다.')
                    self.command_status.config(text=payload,fg=RED)
                    if '6100' in str(payload) or '6101' in str(payload) or 'SLAM' in str(payload).upper():
                        self.real_mapping_active=False
                if self.connected:
                    self.connection_text.config(text='실기 연결됨 · 명령 응답 지연/실패 · 상태 모니터링 유지')
            elif kind=='error':
                self.downloading_map=False
                self.control_enabled=False
                self.connected=False;self.live={};self.raw={}
                self.connection_text.config(text='TCP 오류 · 연결 해제됨')
                self.log('ERROR',payload)
        if self.real and self.connected and now-self.last_state>3:
            self.connected=False
            self.control_enabled=False
            if self.worker_stop:self.worker_stop.set()
            self.connection_text.config(text='응답 지연 · 상태 유효기간 초과')
        if self.real and self.connected and self.task_running and not self.studio_runner.active:
            self._mission_advance_real(now)
            if hasattr(self,'mission_status') and self.task_running:
                total='∞' if self.mission_repeat_target==0 else str(self.mission_repeat_target)
                goal=self.tasks[self.task_index]['goal'] if self.task_index is not None and self.task_index < len(self.tasks) else '-'
                self.mission_status.config(text=f'실행 중 · {self.mission_repeat_done+1}/{total}회 · → {goal}',fg=GREEN)
        if self.connected and not self.real and self.sim_powered:
            if self.held and self.manual.get():
                try:self.sim.drive(*self.held[1])
                except ValueError:self.release_drive()
            self.sim.tick(dt)
            if self.task_running and not self.studio_runner.active and not self.sim.route and self.sim.state.task=='완료':
                self.tasks[self.task_index]['status']='완료'
                delay=max(0,int(self.tasks[self.task_index].get('delay_ms',0)))
                self.task_index+=1
                if self.task_index>=len(self.tasks):
                    self.mission_repeat_done+=1
                    if self.mission_repeat_target and self.mission_repeat_done>=self.mission_repeat_target:
                        self.task_running=False;self.task_index=None
                        self.log('MISSION',f'[SIM] 순환 미션 완료 · {self.mission_repeat_done}회')
                        if hasattr(self,'mission_status'):self.mission_status.config(text=f'SIM 완료 · {self.mission_repeat_done}회',fg=GREEN)
                    else:
                        for t in self.tasks:t['status']='대기'
                        self.task_index=0
                if self.task_running:
                    def sim_next():
                        if not self.task_running or self.task_index is None:return
                        try:
                            self.sim.navigate(self.tasks[self.task_index]['goal'])
                            self.tasks[self.task_index]['status']='실행 중'
                        except ValueError as e:
                            self.cancel_tasks();self.log('ERROR',e)
                    if delay:self.after(delay,sim_next)
                    else:sim_next()
                self.refresh_tasks()
            if now-self.last_scan>.2:
                self.scan_points=self.sim.scan();self.last_scan=now
        self._studio_tick(now,dt)
        state=self.current_state() if self.connected else {}
        node_progress=self._update_location_display(state,now)
        def fmt(key,precision=2):
            value=state.get(key)
            return f'{value:.{precision}f}' if type(value) in (int,float) and math.isfinite(value) else '—'
        values=dict(mode=str(state.get('mode','OFFLINE')),battery=fmt('battery',1)+' %',
                    pose=fmt('x')+' / '+fmt('y'),localization=fmt('localization',3),
                    safety=('STOPPED' if state.get('stopped') else 'BLOCKED' if state.get('blocked') else 'CLEAR')
                        if not self.real and self.connected else str(state.get('safety','UNKNOWN')),
                    task=str(state.get('task','—')))
        for key,value in values.items():self.cards[key].config(text=value[:27])
        self.cards['task'].configure(text=values['task'][:27]+'\n'+node_progress)
        self.badge.config(text='● '+(('REAL / CONTROL' if self.control_enabled else 'REAL / READ ONLY') if self.real else 'SIMULATION') if self.connected else '● OFFLINE',
                          fg=GREEN if self.connected else MUTED)
        self.footer.config(text=f"{'실기 조회' if self.real else '시뮬레이션'}  |  "
            f"heading {fmt('theta',3)} rad · speed {fmt('speed')} m/s  |  "
            f"{datetime.now():%H:%M:%S}  |  local map: {self.map.name}")
        self.mapping_status.config(text=(f'실시간 맵 생성중 · 누적 {len(self.real_mapping_cloud)} pts' if self.real_mapping_active and self.real else (f'DEMO 누적 {len(self.sim.cloud)} pts' if self.sim.mapping and not self.real else ('SLAM: '+str(state.get('slam_status','UNKNOWN')) if self.real else 'SLAM: DEMO'))))
        if self.tabs.select()==str(self.robot_info_page):
            text=self._robot_info_string(); self.robot_info_text.config(state='normal'); self.robot_info_text.delete('1.0','end'); self.robot_info_text.insert('end',text); self.robot_info_text.config(state='disabled')
            self.robot_info_status.config(text=('1100 + 1101 실시간 수신' if self.real and self.connected else '시뮬레이션/연결 대기'),fg=GREEN if self.real and self.connected else MUTED)
        if self.tabs.select()==str(self.telemetry_page):
            data={'source':'REAL' if self.real else 'SIMULATION','connected':self.connected,
                  'state':state,'raw':self.raw if self.real else {'synthetic_lidar_points':len(self.scan_points),'route':self.sim.route,
                  'navigation_points':self.sim.navigation_points(),'path_records':getattr(self.sim.map,'path_records',[]),
                  'display_map':self.map.name,'simulation_map':self.sim.map.name,'same_map':self.map is self.sim.map,
                  'source_version':'studio-v1.0'}}
            self.telemetry.config(state='normal');self.telemetry.delete('1.0','end')
            self.telemetry.insert('end',json.dumps(data,ensure_ascii=False,indent=2));self.telemetry.config(state='disabled')
        if self.tabs.select() in (str(self.operation_page),str(self.nodes_page)):
            interval=.25 if self.real and self.current_robot_map else .10
            if now-self._last_operation_draw>=interval:
                self.draw_map();self._last_operation_draw=now
        if self.pose_autosave.get() and now-self._last_pose_save_at>=1.0 and (self.real or self.sim_powered):
            self.save_last_pose()
        self.after(100,self.tick)

    def close(self):
        self.release_drive()
        if self.pad_after:self.after_cancel(self.pad_after)
        self._spatial_save()
        try:self.save_last_pose(force=True)
        except Exception:pass
        self.disconnect()
        if hasattr(self,'fr5_client') and self.fr5_client.connected:self._fr5_priority_stop()
        if hasattr(self,'studio_bridge'):self.studio_bridge.close()
        self.destroy()


def main():
    app=Console()
    app.mainloop()
