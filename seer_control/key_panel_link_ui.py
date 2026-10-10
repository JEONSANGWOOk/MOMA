"""Integrated connection controls for the cabinet SIM/REAL workspace."""
import json,time
from pathlib import Path
import tkinter as tk
from tkinter import ttk,filedialog
from .key_panel_real import template


class KeyLinkPane(ttk.LabelFrame):
    def __init__(self,parent,link,root_path,on_mode):
        super().__init__(parent,text='SIM / 실제 FR5 통합 제어',padding=6)
        self.link=link;self.root_path=Path(root_path);self.on_mode=on_mode
        self.mode=tk.StringVar(value='SIM');self.ip=tk.StringVar(value='192.168.57.2')
        self.sdk=tk.StringVar(value=str(self.root_path/'.delivery/fairino-sdk/windows'))
        self.contact=tk.BooleanVar(value=False);self.message=tk.StringVar(value='SIM 모드 · FR5 미연결')
        row=ttk.Frame(self);row.pack(fill='x')
        ttk.Label(row,text='실행 모드').pack(side='left')
        combo=ttk.Combobox(row,textvariable=self.mode,values=['SIM','REAL','SIM+REAL'],state='readonly',width=13);combo.pack(side='left',padx=4)
        combo.bind('<<ComboboxSelected>>',lambda e:self.change())
        ttk.Label(row,text='FR5 IP').pack(side='left');ttk.Entry(row,textvariable=self.ip,width=16).pack(side='left',padx=4)
        ttk.Button(row,text='연결 · 상태 읽기',command=lambda:self.perform(lambda:link.connect(self.ip.get().strip(),self.sdk.get().strip()))).pack(side='left',padx=3)
        ttk.Button(row,text='연결 해제',command=lambda:self.perform(link.disconnect)).pack(side='left',padx=3)
        ttk.Checkbutton(row,text='실기 삽입·회전 포함',variable=self.contact).pack(side='left',padx=8)
        ttk.Button(row,text='실측 보정 파일',command=lambda:self.perform(self.load)).pack(side='left',padx=3)
        ttk.Button(row,text='보정 템플릿',command=lambda:self.perform(self.create_template)).pack(side='left',padx=3)
        row=ttk.Frame(self);row.pack(fill='x',pady=3)
        ttk.Label(row,text='SDK windows 폴더').pack(side='left');ttk.Entry(row,textvariable=self.sdk,width=62).pack(side='left',padx=4)
        ttk.Button(row,text='선택',command=lambda:self.sdk.set(filedialog.askdirectory() or self.sdk.get())).pack(side='left',padx=3)
        ttk.Label(row,textvariable=self.message,wraplength=650).pack(side='left',padx=8)

    def perform(self,fn):
        try:fn()
        except Exception as exc:self.link.report(str(exc));self.message.set(str(exc))

    def change(self):
        old=self.link.mode
        try:self.link.set_mode(self.mode.get());self.on_mode(self.mode.get())
        except Exception as exc:self.mode.set(old);self.link.report(str(exc))

    def load(self):
        if self.link.client.connected or self.link.busy:raise ValueError('연결 해제 후 보정 파일을 불러오세요.')
        path=filedialog.askopenfilename(filetypes=[('JSON','*.json')])
        if path:self.link.load(json.loads(Path(path).read_text(encoding='utf-8')))

    def create_template(self):
        path=self.root_path/'.delivery/fr5_key_calibration.json'
        if not path.exists():path.write_text(json.dumps(template(),ensure_ascii=False,indent=2),encoding='utf-8')
        self.link.report('보정 템플릿: '+str(path))

    def refresh(self):
        f=self.link.feedback;fresh=self.link.client.connected and time.monotonic()-self.link.rx<=.6
        self.message.set(('REAL 최신' if fresh else 'REAL 미연결/지연')+' · '+self.link.status)
