"""Desktop/package entry point for the independent LAN virtual robot."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from seer_control.robot_emulator import main

if __name__=='__main__':
    try:main()
    except Exception as error:
        print('가상 로봇 시작 실패:',error,flush=True)
        if getattr(sys,'frozen',False):
            import tkinter as tk
            from tkinter import messagebox
            root=tk.Tk();root.withdraw()
            messagebox.showerror('LAN 가상 로봇',str(error)+'\n19204~19207 포트 사용 여부와 지도 파일을 확인하세요.',parent=root)
            root.destroy()
        sys.exit(1)
