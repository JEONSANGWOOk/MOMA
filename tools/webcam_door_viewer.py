"""Register local webcam door references without restarting the main GUI."""
import json
import sys
import tkinter as tk
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from seer_control.webcam_doors_ui import show_webcam_doors


def main():
    root=tk.Tk();root.withdraw()
    root.world3d=SimpleNamespace(webcam_doors_enabled=tk.BooleanVar(value=True))
    def save():
        path=Path.home()/'.seer_amr_console/studio_settings.json'
        data=json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
        data.setdefault('viewer3d',{})['webcam_doors_enabled']=True
        path.parent.mkdir(exist_ok=True)
        path.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    root._spatial_save=save
    show_webcam_doors(root,on_close=root.destroy)
    root.mainloop()


if __name__=='__main__':main()
