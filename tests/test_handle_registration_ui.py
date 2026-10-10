import base64,io,json,tempfile,time,unittest
from pathlib import Path
import tkinter as tk
from tkinter import ttk
from PIL import Image
from seer_control.vision_camera_ui import VisionCameraPane
from seer_control.vision_stream import publish_frame


class RegistrationTests(unittest.TestCase):
    def setUp(self):
        try:self.root=tk.Tk();self.root.withdraw()
        except tk.TclError as exc:self.skipTest(str(exc))
        self.addCleanup(self.root.destroy)
        self.directory=tempfile.TemporaryDirectory();self.addCleanup(self.directory.cleanup)
        self.pane=VisionCameraPane(self.root,Path(self.directory.name));self.addCleanup(self.pane.close)
        buf=io.BytesIO();Image.new('RGB',(200,150),'white').save(buf,format='JPEG')
        self.snapshot=dict(timestamp=time.time(),image_jpeg_base64=base64.b64encode(buf.getvalue()).decode(),
            board=dict(valid=True,markers_used=2,revision='board',reprojection_px=.1,camera_xyz_m=[0,0,.6],rotation_vector_rad=[3.14159,0,0]),
            camera=dict(matrix=[[580,0,100],[0,580,75],[0,0,1]],distortion=[0]*5,model='none'))
        publish_frame(self.pane.path,self.snapshot)
    def test_click_register_and_delete_local_target(self):
        self.pane.register_handle();dialog=next(w for w in self.pane.winfo_children() if isinstance(w,tk.Toplevel))
        dialog.withdraw();canvas=next(w for w in dialog.winfo_children() if isinstance(w,tk.Canvas))
        # Call the real Tk event binding with a selected pixel, then the real save button.
        self.root.update_idletasks();dialog.deiconify();self.root.update()
        canvas.event_generate('<Button-1>',x=100,y=75);self.root.update()
        save=next(w for w in dialog.winfo_children() if isinstance(w,ttk.Button));save.invoke()
        path=Path(self.directory.name)/'.delivery/d455_handle_target.json';saved=json.loads(path.read_text(encoding='utf-8'))
        self.assertEqual(saved['center_px'],[100,75]);self.assertEqual(saved['board']['revision'],'board')
        self.assertEqual(saved['dimension_source'],'estimated_plane');self.pane.delete_handle();self.assertFalse(path.exists())
    def test_stale_camera_does_not_open_registration(self):
        self.snapshot['timestamp']-=2;publish_frame(self.pane.path,self.snapshot);self.pane.register_handle()
        self.assertFalse(any(isinstance(w,tk.Toplevel) for w in self.pane.winfo_children()))


if __name__=='__main__':unittest.main()
