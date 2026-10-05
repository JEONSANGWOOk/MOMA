import tkinter as tk
from tkinter import ttk,filedialog
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from seer_control.voice_gui import GuiRegistry,chosen_file
from seer_control.voice_commands import validate_command,ground_command,command_schema

class GuiTests(unittest.TestCase):
 def setUp(self):
  self.app=tk.Tk();self.app.withdraw();self.app.voice_real=tk.BooleanVar(value=False)
  self.frame=ttk.Frame(self.app);self.frame.pack();self.called=Mock();self.button=ttk.Button(self.frame,text='지도 맞춤',command=self.called);self.button.pack()
  self.choice=ttk.Combobox(self.frame,values=['2D','3D']);self.choice.pack();self.value=tk.BooleanVar(value=False);self.toggle=tk.Checkbutton(self.frame,text='LiDAR',variable=self.value);self.toggle.pack()
  tk.Checkbutton(self.frame,text='실기 허용',variable=self.app.voice_real).pack()
  self.scale=tk.Scale(self.frame,from_=10,to=100);self.scale.pack();self.app.canvas=tk.Canvas(self.frame);self.app.canvas.pack();self.app.map=SimpleNamespace(nodes={'LM1':{}})
  self.gui=GuiRegistry(self.app)
 def tearDown(self):self.app.destroy()
 def cmd(self,w,op,value=None):
  out=dict(action='gui',target=str(w),operation=op)
  if value is not None:out['value']=value
  return out
 def test_button_reuses_registered_callback(self):
  self.gui.execute(self.cmd(self.button,'invoke'));self.called.assert_called_once()
 def test_disabled_button_cannot_run(self):
  self.button.configure(state='disabled')
  with self.assertRaises(ValueError):self.gui.execute(self.cmd(self.button,'invoke'))
  self.called.assert_not_called()
 def test_choice_validation_and_change_event(self):
  changed=Mock();self.choice.bind('<<ComboboxSelected>>',lambda e:changed());self.app.update()
  self.gui.execute(self.cmd(self.choice,'set','3D'));self.app.update();self.assertEqual(self.choice.get(),'3D')
  with self.assertRaises(ValueError):self.gui.execute(self.cmd(self.choice,'set','bad'))
 def test_toggle_exact_state_is_idempotent(self):
  self.gui.execute(self.cmd(self.toggle,'set','true'));self.gui.execute(self.cmd(self.toggle,'set','true'));self.assertTrue(self.value.get())
 def test_protected_real_permission_not_registered(self):
  self.assertFalse(any('실기 허용' in r['label'] for r in self.gui.targets.values()))
 def test_forged_target_and_code_cannot_execute(self):
  for c in [dict(action='gui',target='os.system',operation='invoke'),dict(action='gui',target=str(self.button),operation='eval',value='code')]:
   with self.assertRaises(ValueError):validate_command(c,[],[],self.gui)
 def test_literal_value_must_be_in_user_request(self):
  with self.assertRaises(ValueError):ground_command(self.cmd(self.choice,'set','3D'),'2D로 설정')
 def test_slider_limits_and_finite(self):
  for value in ('999','nan','-1'):
   with self.assertRaises(ValueError):self.gui.execute(self.cmd(self.scale,'set',value))
 def test_gui_schema_only_registered_targets(self):
  schema=command_schema([],[],self.gui.public());self.assertIn(str(self.button),schema['oneOf'][-1]['properties']['target']['enum'])
 def test_closed_widget_rejected(self):
  self.button.destroy()
  with self.assertRaises(ValueError):self.gui.execute(self.cmd(self.button,'invoke'))
 def test_file_dialog_override_restores_on_error(self):
  original=filedialog.askopenfilename
  with self.assertRaises(RuntimeError):
   with chosen_file('C:/project/map.smap'):
    self.assertEqual(filedialog.askopenfilename(),'C:/project/map.smap');raise RuntimeError('test')
  self.assertIs(filedialog.askopenfilename,original)
 def test_map_unknown_node_and_nonfinite_rejected(self):
  for target,value in [('map::노드 선택','LM99'),('map::좌표 클릭','[1,NaN]')]:
   with self.assertRaises(ValueError):self.gui.validate(dict(action='gui',target=target,operation='set',value=value))

 def test_toggle_llm_requires_explicit_enable_intent(self):
  with self.assertRaises(ValueError):ground_command(self.cmd(self.toggle,'set','true'),'LiDAR 상태 알려줘')
 def test_manual_button_has_bounded_release(self):
  self.button.bind('<ButtonPress-1>',lambda e:None);self.app.manual=tk.BooleanVar(value=True);self.app.task_running=False;self.app.studio_runner=SimpleNamespace(active=False);self.app.real=False;self.app.release_drive=Mock();self.app.after=Mock()
  gui=GuiRegistry(self.app);gui.execute(self.cmd(self.button,'invoke'));delay,callback=self.app.after.call_args.args;self.assertEqual(delay,150);callback();self.app.release_drive.assert_called_once_with(force=True)

 def test_no_matching_candidates_does_not_send_entire_gui(self):self.assertEqual(self.gui.candidates("xyz_no_matching_function"),[])
