"""Developer drag/drop recipe blocks, bound to taught operations and stations."""
import copy
import tkinter as tk
from tkinter import ttk
from .operator_missions import validate_template
from .theme import PANEL,BLUE

class RecipeMixin:
 def _recipe_build(self):
  page=ttk.Frame(self.tabs);self.recipe_page=page;self.tabs.add(page,text='운영 미션 구성');self._register_navigation(page,'운영 미션 구성','▦')
  holder,body,_=self._scrollable_frame(page,bg=PANEL);holder.pack(fill='both',expand=True)
  self._studio_note(body,'개발자: 이동·등록 팔 작업·대기 블록을 조합하세요. 목록을 드래그하거나 ↑/↓로 순서를 변경합니다. 팔 작업 미션은 실제 위치에 맞는 허용 목적지를 지정해야 사용자에게 배포할 수 있습니다.')
  self.recipe_selection=tk.StringVar(value=self.operator_templates[0]['name']);self.recipe_name=tk.StringVar();self.recipe_enabled=tk.BooleanVar();self.recipe_destination=tk.StringVar(value='모든 위치 (이동 미션)');self.recipe_blocks=[];self.recipe_drag=None
  row=self._studio_row(body);self.recipe_combo=ttk.Combobox(row,textvariable=self.recipe_selection,values=[r['name'] for r in self.operator_templates],state='readonly',width=25);self.recipe_combo.pack(side='left');self.recipe_combo.bind('<<ComboboxSelected>>',lambda e:self.guarded(self._recipe_load))
  self.button(row,'새 이름으로 복사',lambda:self.guarded(self._recipe_copy)).pack(side='left',padx=5)
  row=self._studio_row(body);ttk.Label(row,text='미션 이름').pack(side='left');ttk.Entry(row,textvariable=self.recipe_name,width=25).pack(side='left',padx=5);tk.Checkbutton(row,text='사용자 실행 허용',variable=self.recipe_enabled,bg=PANEL).pack(side='left')
  row=self._studio_row(body);ttk.Label(row,text='사용 목적지').pack(side='left');self.recipe_destination_combo=ttk.Combobox(row,textvariable=self.recipe_destination,values=['모든 위치 (이동 미션)']+list(self.map.nodes),state='readonly',width=25);self.recipe_destination_combo.pack(side='left',padx=5)
  self.recipe_list=tk.Listbox(body,height=10,exportselection=False);self.recipe_list.pack(fill='x',padx=12,pady=8)
  self.recipe_list.bind('<ButtonPress-1>',self._recipe_drag_start);self.recipe_list.bind('<ButtonRelease-1>',self._recipe_drag_end)
  row=self._studio_row(body)
  for title,fn in [('↑ 위로',lambda:self._recipe_move(-1)),('↓ 아래로',lambda:self._recipe_move(1)),('선택 삭제',self._recipe_remove)]:self.button(row,title,lambda f=fn:self.guarded(f)).pack(side='left',padx=3)
  self.recipe_block_kind=tk.StringVar(value='이동');self.recipe_block_value=tk.StringVar(value='목적지')
  row=self._studio_row(body);kind=ttk.Combobox(row,textvariable=self.recipe_block_kind,values=['이동','팔 작업','대기'],state='readonly',width=12);kind.pack(side='left');kind.bind('<<ComboboxSelected>>',lambda e:self._recipe_value_options());self.recipe_value_combo=ttk.Combobox(row,textvariable=self.recipe_block_value,width=35);self.recipe_value_combo.pack(side='left',padx=5)
  self.button(row,'블록 추가',lambda:self.guarded(self._recipe_add),BLUE).pack(side='left');self.button(row,'선택 블록 수정',lambda:self.guarded(self._recipe_change)).pack(side='left',padx=4)
  self.recipe_list.bind('<<ListboxSelect>>',self._recipe_selected)
  self._studio_note(body,'이동: 픽업 / 목적지 / 복귀 또는 고정 노드. 팔 작업: 기존 티칭 작업 또는 프로그램. 대기: 초. I/O·조건·복잡한 팔 순서는 기존 로봇팔 프로그램에서 개발한 뒤 팔 작업 블록으로 선택하세요. 반복은 운영 화면에서 지정합니다.')
  row=self._studio_row(body);self.button(row,'템플릿 저장 / 배포',lambda:self.guarded(self._recipe_save),BLUE).pack(side='left');self.button(row,'템플릿 삭제',lambda:self.guarded(self._recipe_delete)).pack(side='left',padx=5)
  self._recipe_load()
 def _recipe_load(self):
  self._require_developer();r=next(t for t in self.operator_templates if t['name']==self.recipe_selection.get());self.recipe_name.set(r['name']);self.recipe_enabled.set(r['enabled']);self.recipe_destination.set(r.get('destinations',[''])[0] if r.get('destinations') else '모든 위치 (이동 미션)');self.recipe_blocks=copy.deepcopy(r['blocks']);self._recipe_refresh();self._recipe_value_options()
 def _recipe_refresh(self):
  self.recipe_list.delete(0,'end')
  for i,b in enumerate(self.recipe_blocks):self.recipe_list.insert('end',f"{i+1}. [{b['type']}] {b['value']}")
 def _recipe_value_options(self):
  kind=self.recipe_block_kind.get();cfg=self._operator_arm_config()
  values=['픽업','목적지','복귀']+list(self.map.nodes) if kind=='이동' else list(cfg.get('operations',{}))+list(cfg.get('programs',{})) if kind=='팔 작업' else ['0','1','2','5','10']
  self.recipe_value_combo.configure(values=values,state='normal' if kind=='대기' else 'readonly')
  if self.recipe_block_value.get() not in values:self.recipe_block_value.set(values[0] if values else '')
 def _recipe_selected(self,e=None):
  if self.recipe_list.curselection():
   b=self.recipe_blocks[self.recipe_list.curselection()[0]];self.recipe_block_kind.set(b['type']);self._recipe_value_options();self.recipe_block_value.set(str(b['value']))
 def _recipe_add(self):
  self._require_developer();self.recipe_blocks.append(dict(type=self.recipe_block_kind.get(),value=self.recipe_block_value.get()));self._recipe_refresh()
 def _recipe_change(self):
  self._require_developer()
  if not self.recipe_list.curselection():raise ValueError('수정할 블록을 선택하세요.')
  index=self.recipe_list.curselection()[0];self.recipe_blocks[index]=dict(type=self.recipe_block_kind.get(),value=self.recipe_block_value.get());self._recipe_refresh();self.recipe_list.selection_set(index)
 def _recipe_move(self,delta):
  self._require_developer()
  if self.recipe_list.curselection():
   index=self.recipe_list.curselection()[0];target=max(0,min(len(self.recipe_blocks)-1,index+delta));block=self.recipe_blocks.pop(index);self.recipe_blocks.insert(target,block);self._recipe_refresh();self.recipe_list.selection_set(target)
 def _recipe_remove(self):
  self._require_developer()
  if self.recipe_list.curselection():self.recipe_blocks.pop(self.recipe_list.curselection()[0]);self._recipe_refresh()
 def _recipe_drag_start(self,e):self.recipe_drag=self.recipe_list.nearest(e.y) if self.recipe_blocks else None
 def _recipe_drag_end(self,e):
  if self.recipe_drag is not None and self.recipe_blocks:
   self.recipe_list.selection_clear(0,'end');self.recipe_list.selection_set(self.recipe_drag);self._recipe_move(self.recipe_list.nearest(e.y)-self.recipe_drag)
  self.recipe_drag=None
 def _recipe_copy(self):self._require_developer();self.recipe_name.set(self.recipe_name.get()+' 복사');self.recipe_enabled.set(False)
 def _recipe_save(self):
  self._require_developer()
  if self.studio_runner.active:raise ValueError('미션 종료 후 템플릿을 변경하세요.')
  destination=self.recipe_destination.get();recipe=dict(name=self.recipe_name.get().strip(),enabled=self.recipe_enabled.get(),destinations=[] if destination=='모든 위치 (이동 미션)' else [destination],blocks=copy.deepcopy(self.recipe_blocks))
  validate_template(recipe,self.map.nodes,self._operator_arm_config(),False)
  existing=next((i for i,t in enumerate(self.operator_templates) if t['name']==recipe['name']),None)
  if existing is None:self.operator_templates.append(recipe)
  else:self.operator_templates[existing]=recipe
  self.studio_config['operator_templates']=copy.deepcopy(self.operator_templates);self._studio_save_settings();self.recipe_selection.set(recipe['name']);self.recipe_combo.configure(values=[r['name'] for r in self.operator_templates]);self.operator_pending=None
 def _recipe_delete(self):
  self._require_developer()
  if self.studio_runner.active or len(self.operator_templates)<=1:raise ValueError('미션 종료 후 삭제하세요. 미션은 하나 이상 유지해야 합니다.')
  self.operator_templates=[r for r in self.operator_templates if r['name']!=self.recipe_selection.get()];self.studio_config['operator_templates']=copy.deepcopy(self.operator_templates);self._studio_save_settings();self.recipe_selection.set(self.operator_templates[0]['name']);self.recipe_combo.configure(values=[r['name'] for r in self.operator_templates]);self.operator_pending=None;self._recipe_load()
