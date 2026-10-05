"""Live GUI capability registry. No eval, arbitrary callbacks or generated code."""
import json,math,re,time,tkinter as tk
from tkinter import ttk,filedialog
from types import SimpleNamespace
from contextlib import contextmanager
from pathlib import Path
from .voice_commands import normalize

def compact(value):return re.sub(r"\s+", "", str(value)).casefold()

class GuiRegistry:
 def __init__(self,app):self.app=app;self.targets={};self.scan()
 def add(self,key,label,kind,widget,**extra):
  label=re.sub(r'joint[_ ]([1-6])',lambda m:'J'+m[1],label,flags=re.I)
  self.targets[key]=dict(id=key,label=label,kind=kind,widget=widget,**extra)
 def scan(self):
  aliases={str(v):k for k,v in vars(self.app).items() if isinstance(v,tk.Variable)}
  protected={str(getattr(self.app,k,'')) for k in ('voice_real','pad_arm_real')}
  def walk(parent,path):
   children=parent.winfo_children();previous=''
   for w in children:
    try:
     cls=w.winfo_class();text=str(w.cget('text')) if 'text' in w.keys() else ''
     if cls in ('Label','TLabel') and text:previous=text.split('\n')[0][:70]
     label=text or previous or aliases.get(str(w.cget('textvariable')) if 'textvariable' in w.keys() else '',w.winfo_name())
     full='/'.join(path+[label]);key=str(w)
     if cls in ('TNotebook',):
      for tab in w.tabs():
       child=w.nametowidget(tab)
       if getattr(self.app,'voice_page',None) is child:continue
       title=w.tab(tab,'text');self.add(key+'::'+tab,'/'.join(path+[title]),'screen',w,tab=tab)
       walk(child,path+[title])
      continue
     if getattr(self.app,'voice_page',None) is w:continue
     if cls in ('Button','TButton') and w.cget('command'):
      if w.bind('<ButtonPress-1>'):
       names={'▲':'AMR 수동 전진 150ms','▼':'AMR 수동 후진 150ms','↶':'AMR 수동 좌회전 150ms','↷':'AMR 수동 우회전 150ms','■':'AMR 수동 정지'}
       self.add(key,names.get(text,full),'hold',w,direction={'▲':'forward','▼':'back','↶':'left','↷':'right','■':'zero'}.get(text))
      else:self.add(key,full,'button',w)
     elif cls in ('Checkbutton','TCheckbutton'):
      variable=str(w.cget('variable'))
      if variable not in protected and not re.search(r'실기.*허용|호환성.*확인|안전.*확인',text):self.add(key,full,'toggle',w,variable=variable)
     elif cls in ('Radiobutton','TRadiobutton'):self.add(key,full,'radio',w)
     elif cls in ('TCombobox',):self.add(key,full,'choice',w,values=list(w.cget('values')))
     elif cls in ('Entry','TEntry','Spinbox','TSpinbox') and not w.cget('show'):
      self.add(key,full,'entry',w,alias=aliases.get(str(w.cget('textvariable')),''))
     elif cls=='Text':self.add(key,full,'text',w)
     elif cls in ('Scale','TScale'):self.add(key,full,'scale',w,minimum=float(w.cget('from')),maximum=float(w.cget('to')))
     elif cls in ('Treeview','Listbox'):self.add(key,full,'list',w)
     elif cls=='Menu':
      end=w.index('end')
      for index in range((end+1) if end is not None else 0):
       if w.type(index)=='command':self.add(key+'::'+str(index),'/'.join(path+[str(w.entrycget(index,'label'))]),'menu',w,index=index)
     walk(w,path)
    except tk.TclError:continue
  walk(self.app,[])
  for name in ('노드 선택','좌표 클릭','좌표 드래그'):
   self.add('map::'+name,'지도 / '+name,'map',self.app.canvas)
  if hasattr(self.app,'_role_gui_allowed'):self.targets={key:row for key,row in self.targets.items() if self.app._role_gui_allowed(row)}
 def public(self,targets=None):
  return [{k:v for k,v in row.items() if k in ('id','label','kind','values','alias','minimum','maximum')} for row in (self.targets.values() if targets is None else targets)]
 def candidates(self,text,limit=14):
  source=compact(normalize(text));words=re.findall(r'[가-힣A-Za-z0-9_]+',text)
  ranked=[]
  for row in self.targets.values():
   parts=[compact(part) for part in row['label'].split('/')];tail=parts[-1]
   score=(30 if tail and tail in source else 0)+sum(4 for part in parts[:-1] if part and part in source)
   score+=sum(2 for word in words if len(word)>1 and compact(word) in compact(row['label']))
   score+=sum(12 for value in row.get('values',[]) if compact(value) and compact(value) in source)
   if score:ranked.append((score,row))
  ranked.sort(key=lambda item:item[0],reverse=True)
  return self.public([row for _,row in ranked[:limit]])
 def validate_order(self,command):
  if set(command)-{'action','mission','destination','pickup','return_to','repeat'} or command.get('action')!='operator_order':raise ValueError('운영 미션 명령 형식 오류')
  if command.get('mission') not in [r['name'] for r in self.app.operator_templates]:raise ValueError('등록되지 않은 운영 미션입니다.')
  for key in ('destination','pickup','return_to'):
   if key=='destination' or key in command:
    if command.get(key) not in self.app.map.nodes:raise ValueError('지도에 없는 위치입니다.')
  repeat=command.get('repeat',1)
  if type(repeat) is not int or not 1<=repeat<=100:raise ValueError('실행 횟수는 1~100입니다.')
  return dict(command,repeat=repeat)
 def validate(self,command):
  if set(command)-{'action','target','operation','value'} or command.get('action')!='gui':raise ValueError('GUI 명령 형식 오류')
  row=self.targets.get(command.get('target'))
  if row and hasattr(self.app,'_role_gui_allowed') and not self.app._role_gui_allowed(row):raise ValueError('현재 사용자 모드에서 허용되지 않은 기능입니다.')
  if not row:raise ValueError('현재 화면에 없는 기능입니다. 기능 목록에서 다시 선택하세요.')
  op=command.get('operation');kind=row['kind'];value=command.get('value','')
  if op not in ('invoke','set','select','read'):raise ValueError('지원하지 않는 GUI 조작')
  if not isinstance(value,str) or len(value)>20000:raise ValueError('GUI 입력은 최대 20000자 문자열입니다.')
  if op=='invoke' and kind not in ('button','toggle','radio','menu','hold'):raise ValueError('버튼 기능이 아닙니다.')
  if op=='set':
   if kind not in ('entry','text','choice','toggle','scale','map'):raise ValueError('값을 입력할 수 없는 기능입니다.')
   if kind=='choice' and value not in row['values']:raise ValueError('선택 가능한 값이 아닙니다.')
   if kind=='toggle' and value not in ('true','false'):raise ValueError('체크 설정은 true/false입니다.')
   if kind=='scale':
    try:number=float(value)
    except ValueError:raise ValueError('슬라이더 값은 숫자입니다.')
    if not math.isfinite(number) or not min(row['minimum'],row['maximum'])<=number<=max(row['minimum'],row['maximum']):raise ValueError('슬라이더 범위 초과')
  if kind=='map':
   if op!='set':raise ValueError('지도 명령은 set과 값이 필요합니다.')
   if row['id']=='map::노드 선택':
    if value not in self.app.map.nodes:raise ValueError('지도에 없는 노드')
   else:
    try:points=json.loads(value)
    except (ValueError,TypeError):raise ValueError('지도 좌표는 [x,y] 또는 [x1,y1,x2,y2] 입니다.')
    count=4 if row['id']=='map::좌표 드래그' else 2
    if not isinstance(points,list) or len(points)!=count or any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>10000 for v in points):raise ValueError('지도 좌표 형식/범위 오류')
  if op=='select' and kind not in ('screen','list'):raise ValueError('선택 기능이 아닙니다.')
  if op=='read' and value:raise ValueError('조회에는 값이 필요하지 않습니다.')
  if op=='invoke' and value:
   if kind!='button' or not re.search(r'가져오기|불러오기|내보내기|저장|URDF',row['label']):raise ValueError('이 버튼은 파일 경로를 받지 않습니다.')
   if not Path(value).is_absolute() or '\x00' in value:raise ValueError('파일은 절대 경로로 지정하세요.')
  return dict(action='gui',target=row['id'],operation=op,**({'value':value} if value else {}))
 def describe(self,command):
  row=self.targets[command['target']];names={'invoke':'실행','set':'값 변경','select':'선택','read':'조회'}
  return row['label']+' · '+names[command['operation']]+(' → '+command['value'] if command.get('value') else '')
 def exact(self,text):
  source=compact(normalize(text));matches=[]
  for row in self.targets.values():
   tail=compact(row['label'].split('/')[-1]);kind=row['kind'];op=None;value=''
   if not tail:continue
   if kind=='map' and tail in source:
    match=re.search(r'\[[^\]]+\]',text)
    if match:op='set';value=match[0]
    elif row['id']=='map::노드 선택':
     ids=re.findall(r'(?:LM|CP)\d+',normalize(text),re.I)
     if len(ids)==1:op='set';value=ids[0].upper()
   if source in (tail+'열어줘',tail+'화면열어줘',tail+'보여줘') and kind=='screen':op='select'
   elif kind in ('button','menu','hold') and source in (tail,tail+'실행',tail+'실행해',tail+'눌러줘',tail+'해줘'):op='invoke'
   elif kind=='toggle' and tail in source:
    if source in (tail+'켜줘',tail+'켜',tail+'활성화'):op='set';value='true'
    elif source in (tail+'꺼줘',tail+'꺼',tail+'비활성화'):op='set';value='false'
   elif kind=='choice':
    for choice in row['values']:
     if source in (tail+compact(choice)+'설정',tail+compact(choice)+'로설정',tail+compact(choice)+'로바꿔줘'):op='set';value=str(choice)
   if op:matches.append(dict(action='gui',target=row['id'],operation=op,**({'value':value} if value else {})))
  if len(matches)==1:return self.validate(matches[0])
  return None
 def execute(self,command):
  command=self.validate(command);row=self.targets[command['target']];w=row['widget'];op=command['operation'];kind=row['kind'];value=command.get('value','')
  if not w.winfo_exists():raise ValueError('화면이 닫혔습니다.')
  state=str(w.cget('state')) if 'state' in w.keys() else ''
  if state=='disabled' and op!='read':raise ValueError('현재 비활성화된 기능입니다.')
  if op=='read':
   if kind=='toggle':result=w.getvar(row['variable'])
   elif kind=='text':result=w.get('1.0','end-1c')
   elif kind in ('entry','choice','scale'):result=w.get()
   elif kind=='list':result=str(w.selection()) if isinstance(w,ttk.Treeview) else str(w.curselection())
   else:result=row['label']
   return str(result)[:5000]
  if op=='invoke':
   if kind=='hold':
    if not self.app.manual.get() or self.app.task_running or self.app.studio_runner.active:raise ValueError('수동 활성화 및 미션 종료 후 조작하세요.')
    if row.get('direction'):self.app.press_drive(row['direction'])
    else:w.event_generate('<ButtonPress-1>')
    if self.app.real:
     with self.app._jog_lock:
      if self.app._jog_desired:self.app._jog_desired['_expires']=time.monotonic()+.15
    self.app.after(150,lambda:self.app.release_drive(force=True))
   elif kind=='menu':w.invoke(row['index'])
   elif value:
    with chosen_file(value):w.invoke()
   else:w.invoke()
  elif op=='set':
   if kind=='map':
    if row['id']=='map::노드 선택':self.app._select_node(value);self.app.draw_map()
    else:
     points=json.loads(value)
     # Reuse the actual editor interaction and its edit/relocation guards.
     editor=getattr(self.app,'nodes_page',None)
     if editor is not None:self.app.tabs.select(editor);self.app.update_idletasks()
     self.app.draw_map();x,y=self.app.xy(*points[:2]);self.app.map_click(SimpleNamespace(x=x,y=y))
     if len(points)==4:
      x,y=self.app.xy(*points[2:]);self.app.map_drag(SimpleNamespace(x=x,y=y))
     self.app.map_release(SimpleNamespace(x=x,y=y))
   elif kind=='toggle':
    current=str(w.getvar(row['variable']));wanted=value=='true'
    on=str(w.cget('onvalue')) if 'onvalue' in w.keys() else '1'
    if (current==on)!=wanted:w.invoke()
   elif kind=='choice':w.set(value);w.event_generate('<<ComboboxSelected>>')
   elif kind=='scale':w.set(float(value))
   elif kind=='text':w.delete('1.0','end');w.insert('1.0',value)
   else:w.delete(0,'end');w.insert(0,value)
  elif op=='select':
   if kind=='screen':w.select(row['tab'])
   elif isinstance(w,ttk.Treeview):
    def items(parent=''):
     for item in w.get_children(parent):yield item;yield from items(item)
    matches=[item for item in items() if value in (item,str(w.item(item,'text')),*map(str,w.item(item,'values'))) ]
    if len(matches)!=1:raise ValueError('목록 항목을 고유 이름으로 지정하세요.')
    w.selection_set(matches[0]);w.focus(matches[0]);w.see(matches[0]);w.event_generate('<<TreeviewSelect>>')
   else:
    matches=[i for i in range(w.size()) if w.get(i)==value]
    if len(matches)!=1:raise ValueError('목록 항목을 정확히 지정하세요.')
    w.selection_clear(0,'end');w.selection_set(matches[0]);w.see(matches[0]);w.event_generate('<<ListboxSelect>>')
  return self.describe(command)

def gui_schema(targets):
 branches=[]
 kinds={'invoke':{'button','menu','hold','radio','toggle'},'set':{'entry','text','choice','toggle','scale','map'},'select':{'screen','list'},'read':{row['kind'] for row in targets}}
 for op,allowed in kinds.items():
  ids=[row['id'] for row in targets if row['kind'] in allowed]
  if not ids:continue
  properties={'action':{'type':'string','enum':['gui']},'target':{'type':'string','enum':ids},'operation':{'type':'string','enum':[op]}}
  if op!='read':properties['value']={'type':'string'}
  branches.append({'type':'object','properties':properties,'required':['action','target','operation']+(['value'] if op=='set' else []),'additionalProperties':False})
 return {'oneOf':branches}


@contextmanager
def chosen_file(path):
 # Scope lasts only for a single reviewed GUI callback, restored on error too.
 names=('askopenfilename','asksaveasfilename','askdirectory','askopenfilenames')
 old={name:getattr(filedialog,name) for name in names}
 try:
  for name in names:setattr(filedialog,name,(lambda *a,**k:(path,)) if name=='askopenfilenames' else (lambda *a,**k:path))
  yield
 finally:
  for name,fn in old.items():setattr(filedialog,name,fn)
