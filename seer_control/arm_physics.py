"""Offline engineering gripper/contact model (SI units), never hardware I/O."""
from .decision_log import decide, audited, snapshot
import copy,math
from .geometry3d import box,cylinder,transform,multiply,point,identity

def inverse(t):
 return tuple(tuple(t[j][i] if j<3 else -sum(t[k][i]*t[k][3] for k in range(3)) for j in range(4)) if i<3 else (0,0,0,1) for i in range(4))
def dot(a,b):return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def obb(t,size):return (point(t,(0,0,0)),[tuple(t[i][j] for i in range(3)) for j in range(3)],[v/2 for v in size])
def separation(a,b):
 """SAT separating-axis distance; <=0 indicates conservative OBB overlap."""
 ca,aa,ha=a;cb,ab,hb=b;axes=aa+ab
 for x in aa:
  for y in ab:
   z=(x[1]*y[2]-x[2]*y[1],x[2]*y[0]-x[0]*y[2],x[0]*y[1]-x[1]*y[0]);n=math.sqrt(dot(z,z))
   if n>1e-8:axes.append(tuple(v/n for v in z))
 d=tuple(y-x for x,y in zip(ca,cb))
 return max(abs(dot(d,n))-(ha[0]*abs(dot(aa[0],n))+ha[1]*abs(dot(aa[1],n))+ha[2]*abs(dot(aa[2],n)))-(hb[0]*abs(dot(ab[0],n))+hb[1]*abs(dot(ab[1],n))+hb[2]*abs(dot(ab[2],n))) for n in axes)

def contact_gap(a,b,margin):
 # Bounding spheres reject distant pairs before the full oriented-box SAT.
 # This only skips pairs whose conservative lower gap already exceeds margin.
 ra=math.sqrt(dot(a[2],a[2]));rb=math.sqrt(dot(b[2],b[2]))
 distance2=sum((x-y)**2 for x,y in zip(a[0],b[0]))
 if distance2>(ra+rb+margin)**2:return math.sqrt(distance2)-ra-rb
 return separation(a,b)

DEFAULT=dict(kind='none',do_bank='ToolDO',do_channel=0,di_bank='ToolDI',di_channel=0,
 mount_offset_m=0.,stroke_m=.085,speed_m_s=.08,force_n=60.,friction=.5,tool_mass_kg=.8,
 cup_diameter_m=.04,vacuum_kpa=60.,pump_tau_s=.3,leak_tau_s=.18,seal_efficiency=.8,
 safety_factor=1.5,payload_limit_kg=5.,collision_margin_m=.025,stop_on_collision=True,
 self_collision=False,ground_collision=True)
class ArmPhysics:
 def __init__(self,kin):
  self.kin=kin;self.config=copy.deepcopy(DEFAULT);self.objects=[];self.events=[];self.opening=self.config['stroke_m'];self.pressure=0.;self.held=None;self.relative=None;self.status='그리퍼 없음';self.risk=[];self.near=[];self.last_position=None;self.last_velocity=[0.,0.,0.];self.force=0.;self.acceleration=[0.,0.,0.];self.blocked=[];self.future_risk=[];self.forecast_q=None;self.time=0.
  self.bounds=[]
  for name,visuals in kin.asset.links.items():
   vertices=[point(local,p) for faces,local,color in visuals for face in faces for p in face]
   if vertices:
    lo=[min(p[i] for p in vertices) for i in range(3)];hi=[max(p[i] for p in vertices) for i in range(3)]
    self.bounds.append((name,[(a+b)/2 for a,b in zip(lo,hi)],[max(.002,b-a) for a,b in zip(lo,hi)]))
    if name==kin.tip and getattr(kin.asset,'name','')=='fairino5_v6_robot':self.config['mount_offset_m']=round(max(0,hi[2])+.002,3)
 def configure(self,values):
  cfg=dict(self.config);cfg.update(values)
  if cfg['kind'] not in ('none','finger','vacuum'):raise ValueError('그리퍼 종류 오류')
  if cfg['do_bank'] not in ('DO','ToolDO') or cfg['di_bank'] not in ('DI','ToolDI'):raise ValueError('I/O bank 오류')
  for key in ('do_channel','di_channel'):
   if type(cfg[key]) is not int or not 0<=cfg[key]<=15:raise ValueError('I/O 채널은 0~15 정수입니다.')
  for key in DEFAULT:
   if key in ('kind','do_bank','di_bank','do_channel','di_channel','stop_on_collision','self_collision','ground_collision'):continue
   v=cfg[key]
   if type(v) not in (int,float) or not math.isfinite(v) or (v<0 if key=='mount_offset_m' else v<=0):raise ValueError(key+'는 유효한 양수여야 합니다(장착 오프셋은 0 허용).')
  if cfg['seal_efficiency']>1 or cfg['safety_factor']<1:raise ValueError('seal_efficiency ≤ 1, safety_factor ≥ 1 필요')
  if any(type(cfg[k]) is not bool for k in ('stop_on_collision','self_collision','ground_collision')):raise ValueError('충돌 옵션은 true/false입니다.')
  if cfg['kind']!=self.config['kind'] and self.held:raise ValueError('물체를 놓은 후 그리퍼를 변경하세요.')
  self.config=cfg;self.opening=min(self.opening,cfg['stroke_m'])
 def add_object(self,name,kind,size,mass,pose,static=False,friction=.5,sealable=True):
  if kind not in ('box','panel','obstacle'):raise ValueError('물체 종류 오류')
  if len(size)!=3 or any(not math.isfinite(float(v)) or float(v)<=0 for v in size):raise ValueError('크기는 양수 m 3개입니다.')
  if not math.isfinite(float(mass)) or mass<=0 or not math.isfinite(float(friction)) or friction<=0:raise ValueError('질량과 마찰계수는 양수입니다.')
  if len(pose)!=6 or any(not math.isfinite(float(v)) for v in pose):raise ValueError('자세는 유한한 숫자 6개입니다.')
  name=name.strip()
  if not name or any(o['name']==name for o in self.objects):raise ValueError('물체 이름이 비었거나 중복됩니다.')
  o=dict(name=name,kind=kind,size=list(size),mass=float(mass),matrix=transform(pose[:3],pose[3:]),static=bool(static),friction=float(friction),sealable=bool(sealable),velocity=[0.,0.,0.],state='배치')
  self.objects.append(o);return o
 def tcp(self,q):return multiply(self.kin.fk(q),transform((0,0,self.config['mount_offset_m'])))
 def tool_boxes(self,q):
  c=self.config;t=self.tcp(q);kind=c['kind'];rows=[]
  if kind=='finger':
   rows=[('gripper_body',multiply(t,transform((0,0,.025))),(.10,.065,.05))]
   for side in (-1,1):rows.append(('finger',multiply(t,transform((side*(self.opening/2+.008),0,.095))),(.016,.025,.06)))
  elif kind=='vacuum':rows=[('vacuum_body',multiply(t,transform((0,0,.045))),(.07,.07,.09)),('cup',multiply(t,transform((0,0,.115))),(c['cup_diameter_m'],c['cup_diameter_m'],.025))]
  return rows
 def contacts(self,q):
  poses=self.kin.asset.link_transforms(self.kin.positions(q));robot=[(name,obb(multiply(poses[name],transform(center)),size)) for name,center,size in self.bounds]
  tools=[(name,obb(t,size)) for name,t,size in self.tool_boxes(q)]
  risks=[];near=[];margin=self.config['collision_margin_m']
  for o in self.objects:
   if o is self.held:continue
   target=obb(o['matrix'],o['size'])
   for name,bounds in robot+tools:
    gap=contact_gap(bounds,target,margin)
    # Movable workpiece contact with fingers/cup is the intentional grasp interface.
    if name in ('finger','cup') and not o['static']:continue
    if gap<=0:risks.append((name,o['name']))
    elif gap<margin:near.append((name,o['name'],gap))
  if self.held:
   matrix=multiply(self.tcp(q),self.relative)
   parts=self.held.get('collision_parts') or [dict(matrix=transform(),size=self.held['size'])]
   held_parts=[(multiply(matrix,p['matrix']),p['size']) for p in parts]
   held_bounds=[obb(t,size) for t,size in held_parts]
   for o in self.objects:
    if o is not self.held and any(contact_gap(bounds,obb(o['matrix'],o['size']),0)<-.0005 for bounds in held_bounds):risks.append((self.held['name'],o['name']))
  if self.config['ground_collision']:
   # Exact vertical support of each oriented box, equivalent to all 8 vertices.
   ground_boxes=[(name,bounds) for name,bounds in robot if name not in getattr(self.kin.asset,'roots',[])+['base_link']]+tools
   for name,(center,axes,half) in ground_boxes:
    low=center[2]-sum(abs(axis[2])*h for axis,h in zip(axes,half))
    if low<-.005:risks.append((name,'바닥'))
    elif low<margin:near.append((name,'바닥',max(0,low)))
   if self.held and min(c[2]-sum(abs(a[2])*h for a,h in zip(axes,half)) for c,axes,half in held_bounds)<-.001:risks.append((self.held['name'],'바닥'))
  if self.config['self_collision']:
   adjacent={frozenset((j['parent'],j['child'])) for j in self.kin.asset.joints}
   for i,(name,a) in enumerate(robot):
    for other,b in robot[i+1:]:
     if frozenset((name,other)) not in adjacent and separation(a,b)<-.005:risks.append((name,other))
  return risks,near
 def motion_samples(self,source,target,thickness):
  angular_step=min(.025,max(.0001,thickness/8))
  return max(1,math.ceil(max((abs(a-b) for a,b in zip(source,target)),default=0)/angular_step))
 def check_motion(self,source,target):
  # Joint sweep spacing is tightened for thin obstacles (conservative 2 m reach).
  thickness=min((min(o['size']) for o in self.objects if o is not self.held),default=.08)
  count=self.motion_samples(source,target,thickness)
  for i in range(1,count+1):
   q=[a+(b-a)*i/count for a,b in zip(source,target)];risks,near=self.contacts(q)
   if risks:
    decide(self,'SIM.팔 연속 충돌 검사',dict(source=source,target=target,sample_q=q),dict(contacts=risks,stop_on_collision=self.config['stop_on_collision']),'이동 중간 자세에서 충돌 예상','이동 차단' if self.config['stop_on_collision'] else '설정에 따라 경고 후 이동 허용',identity=risks)
    self.risk=risks;self.blocked=risks;self.forecast_q=list(q);self.future_risk=risks;self.events.append(dict(event='충돌 차단',detail=str(risks)))
    if self.config['stop_on_collision']:raise ValueError('충돌 예상: '+', '.join(a+' ↔ '+b for a,b in risks)+' · 이동 차단')
    return
  decide(self,'SIM.팔 연속 충돌 검사',dict(source=source,target=target),dict(samples=count),'중간 자세 충돌 없음','검증된 관절 이동 허용')
  self.blocked=[]
 def forecast(self,q):
  self.future_risk,_=self.contacts(q);
  decide(self,'SIM.팔 미래 충돌',dict(forecast_q=q),dict(contacts=self.future_risk),'예상 자세 충돌 위험' if self.future_risk else '예상 자세 충돌 없음','위험 표시 · 실제 이동은 연속 충돌 검사로 결정' if self.future_risk else '주행 예측 계속',identity=self.future_risk)
  self.forecast_q=list(q) if self.future_risk else None
 def _candidate(self,q):
  inv=inverse(self.tcp(q));c=self.config;tip=.125 if c['kind']=='vacuum' else .095
  for o in self.objects:
   if o['static'] or o is self.held:continue
   local=multiply(inv,o['matrix']);center=point(local,(0,0,0));corners=[point(local,p) for face in box(o['size']) for p in face]
   lo=[min(p[i] for p in corners) for i in range(3)];hi=[max(p[i] for p in corners) for i in range(3)]
   if c['kind']=='finger':
    width=hi[0]-lo[0]
    if abs(center[0])<.008 and abs(center[1])<(o['size'][1]/2+.012) and lo[2]<.14 and hi[2]>.055 and width<=c['stroke_m']+.001:return o,width
   else:
    normals=[tuple(local[i][j] for i in range(3)) for j in range(3)];axis=max(range(3),key=lambda j:abs(normals[j][2]));other=[j for j in range(3) if j!=axis]
    # Flat face, full cup coverage, near contact and <15 degree misalignment.
    if o['sealable'] and abs(normals[axis][2])>math.cos(math.radians(15)) and abs(lo[2]-tip)<.012 and abs(center[0])<.012 and abs(center[1])<.012 and min(o['size'][j] for j in other)>=c['cup_diameter_m']:return o,0
  return None,None
 def release(self,reason='해제'):
  if self.held:
   decide(self,'SIM.그리퍼 해제',dict(object=self.held['name']),dict(reason=reason,force_n=self.force,pressure_pa=self.pressure),'파지 해제 조건 발생','물체 해제 · 낙하 모델 적용',force=True)
   self.held['state']='해제 / 낙하';self.held['velocity']=list(self.last_velocity);self.events.append(dict(event=reason,object=self.held['name']));self.held=None;self.relative=None
 def advance(self,q,io,dt):
  dt=max(0,min(float(dt),.15));self.time+=dt;c=self.config;kind=c['kind'];on=bool(io[c['do_bank']].get(c['do_channel'],0));t=self.tcp(q);pos=point(t,(0,0,0))
  if dt>1e-6 and self.last_position is not None:
   velocity=[(a-b)/dt for a,b in zip(pos,self.last_position)];self.acceleration=[(a-b)/dt for a,b in zip(velocity,self.last_velocity)];self.last_velocity=velocity
  self.last_position=pos
  candidate,width=self._candidate(q) if kind!='none' else (None,None)
  if self.held:
   candidate=self.held
   local=multiply(inverse(t),multiply(t,self.relative));corners=[point(local,p) for face in box(candidate['size']) for p in face]
   width=max(p[0] for p in corners)-min(p[0] for p in corners)
  if kind=='finger':
   target=width if on and candidate else 0 if on else c['stroke_m'];travel=c['speed_m_s']*dt
   self.opening+=max(-travel,min(travel,target-self.opening));self.force=c['force_n'] if on and candidate and abs(self.opening-width)<.001 else 0
   ready=self.force>0
  elif kind=='vacuum':
   seal=candidate is not None or self.held is not None
   target=c['vacuum_kpa']*1000*c['seal_efficiency'] if on and seal else 0
   self.pressure+=(target-self.pressure)*(1-math.exp(-dt/(c['pump_tau_s'] if target else c['leak_tau_s'])))
   self.force=self.pressure*math.pi*(c['cup_diameter_m']/2)**2;ready=on and self.pressure>target*.7 and seal
  else:self.force=0;ready=False
  if not on and kind!='vacuum':self.release('I/O OFF 해제')
  if kind!='none':
   decide(self,'SIM.그리퍼 파지',dict(kind=kind,io_on=on,candidate=candidate['name'] if candidate else None,held=self.held['name'] if self.held else None),dict(ready=ready,force_n=self.force,pressure_pa=self.pressure,opening_m=self.opening),'파지 조건 충족' if candidate and ready else '파지 조건 미충족','물체 파지' if self.held is None and candidate and ready else ('파지 유지' if self.held else '파지 대기'),identity=(on,candidate['name'] if candidate else None,ready,self.held is not None))
  if self.held is None and candidate and ready:
   self.held=candidate;self.relative=multiply(inverse(t),candidate['matrix']);candidate['state']='파지';self.events.append(dict(event='파지',object=candidate['name']))
  if self.held:
   o=self.held;required=o['mass']*math.sqrt(sum((self.acceleration[i]+(9.81 if i==2 else 0))**2 for i in range(3)))*c['safety_factor']
   capacity=self.force*min(c['friction'],o['friction']) if kind=='finger' else self.force
   overload=required>capacity
   if kind=='vacuum':
    load=[o['mass']*(self.acceleration[i]+(9.81 if i==2 else 0))*c['safety_factor'] for i in range(3)];axis=tuple(t[i][2] for i in range(3));normal=abs(dot(load,axis));shear=math.sqrt(max(0,dot(load,load)-normal*normal))
    overload=normal>self.force or shear>self.force*min(c['friction'],o['friction'])
   decide(self,'SIM.그리퍼 하중',dict(object=o['name'],mass_kg=o['mass']),dict(required_n=required,capacity_n=capacity,overload=overload,tool_mass_kg=c['tool_mass_kg'],payload_limit_kg=c['payload_limit_kg'],acceleration=self.acceleration),'파지력/허용하중 초과' if overload or o['mass']+c['tool_mass_kg']>c['payload_limit_kg'] else '파지력/하중 조건 충족','물체 해제' if overload or o['mass']+c['tool_mass_kg']>c['payload_limit_kg'] else '파지 유지',identity=(o['name'],overload,o['mass']+c['tool_mass_kg']>c['payload_limit_kg']))
   if overload or o['mass']+c['tool_mass_kg']>c['payload_limit_kg']:self.release('I/O OFF 해제' if not on else '파지력/가속도/허용하중 초과')
   else:o['matrix']=multiply(t,self.relative)
  if kind!='none':io[c['di_bank']][c['di_channel']]=int(self.held is not None)
  for o in self.objects:
   if o['static'] or o is self.held:continue
   matrix=[list(row) for row in o['matrix']];low=min(point(matrix,p)[2] for face in box(o['size']) for p in face)
   support=0.
   for other in self.objects:
    if other is o or other is self.held:continue
    center=point(matrix,(0,0,0));corners=[point(other['matrix'],p) for face in box(other['size']) for p in face];lo=[min(p[i] for p in corners) for i in range(3)];hi=[max(p[i] for p in corners) for i in range(3)]
    if lo[0]<=center[0]<=hi[0] and lo[1]<=center[1]<=hi[1] and hi[2]<=low+.005:support=max(support,hi[2])
   o['velocity'][2]-=9.81*dt
   for i in range(3):matrix[i][3]+=o['velocity'][i]*dt
   newlow=min(point(matrix,p)[2] for face in box(o['size']) for p in face)
   if newlow<support:
    matrix[2][3]+=support-newlow;o['velocity']=[0.,0.,0.];o['state']='안착'
   o['matrix']=matrix
  self.risk,self.near=self.contacts(q);self.risk=list(dict.fromkeys(self.risk+self.blocked))
  self.status=('없음' if kind=='none' else ('2지' if kind=='finger' else '진공'))+f' · 간격 {self.opening*1000:.1f} mm · 힘 {self.force:.1f} N · 진공 {self.pressure/1000:.1f} kPa · '+('파지 '+self.held['name'] if self.held else '미파지')
  for event in self.events:
   if 'time_s' not in event:event['time_s']=round(self.time,3)
  self.events=self.events[-500:]
 def geometry(self,q,show_bounds=True):
  faces=[];lines=[]
  for name,t,size in self.tool_boxes(q):
   color='#303c4d' if name.endswith('body') else '#40b4cf'
   geometry=cylinder(self.config['cup_diameter_m']/2,.025) if name=='cup' else box(size)
   faces.extend((tuple(point(t,p) for p in face),color,'gripper') for face in geometry)
  risky={name for pair in self.risk+self.future_risk for name in pair};near={name for pair in self.near for name in pair[:2]}
  boxes=[]
  for o in self.objects:
   color='#f04e55' if o['name'] in risky else '#edb339' if o['name'] in near else '#41ad88' if o is self.held else '#b48854' if o['kind']=='box' else '#819aba'
   faces.extend((tuple(point(o['matrix'],p) for p in face),color,'workpiece') for face in box(o['size']));boxes.append((o['name'],o['matrix'],o['size']))
  if show_bounds:
   poses=self.kin.asset.link_transforms(self.kin.positions(q));boxes.extend((name,multiply(poses[name],transform(center)),[v+2*self.config['collision_margin_m'] for v in size]) for name,center,size in self.bounds)
   boxes.extend(self.tool_boxes(q))
   for name,t,size in boxes:
    color='#f04e55' if name in risky else '#edb339' if name in near else '#9bb4c5'
    for face in box(size):
     for a,b in zip(face,face[1:]+face[:1]):lines.append((point(t,a),point(t,b),color,1))
  return faces,lines
 def snapshot(self):
  objects=[{k:copy.deepcopy(v) for k,v in o.items() if k!='velocity'} for o in self.objects]
  if self.held:
   next(o for o in objects if o['name']==self.held['name'])['state']='배치'
  return dict(config=copy.deepcopy(self.config),objects=objects)
 def restore(self,data):
  self.configure(data.get('config',{}));objects=[]
  for o in data.get('objects',[]):
   matrix=o['matrix']
   if len(matrix)!=4 or any(len(row)!=4 or any(type(v) not in (int,float) or not math.isfinite(v) for v in row) for row in matrix):raise ValueError('물체 변환 오류')
   item=self.add_object(o['name'],o['kind'],o['size'],o['mass'],[0]*6,o.get('static',False),o.get('friction',.5),o.get('sealable',True));item['matrix']=matrix
  self.held=None;self.relative=None
