"""Independent kinematic arm simulator. No network or hardware calls."""
import copy
import math
from .geometry3d import transform
from .fairino_api import vector
from .fairino_programs import program_actions

def wrap(v):return (v+math.pi)%(2*math.pi)-math.pi

def rotation_error(target,current):
 r=[[sum(target[i][k]*current[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
 cosine=max(-1.,min(1.,(sum(r[i][i] for i in range(3))-1)/2))
 angle=math.acos(cosine)
 if angle<1e-7:return [(r[2][1]-r[1][2])/2,(r[0][2]-r[2][0])/2,(r[1][0]-r[0][1])/2]
 if math.pi-angle<1e-5:
  axis=[math.sqrt(max(0.,(r[i][i]+1)/2)) for i in range(3)]
  biggest=max(range(3),key=lambda i:axis[i])
  for i in range(3):
   if i!=biggest and axis[biggest]>1e-8:axis[i]=(r[i][biggest]+r[biggest][i])/(4*axis[biggest])
  return [v*angle for v in axis]
 scale=angle/(2*math.sin(angle))
 return [(r[2][1]-r[1][2])*scale,(r[0][2]-r[2][0])*scale,(r[1][0]-r[0][1])*scale]

def linear_solve(matrix,values):
 a=[list(row)+[v] for row,v in zip(matrix,values)];n=len(values)
 for i in range(n):
  pivot=max(range(i,n),key=lambda j:abs(a[j][i]))
  if abs(a[pivot][i])<1e-14:raise ValueError('역기구학 행렬이 특이합니다.')
  a[i],a[pivot]=a[pivot],a[i];div=a[i][i];a[i]=[v/div for v in a[i]]
  for j in range(n):
   if j!=i:
    scale=a[j][i];a[j]=[x-scale*y for x,y in zip(a[j],a[i])]
 return [row[-1] for row in a]

class ArmKinematics:
 def __init__(self,asset,tip=None):
  self.asset=asset;self.joints=asset.movable()
  children={j['parent'] for j in asset.joints};leaves=[n for n in asset.links if n not in children]
  self.tip=tip or ('tool' if 'tool' in leaves else leaves[-1])
  if self.tip not in asset.links:raise ValueError('TCP link가 없습니다.')
 def clamp(self,q):
  if len(q)!=len(self.joints):raise ValueError('모델의 관절 수가 일치하지 않습니다.')
  values=[]
  for joint,value in zip(self.joints,q):
   if type(value) not in (int,float) or not math.isfinite(value):raise ValueError('관절값 오류')
   values.append(value if joint['type']=='continuous' else max(joint['lower'],min(joint['upper'],value)))
  return values
 def positions(self,q):return dict(zip((j['name'] for j in self.joints),q))
 def fk(self,q):return self.asset.link_transforms(self.positions(q))[self.tip]
 def pose(self,q):
  t=self.fk(q);pitch=math.asin(max(-1.,min(1.,-t[2][0])))
  if abs(math.cos(pitch))>1e-7:roll=math.atan2(t[2][1],t[2][2]);yaw=math.atan2(t[1][0],t[0][0])
  else:roll=0.;yaw=math.atan2(-t[0][1],t[1][1])
  return [t[i][3]*1000 for i in range(3)]+[math.degrees(v) for v in (roll,pitch,yaw)]
 def ik(self,pose,seed,iterations=120):
  p=vector(pose,'TCP');target=transform([v/1000 for v in p[:3]],[math.radians(v) for v in p[3:]])
  q=self.clamp(list(seed));damping=.025;weight=.3
  for iteration in range(iterations):
   current=self.fk(q);position=[target[i][3]-current[i][3] for i in range(3)];rotation=rotation_error(target,current)
   if math.sqrt(sum(v*v for v in position))<.0008 and math.sqrt(sum(v*v for v in rotation))<.004:return q
   error=position+[v*weight for v in rotation];columns=[];eps=1e-5
   for i in range(len(q)):
    shifted=list(q);shifted[i]+=eps;t=self.fk(shifted)
    columns.append([(t[j][3]-current[j][3])/eps for j in range(3)]+[v*weight/eps for v in rotation_error(t,current)])
   jac=[[column[r] for column in columns] for r in range(6)]
   jj=[[sum(jac[i][k]*jac[j][k] for k in range(len(q)))+(damping*damping if i==j else 0) for j in range(6)] for i in range(6)]
   solved=linear_solve(jj,error);delta=[sum(jac[r][k]*solved[r] for r in range(6)) for k in range(len(q))]
   size=max(1.,max(abs(v) for v in delta)/.15);q=self.clamp([a+b/size for a,b in zip(q,delta)])
  raise ValueError('TCP 목표의 역기구학을 찾지 못했습니다. 도달 범위·자세·관절 한계를 확인하세요.')

class ArmSimulator:
 def __init__(self,asset):
  self.kin=ArmKinematics(asset);self.q=self.kin.clamp([0.]*len(self.kin.joints));self.state='IDLE'
  self.actions=[];self.index=0;self.elapsed=0.;self.entered=False;self.path=[];self.path_time=0.
  self.io={'DO':{},'ToolDO':{},'DI':{},'ToolDI':{}};self.events=[];self.error='';self.trail=[]
 def set_joints(self,q):
  if self.state in ('RUNNING','PAUSED'):raise ValueError('재생을 정지한 뒤 관절을 조절하세요.')
  checked=self.kin.clamp(q)
  if any(abs(a-b)>1e-7 for a,b in zip(q,checked)):raise ValueError('URDF 관절 범위를 벗어났습니다.')
  self.q=checked
 def start(self,config,program=None,operation=None):
  from .fairino_api import validate
  validate(config)
  if self.state in ('RUNNING','PAUSED'):raise ValueError('현재 재생을 정지하세요.')
  self.config=copy.deepcopy(config)
  self.actions=program_actions(config,program) if program else [dict(type='Arm Action',operation=operation,timeout_s=60)]
  # Check all referenced operations before any simulated movement.
  for a in self.actions:
   if a['type']=='Arm Action' and a['operation'] not in config['operations']:raise ValueError('미등록 작업: '+str(a['operation']))
  self.state='RUNNING';self.index=0;self.elapsed=0;self.entered=False;self.error='';self.events=[];self.trail=[]
 def stop(self):
  if self.state in ('RUNNING','PAUSED'):self.events.append(dict(step=self.index+1,result='취소'))
  self.state='CANCELED';self.path=[];self.entered=False
 def _enter(self,a):
  self.elapsed=0.;self.entered=True;self.path=[];self.path_time=0.
  if a['type']=='Wait':return
  spec=self.config['operations'][a['operation']];method=spec['method']
  if method in ('MoveJ','MoveL'):
   if spec.get('tool',0)!=0 or spec.get('user',0)!=0:raise ValueError('독립 SIM은 tool/user=0 좌표만 지원합니다. 다른 좌표계는 변환 후 시험하세요.')
   speed=float(spec.get('vel',10))/100
   if len(self.q)!=6 or any(j['type'] not in ('revolute','continuous') for j in self.kin.joints):raise ValueError('FR5 동작 미리보기는 6축 회전 관절 모델이 필요합니다.')
   if method=='MoveJ':
    target=[math.radians(v) for v in vector(spec['target'],'target')];clamped=self.kin.clamp(target)
    if any(abs(a-b)>1e-6 for a,b in zip(target,clamped)):raise ValueError('MoveJ 목표가 모델 관절 한계를 벗어났습니다.')
    self.path=[list(self.q),target];self.path_time=max(.2,max(abs(a-b) for a,b in zip(self.q,target))/(math.pi*speed))
   else:
    source=self.kin.pose(self.q);target=vector(spec['target'],'target');distance=math.dist(source[:3],target[:3])/1000
    angular=max(abs(wrap(math.radians(b-a))) for a,b in zip(source[3:],target[3:]))
    count=max(2,min(100,math.ceil(max(distance/.015,angular/.05))));seed=list(self.q);self.path=[seed]
    for i in range(1,count+1):
     ratio=i/count;pose=[a+(b-a)*ratio for a,b in zip(source[:3],target[:3])]+[a+math.degrees(wrap(math.radians(b-a)))*ratio for a,b in zip(source[3:],target[3:])]
     seed=self.kin.ik(pose,seed);self.path.append(seed)
    self.path_time=max(.2,distance/(.25*speed),angular/(math.pi*speed))
  elif method.startswith('Set'):
   self.io['ToolDO' if method=='SetToolDO' else 'DO'][spec['id']]=spec['status']
 def tick(self,dt):
  if self.state!='RUNNING':return
  a=self.actions[self.index]
  try:
   if not self.entered:self._enter(a)
   self.elapsed+=max(0.,dt)
   if self.elapsed>a.get('timeout_s',60):raise TimeoutError('단계 제한 시간 초과')
   done=False
   if a['type']=='Wait':done=self.elapsed>=a['duration_s']
   else:
    spec=self.config['operations'][a['operation']];method=spec['method']
    if method in ('MoveJ','MoveL'):
     ratio=min(1.,self.elapsed/self.path_time)*(len(self.path)-1);i=min(len(self.path)-2,int(ratio));fraction=ratio-i
     self.q=[a+(b-a)*fraction for a,b in zip(self.path[i],self.path[i+1])];done=self.elapsed>=self.path_time
     p=tuple(v/1000 for v in self.kin.pose(self.q)[:3])
     if not self.trail or math.dist(p,self.trail[-1])>.002:self.trail.append(p);self.trail=self.trail[-800:]
    elif method.startswith('Wait'):done=self.io['ToolDI' if method=='WaitToolDI' else 'DI'].get(spec['id'],0)==spec['status']
    else:done=True
   if done:
    self.events.append(dict(step=self.index+1,operation=a.get('operation','대기'),result='성공',seconds=round(self.elapsed,3)))
    self.index+=1;self.entered=False
    if self.index>=len(self.actions):self.state='COMPLETED'
  except Exception as error:
   self.error=str(error);self.state='FAILED';self.events.append(dict(step=self.index+1,operation=a.get('operation','대기'),result='실패',detail=self.error))
