"""Shared pad axis groups, bounded SIM jog and expiring single-axis SDK pulse."""
import math
GROUPS={'J1 / J2':(0,0,1),'J3 / J4':(0,2,3),'J5 / J6':(0,4,5),
        'TCP X / Y':(2,0,1),'TCP Z / RZ':(2,2,5),'TCP RX / RY':(2,3,4)}
def axes(group,v,w):
 if group not in GROUPS or not all(math.isfinite(x) and abs(x)<=1 for x in (v,w)):raise ValueError('팔 조이스틱 축 설정 오류')
 ref,horizontal,vertical=GROUPS[group]
 return ref,[(horizontal,w),(vertical,v)]
def sim_jog(sim,group,v,w,dt,joint_speed=12.,linear_speed=30.,angular_speed=10.):
 ref,values=axes(group,v,w);dt=max(0.,min(.1,dt))
 if sim.state in ('RUNNING','PAUSED'):raise ValueError('팔 재생 종료 후 조종하세요.')
 if ref==0:
  q=list(sim.q)
  for index,value in values:
   if index>=len(q):raise ValueError('6축 팔 모델을 선택하세요.')
   q[index]+=value*math.radians(joint_speed)*dt
 else:
  pose=sim.kin.pose(sim.q)
  for index,value in values:pose[index]+=value*(linear_speed if index<3 else angular_speed)*dt
  q=sim.kin.ik(pose,sim.q)
 sim.set_joints(q)
 return q
def sdk_pulse(group,v,w,now,scale=1.):
 if not math.isfinite(scale) or not .1<=scale<=1:raise ValueError("팔 속도 비율 오류")
 ref,values=axes(group,v,w);index,value=max(values,key=lambda item:abs(item[1]))
 if abs(value)<1e-9:return None
 return dict(ref=ref,axis=index+1,direction=int(value>0),distance=(.3 if ref==2 and index<3 else .2)*abs(value)*scale,vel=max(.05,5*abs(value)*scale),expires=now+.15)


BUTTONS=dict(mode_up=5,mode_down=7,speed_up=11,speed_down=10,gripper=3)
SPEEDS=(10,25,50,75,100)
class ButtonEdges:
 def __init__(self):self.identity=None;self.previous=0
 def update(self,sample,now,allowed):
  if not sample or not allowed or not 0<=now-sample.timestamp<=.25:
   self.identity=None;self.previous=0;return 0
  if self.identity!=sample.identity:self.identity=sample.identity;self.previous=sample.buttons;return 0
  edges=sample.buttons&~self.previous;self.previous=sample.buttons;return edges

def step_group(group,delta):
 names=list(GROUPS);return names[(names.index(group)+delta)%len(names)]
def step_speed(speed,delta):
 index=min(range(len(SPEEDS)),key=lambda i:abs(SPEEDS[i]-speed));return SPEEDS[max(0,min(len(SPEEDS)-1,index+delta))]
