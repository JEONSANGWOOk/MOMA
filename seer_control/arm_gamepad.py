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
def sdk_pulse(group,v,w,now):
 ref,values=axes(group,v,w);index,value=max(values,key=lambda item:abs(item[1]))
 if abs(value)<1e-9:return None
 return dict(ref=ref,axis=index+1,direction=int(value>0),distance=(.3 if ref==2 and index<3 else .2)*abs(value),vel=max(.5,5*abs(value)),expires=now+.15)
