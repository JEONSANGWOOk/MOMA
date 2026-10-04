"""Offline FR5 task composition. Templates contain names, never guessed poses."""
import copy
import math

FORMAT='fairino-task-library-v1'
TEMPLATES={
 '픽앤플레이스': ['safe_pose','grip_open','pick_approach','pick_contact','grip_close','grip_confirm','pick_approach','place_approach','place_contact','grip_open','place_approach','safe_pose'],
 '도어 열기': ['safe_pose','grip_open','door_approach','door_handle','grip_close','grip_confirm','door_unlatch','door_arc_1','door_arc_2','door_arc_3','grip_open','door_retreat','safe_pose'],
 '자재 공급': ['safe_pose','machine_ready','grip_open','supply_approach','supply_pick','grip_close','grip_confirm','supply_approach','machine_approach','machine_insert','grip_open','machine_approach','supply_done','safe_pose'],
}

def template(name):
 return [dict(operation=n,timeout_s=60) for n in TEMPLATES[name]]

def validate_programs(config,strict=False):
 programs=config.get('programs',{})
 if not isinstance(programs,dict):raise ValueError('programs는 JSON 객체입니다.')
 operations=config.get('operations',{})
 for name,steps in programs.items():
  if not isinstance(name,str) or not name.strip() or name in operations:raise ValueError('작업과 프로그램 이름은 비어 있거나 중복될 수 없습니다.')
  if not isinstance(steps,list) or not 1<=len(steps)<=300:raise ValueError(name+': 단계 수 1~300')
  for step in steps:
   if not isinstance(step,dict) or ('operation' in step)==('wait_s' in step):raise ValueError(name+': operation 또는 wait_s 중 하나를 지정하세요.')
   if 'operation' in step:
    op=step['operation']
    if not isinstance(op,str) or not op.strip():raise ValueError(name+': 작업 이름 오류')
    if op in programs:raise ValueError('프로그램 중첩은 지원하지 않습니다: '+op)
    if strict and op not in operations:raise ValueError('미등록 작업: '+op)
   else:
    value=step['wait_s']
    if type(value) not in (int,float) or not math.isfinite(value) or not 0<=value<=3600:raise ValueError('wait_s: 0~3600초')
   timeout=step.get('timeout_s',60)
   if type(timeout) not in (int,float) or not math.isfinite(timeout) or not 0<timeout<=86400:raise ValueError('timeout_s: 0 초과~86400초')
 return programs

def program_actions(config,name):
 # Validate only the selected program: other drafts may have unresolved references.
 steps=config.get('programs',{}).get(name)
 if steps is None:raise ValueError('등록되지 않은 프로그램: '+name)
 subset=dict(config,programs={name:steps});validate_programs(subset,True)
 actions=[]
 for step in steps:
  if 'wait_s' in step:
   actions.append(dict(type='Wait',duration_s=step['wait_s'],timeout_s=max(step.get('timeout_s',60),step['wait_s']+1)))
  else:actions.append(dict(type='Arm Action',operation=step['operation'],duration_s=2,timeout_s=step.get('timeout_s',60)))
 return actions

def expand_actions(config,actions):
 out=[];starts={}
 for i,action in enumerate(actions,1):
  starts[i]=len(out)+1
  if action.get('type')=='Arm Action' and action.get('operation') in config.get('programs',{}):
   expanded=program_actions(config,action['operation'])
   if action.get('delay_ms'):expanded[-1]['delay_ms']=action['delay_ms']
   out.extend(expanded)
  else:out.append(copy.deepcopy(action))
 for action in out:
  if action.get('type')=='Branch DI' and action.get('target') in starts:action['target']=starts[action['target']]
 if len(out)>3000:raise ValueError('확장된 미션 단계가 너무 많습니다.')
 return out

def library(config):
 return dict(format=FORMAT,operations=copy.deepcopy(config.get('operations',{})),programs=copy.deepcopy(config.get('programs',{})))
