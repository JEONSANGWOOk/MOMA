"""Operator recipes compile to the existing mission engine; no guessed arm poses."""
import copy,math
from .studio_core import validate_actions
from .fairino_programs import expand_actions
from .route_planner import plan_stops

def defaults():
 return [dict(name='이동만',enabled=True,destinations=[],blocks=[dict(type='이동',value='목적지')]),
  dict(name='순환 운반',enabled=True,destinations=[],blocks=[dict(type='이동',value='픽업'),dict(type='이동',value='목적지'),dict(type='이동',value='픽업')]),
  dict(name='자재 공급',enabled=False,destinations=[],blocks=[dict(type='이동',value='픽업'),dict(type='팔 작업',value=''),dict(type='이동',value='목적지'),dict(type='팔 작업',value=''),dict(type='이동',value='복귀')]),
  dict(name='완제품 회수',enabled=False,destinations=[],blocks=[dict(type='이동',value='픽업'),dict(type='팔 작업',value=''),dict(type='이동',value='목적지'),dict(type='팔 작업',value=''),dict(type='이동',value='복귀')]),
  dict(name='현재 노드 왕복 테스트',enabled=True,destinations=[],blocks=[dict(type='이동',value='목적지'),dict(type='이동',value='현재 노드')])]

def validate_template(recipe,nodes,arm,require_enabled=True):
 if not isinstance(recipe,dict) or not isinstance(recipe.get('name'),str) or not 1<=len(recipe['name'].strip())<=60:raise ValueError('미션 이름은 1~60자입니다.')
 if type(recipe.get('enabled')) is not bool:raise ValueError('사용 허용은 true/false입니다.')
 if require_enabled and not recipe['enabled']:raise ValueError('아직 준비되지 않은 미션입니다. 개발자가 팔 작업과 사용 목적지를 설정해야 합니다.')
 destinations=recipe.get('destinations',[])
 if not isinstance(destinations,list) or any(n not in nodes for n in destinations):raise ValueError('사용 목적지가 지도에 없습니다.')
 blocks=recipe.get('blocks')
 if not isinstance(blocks,list) or not 1<=len(blocks)<=100:raise ValueError('미션 블록은 1~100개입니다.')
 has_arm=False
 for b in blocks:
  if not isinstance(b,dict) or set(b)-{'type','value'}:raise ValueError('블록 형식 오류')
  kind=b.get('type');value=b.get('value')
  if kind=='이동':
   if value not in ('픽업','목적지','복귀','현재 노드') and value not in nodes:raise ValueError('이동 위치를 지정하세요.')
  elif kind=='팔 작업':
   has_arm=True
   if value not in arm.get('operations',{}) and value not in arm.get('programs',{}):raise ValueError('등록된 팔 작업/프로그램을 선택하세요.')
  elif kind=='대기':
   try:number=float(value)
   except (ValueError,TypeError):raise ValueError('대기 시간은 숫자입니다.')
   if not math.isfinite(number) or not 0<=number<=600:raise ValueError('대기 시간은 0~600초입니다.')
  else:raise ValueError('지원하지 않는 미션 블록입니다.')
 if has_arm and not destinations:raise ValueError('팔 작업이 있는 미션은 사용 가능한 목적지를 지정하세요.')
 return copy.deepcopy(recipe)

def compile_mission(recipe,params,model,arm,pose):
 recipe=validate_template(recipe,model.nodes,arm)
 repeat=params.get('repeat',1)
 if type(repeat) is not int or not 1<=repeat<=100:raise ValueError('실행 횟수는 1~100입니다.')
 destination=params.get('목적지')
 if destination not in model.nodes:raise ValueError('목적지를 선택하세요.')
 if recipe['destinations'] and destination not in recipe['destinations']:raise ValueError('이 미션에 허용되지 않은 목적지입니다.')
 current=model.nearest(pose['x'],pose['y']) if model.nodes else None
 if any(b['type']=='이동' and b['value']=='현재 노드' for b in recipe['blocks']):
  if not current or math.hypot(pose['x']-model.nodes[current]['x'],pose['y']-model.nodes[current]['y'])>.25:raise ValueError('현재 노드 왕복 테스트는 노드에서 0.25m 이내에 정지한 뒤 미리보기 하세요.')
  if destination==current:raise ValueError('현재 노드와 다른 방문 목적지를 선택하세요.')
 actions=[];stops=[];labels=[]
 for b in recipe['blocks']:
  value=b['value']
  if b['type']=='이동':
   goal=current if value=='현재 노드' else params.get(value) if value in ('픽업','목적지','복귀','현재 노드') else value
   if goal not in model.nodes:raise ValueError(value+' 위치를 선택하세요.')
   actions.append(dict(type='Path Nav',goal=goal,timeout_s=300));stops.append(goal);labels.append('이동 · '+goal)
  elif b['type']=='팔 작업':actions.append(dict(type='Arm Action',operation=value,timeout_s=120));labels.append('팔 작업 · '+value)
  else:actions.append(dict(type='Wait',duration_s=float(value),timeout_s=max(1,float(value)+1)));labels.append('대기 · '+str(value)+'초')
 actions=validate_actions(expand_actions(arm,actions))
 for action in actions:
  if action['type']=='Arm Action' and action['operation'] not in arm.get('operations',{}):raise ValueError('미등록 팔 작업: '+action['operation'])
 current=model.nearest(pose['x'],pose['y']) if model.nodes else None
 route=([current] if current and stops and current!=stops[0] else [])+stops
 plan=plan_stops(model,route)
 # Also check the join from the previous loop endpoint back to the first stop.
 if repeat>1 and len(stops)>1:
  repeated=plan_stops(model,[stops[-1],stops[0]])
  if repeated['errors']:raise ValueError('다음 반복으로 이동할 경로가 없습니다: '+repeated['errors'][0])
 if plan['errors']:raise ValueError(plan['errors'][0])
 return dict(name=recipe['name'],template=copy.deepcopy(recipe),actions=actions,repeat=repeat,stops=route,labels=labels,route=plan)
