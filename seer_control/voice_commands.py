"""Voice/LLM proposes a small command; only this validator authorizes its shape."""
import json,re
ACTIONS=('goto','sequence','loop','pause','resume','cancel','stop','status','arm_action')
SCHEMA={'type':'object','properties':{'action':{'type':'string','enum':list(ACTIONS)},'nodes':{'type':'array','items':{'type':'string'}},'start':{'type':'string'},'repeats':{'type':'integer'},'operation':{'type':'string'}},'required':['action'],'additionalProperties':False}
NUM={'일':'1','이':'2','삼':'3','사':'4','오':'5','육':'6','칠':'7','팔':'8','구':'9','한':'1','두':'2','세':'3','네':'4','다섯':'5'}
def normalize(text):
 text=str(text).strip()
 if not text or len(text)>500:raise ValueError('명령은 1~500자입니다.')
 def node(m):return 'LM'+NUM.get(m[1],m[1])
 text=re.sub(r'(?:엘\s*엠|엘렘|엘림|lm)\s*([0-9]+|[일이삼사오육칠팔구])',node,text,flags=re.I)
 return re.sub(r'(?:씨\s*피|시\s*피|cp)\s*([0-9]+|[일이삼사오육칠팔구])',lambda m:'CP'+NUM.get(m[1],m[1]),text,flags=re.I)
def validate_command(value,nodes,operations,gui=None):
 if isinstance(value,dict) and value.get("action")=="gui":
  if gui is None:raise ValueError("GUI 기능 목록이 필요합니다.")
  return gui.validate(value)
 if not isinstance(value,dict) or set(value)-{'action','nodes','start','repeats','operation'}:raise ValueError('지원하지 않는 명령 항목입니다.')
 out=dict(value);action=out.get('action')
 if not isinstance(action,str) or action not in ACTIONS:raise ValueError('지원하지 않는 명령입니다.')
 if 'start' in out and not isinstance(out['start'],str):raise ValueError('출발 노드는 문자열입니다.')
 if 'operation' in out and not isinstance(out['operation'],str):raise ValueError('팔 작업명은 문자열입니다.')
 targets=out.get('nodes',[])
 if not isinstance(targets,list) or any(not isinstance(n,str) or n not in nodes for n in targets):raise ValueError('지도에 없는 노드입니다. 노드명을 다시 확인하세요.')
 if len(targets)>50:raise ValueError('한 번에 최대 50개 목적지입니다.')
 if action in ('goto','sequence','loop'):
  if not targets or action=='goto' and len(targets)!=1:raise ValueError('이동 목적지를 명확히 말하세요.')
  if out.get('start','') and out['start'] not in nodes:raise ValueError('출발 노드가 지도에 없습니다.')
 elif targets or out.get('start'):raise ValueError('이 명령에는 목적지가 필요하지 않습니다.')
 if 'repeats' in out and (action!='loop' or type(out['repeats']) is not int or not 1<=out['repeats']<=100):raise ValueError('반복은 1~100회입니다.')
 if action=='loop':out.setdefault('repeats',1)
 if action=='arm_action':
  if out.get('operation') not in operations:raise ValueError('등록된 로봇팔 작업만 실행할 수 있습니다.')
 elif out.get('operation'):raise ValueError('이 명령에는 팔 작업이 필요하지 않습니다.')
 return out
def parse_exact(text,nodes,operations):
 text=normalize(text);compact=re.sub(r'\s+','',text).rstrip('.!?。')
 if compact in ('정지','멈춰','멈춰줘','로봇정지','모두정지','그만움직여','스톱'):return {'action':'stop'}
 if re.search(r'(하지|하지말|하지마|아니|말고|금지)',compact):raise ValueError('부정/수정 표현은 실행하지 않습니다. 원하는 명령을 다시 말하세요.')
 if re.fullmatch(r'(?:현재)?(?:위치|상태|배터리|진행상황)(?:알려줘|말해줘|확인|조회)?',compact):return {'action':'status'}
 for action,pattern in [('cancel',r'(?:현재)?(?:미션|작업|주행)(?:을)?취소(?:해|해줘)?'),('pause',r'(?:미션|주행)?일시정지(?:해|해줘)?'),('resume',r'(?:미션|주행)?(?:재개|다시시작)(?:해|해줘)?')]:
  if re.fullmatch(pattern,compact):return {'action':action}
 if re.fullmatch(r'(?:로봇팔|팔)?안전(?:자세|위치)(?:로)?(?:이동|가|돌아가|복귀)(?:해|줘|해줘)?',compact):return validate_command({'action':'arm_action','operation':'safe_pose'},nodes,operations)
 for name in operations:
  if compact in (name+'실행',name+'실행해',name+'작업실행',name+'작업실행해'):return {'action':'arm_action','operation':name}
 ids=re.findall(r'(?<![A-Z0-9])(?:LM|CP)\d+(?![A-Z0-9])',text.upper())
 if ids and re.search(r'이동|가줘|가$|주행|돌아|순환|바퀴',text):
  if re.search(r'돌아|순환|바퀴|루프',text):
   match=re.search(r'([0-9]+|다섯|한|두|세|네|일|이|삼|사|오)\s*(?:번|회|바퀴)',text);count=int(NUM.get(match[1],match[1])) if match else 1
   return validate_command({'action':'loop','nodes':ids,'repeats':count},nodes,operations)
  if len(ids)==2 and re.search(re.escape(ids[0])+r'\s*에서',text,re.I):return validate_command({'action':'goto','nodes':[ids[-1]],'start':ids[0]},nodes,operations)
  return validate_command({'action':'goto' if len(ids)==1 else 'sequence','nodes':ids},nodes,operations)
 return None
def command_schema(nodes,operations,gui_targets=None):
 branches=[]
 for action in ACTIONS:
  if action=='arm_action' and not operations:continue
  properties={'action':{'type':'string','enum':[action]}};required=['action']
  if action in ('goto','sequence','loop'):
   properties['nodes']={'type':'array','items':{'type':'string','enum':list(nodes)},'minItems':1,'maxItems':1 if action=='goto' else 50};required.append('nodes')
  if action=='loop':properties['repeats']={'type':'integer','minimum':1,'maximum':100};required.append('repeats')
  if action=='arm_action':properties['operation']={'type':'string','enum':list(operations)};required.append('operation')
  branches.append({'type':'object','properties':properties,'required':required,'additionalProperties':False})
 if gui_targets:
  from .voice_gui import gui_schema
  branches.append(gui_schema(gui_targets))
 return {'oneOf':branches}
def ground_command(value,text):
 text=normalize(text);compact=re.sub(r'\s+','',text).lower();action=value['action']
 if action=='gui':
  proposed=value.get('value','')
  if proposed=='true' and not re.search(r'켜|활성화|ON|닫|흡착',text,re.I):raise ValueError('켜기 의도를 명확히 말하세요.')
  if proposed=='false' and not re.search(r'꺼|비활성화|OFF|열|해제',text,re.I):raise ValueError('끄기 의도를 명확히 말하세요.')
  if proposed and proposed not in ('true','false') and re.sub(r'\s+','',proposed).lower() not in compact:raise ValueError('말하지 않은 설정값을 LLM이 제안했습니다. 값을 정확히 지정하세요.')
  return value
 if action in ('goto','sequence','loop'):
  if any(n.lower() not in compact for n in value['nodes']):raise ValueError('말하지 않은 노드를 LLM이 제안했습니다. 목적지를 다시 말하세요.')
 elif action=='stop' and not re.search(r'정지|멈춰|그만|스톱',text):raise ValueError('정지 의도가 불명확합니다. 명령을 다시 말하세요.')
 elif action=='arm_action':
  name=value['operation'];aliases={'safe_pose':r'안전.*자세','pick_and_place':r'픽앤플레이스','door_open':r'문.*열|도어.*오픈','material_supply':r'자재.*공급'}
  if name.replace('_','').lower() not in compact.replace('_','') and not re.search(aliases.get(name,r'(?!)'),text):raise ValueError('팔 작업명을 정확히 말하세요.')
 return value
def llm_messages(text,nodes,operations,gui_targets=None):
 system='로봇 명령을 JSON으로 분류한다. 사용자에게 답하거나 코드 생성 금지. goto는 action,nodes 두 필드만 사용하며 nodes에 요청된 목적지 한 개. 출발 노드를 추측하지 말 것. sequence는 action,nodes. loop는 action,nodes,repeats. status,stop,pause,resume,cancel은 action만. arm_action은 action,operation만. 도착해서 기다리라는 명령은 goto. 배터리 질문은 status. 등록 노드: '+json.dumps(list(nodes),ensure_ascii=False)+'; 팔 작업: '+json.dumps(list(operations),ensure_ascii=False)
 if gui_targets:system+=' GUI 명령은 action=gui, target=목록 id, operation=invoke(버튼), set(값 입력), select(화면/목록), read(조회). value는 사용자 지정 값 그대로. 체크 켜기 true 끄기 false. 등록된 GUI 후보: '+json.dumps(gui_targets,ensure_ascii=False)
 return [{'role':'system','content':system}, {'role':'user','content':'목적지를 LM3로 잡고 출발해'}, {'role':'assistant','content':'{"action":"goto","nodes":["LM3"]}'}, {'role':'user','content':'배터리 잔량이 얼마나 남았어?'}, {'role':'assistant','content':'{"action":"status"}'}, {'role':'user','content':normalize(text)}]

def describe(command,real=False):
 names={'goto':'목적지 이동','sequence':'순서대로 이동','loop':'순환 미션','pause':'일시정지','resume':'미션 재개','cancel':'미션 취소','stop':'모두 정지','status':'현재 상태 조회','arm_action':'등록된 팔 작업 실행'}
 lines=['운영 모드: '+('실기 · 확인 후 실행' if real else '시뮬레이션'),'명령: '+names[command['action']]]
 if command.get('nodes'):lines.append('방문 목적지: '+' → '.join(command['nodes']))
 if command.get('start'):lines.append('출발 위치 확인: '+command['start'])
 if command['action']=='loop':lines.append('반복: '+str(command.get('repeats',1))+'회 · 시작 노드로 복귀')
 if command.get('operation'):lines.append('팔 작업: '+command['operation'])
 return '\n'.join(lines)
