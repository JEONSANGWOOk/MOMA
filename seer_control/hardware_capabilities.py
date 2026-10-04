"""Explicit implementation audit, not a claim of hardware compatibility."""
import json

FEATURES=[
 ('2D/3D 지도·로봇 표시','지원','실제 상태 표시 연동','지도 좌표/로봇 모델/센서 정렬 확인'),
 ('현재 위치·통과 노드','지원','위치 API 연동','실기 지도 노드와 좌표 일치'),
 ('목적지 주행','지원','3050/3051 API 선택 연동','대상 펌웨어에서 응답/도착 확인'),
 ('미션·루프·일시정지·취소','지원','미션 실행기/API 연동','3001/3002/3003 및 도착 상태 확인'),
 ('미션 보고서·이벤트','지원','실기 상태 이력 연동','위치/Task/BLOCKED/오류 상태 수신'),
 ('지도 업로드·로드','지원','4010/4011/2022 연동','지도 백업/SMAP 호환성 확인'),
 ('노드·Bezier 경로 편집','지원','SMAP 업로드 연동','곡선 형상은 제어기 설정/지도 반영 후 확인'),
 ('통과 노드 무정차','지원','제어기 동작 확인 필요','정차/회전/경로 연속 주행 설정/API 확인'),
 ('키보드·DualSense 조그','지원','2010/2000 및 상태 인터록','수동 모드/제어권/제어기 충돌 정지 확인'),
 ('수동 장애물 정지','PC 궤적 검사','제어기 BLOCKED 기반 차단','독립 PC LiDAR 궤적 검사·실기 검증 추가 필요'),
 ('최적 경로·자유 공간 우회·복귀','PC 알고리즘','PC 알고리즘은 SIM 전용','3050 제어기 자유 주행 연결; PC 복귀 알고리즘은 별도'),
 ('정체 탈출·목적지 패스·다음 목표','PC 알고리즘','SIM 자동 회복은 실기 미연결','명령 취소 완료/실패 사유/다음 주행 동기화 필요'),
 ('신규 장애물 추적·동적 판단','SIM 관측 기반','실기 추적 미구현','실제 LiDAR 분할/지도 제거/추적·좌표 검증 필요'),
 ('장애물별 자동 대응 설정','PC 시나리오','SIM 설정만 변경','제어기 설정 API 또는 검증된 외부 제어기 필요'),
 ('벽·장애물 생성/삭제','시험 장면 지원','가상벽은 지도 전송 대상','실제 물체 생성/삭제는 장비 제어 기능이 아님'),
 ('사람·다른 AMR 왕복 생성','시험 장면 지원','가상 이동체는 SIM 전용','실제 다른 AMR 제어에는 별도 연결/제어권 필요'),
 ('자동 충전·미션 복귀','지원','충전 노드/상태 기반 연동','실제 도킹·charging·배터리 회복 확인'),
 ('DI/DO·외부 장비','지원','등록한 장비 API 사용','장비별 read_io/set_do API 및 응답 설정'),
 ('FR5 로봇팔 작업','SIM MoveJ; 프로그램 편집/검증','SDK 이동·I/O·작업 프로그램 연동','FR5 펌웨어에 맞는 SDK/작업/상태 피드백 검증'),
 ('ROS 명령','설정/연동 범위 확인','범용 ROS 주행 브리지 미구현','ROS 버전/토픽·Action/제어기 인터페이스 필요'),
]

def controller_identity(raw):
 sources=[raw.get(k,{}) for k in ('api_1000','api_1002','all')] if isinstance(raw,dict) else []
 result={}
 for name,keys in [('model',('robot_model','model','robot_type')),('version',('robokit_version','robot_version','software_version','version'))]:
  for source in sources:
   if not isinstance(source,dict):continue
   value=next((source[k] for k in keys if isinstance(source.get(k),(str,int,float))),None)
   if value is not None:result[name]=str(value);break
 return result

def real_state_event(state):
 # State changes, not continuously changing position/speed or guessed trajectories.
 data={k:state[k] for k in ('task','task_status','blocked','emergency','motor','charging','errors','warnings','fatals') if k in state}
 return json.dumps(data,ensure_ascii=False,sort_keys=True,default=str)
