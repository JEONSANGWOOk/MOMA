# AMR Control Studio v1.0 Extended

실행: `run_windows.bat`. 창 제목의 `v1.0 Extended`와 실시간 데이터의
`source_version: studio-v1.0`으로 수정된 소스 실행 여부를 확인합니다.
기존 EXE에는 이 소스 변경이 들어 있지 않습니다. EXE는 `build_native.py`로 다시 빌드해야 합니다.

## 1~14 기능과 구현 범위

| 번호 | 기능 | 사용 방법 / 구현 범위 |
|---|---|---|
| 1 | 경로 속성 시뮬레이션 | Path와 Area의 maxspeed/maxacc/maxdec/maxrot 반영, 곡선 길이 기반 경로 비용, 전진/후진 방향, 도착 방향과 Spin. 모델 maxrot은 rad/s, 기존 Path/Area 속성 maxrot은 deg/s. |
| 2 | 장애물·금지구역 | SIM 장애물 클릭 배치/삭제, 가상벽·금지영역·기존 벽에 대한 원형 충돌 반경 검사. 전방 경로를 검사해 감속·정지하고 제거 후 재개. 자동 우회 경로 생성은 포함하지 않음. |
| 3 | 전체 Taskchain | Path Nav, Translation, Rotation, Wait, Wait DI Trigger, Set DO, Branch DI, Custom Action, Arm Action 순차 실행. 체크되지 않은 Task/Group 제외. |
| 4 | DI/DO | 확장 도구 → DI/DO. SIM 64채널 입력 토글·출력 확인. 실기는 검증된 read_io/set_do 매핑 사용. 3초 이상 오래된 실기 DI는 판단에 사용하지 않음. |
| 5 | Undo/Redo | 확장 도구 → 지도 도구, Ctrl+Z / Ctrl+Y. 지도 변경 전후 이력 저장. 로봇에 이미 업로드된 지도는 자동으로 되돌리지 않음. |
| 6 | 경로 진단 | 지도에 경로 방향·속도·샘플 개수 표시. 확장 도구에 화면 지도와 SIM 지도 일치 여부, 적용 속도, 정지 이유 표시. |
| 7 | 편집 편의 | Point 이동 모드에서 드래그, 좌표 스냅, Point/Path Ctrl 다중 선택 후 일괄 속성 변경, 선택 경로 직선화. |
| 8 | 지도 정리 | 점군 지우개 반경 0.3m, 0.03m 격자 중복점 제거, 가상벽 두 점 클릭, 다각형 꼭짓점 클릭 후 Enter 또는 완성 버튼. 가상벽은 SMAP 내보내기 시 얇은 금지 Advanced Area로 변환. |
| 9 | 미션 오류 처리 | Action timeout_s, DI 조건 분기, 실패 이유, 일시정지/재개, 실패 단계부터 수동 재실행. 장비 명령 자동 재전송 없음. |
| 10 | 자동 충전 | 시작/재개 배터리 기준과 SIM 충전 속도 설정. SIM은 연결 가능한 충전 노드 이동 후 중단 목적지 복귀. 실기는 미션 단계 사이에 충전 노드 이동 후 charging·배터리 상태 확인. 실기 충전 이동 180초, 충전 확인 3600초 제한. |
| 11 | 알람·운행 기록 | 연결/장애물/정지/저전압/미션 실패 및 raw errors/warnings/fatals 이력. 0.2초 운행 기록, JSON 저장·로드, 타임라인·배속 재생. 보라색 재생 로봇은 시각화이며 제어 명령을 전송하지 않음. |
| 12 | 로봇 모델 | 길이/폭/충돌반경/속도/가감속/회전속도/거리배율/LiDAR 설치 위치의 로컬 편집·저장·JSON 입출력. SIM 모델로 적용. 실기 read_model/write_model은 검증된 장비별 매핑 필요. RoboShop 원본 모델 파일과 동일한 형식은 아님. |
| 13 | 보정 | 최소 2개의 대응점으로 2D LiDAR 평행이동/회전과 RMS 계산. 명령거리/측정거리로 주행 배율 계산. 로컬 모델 적용. 실기 calibrate는 검증된 매핑 사용. 제조사 자동 기구학 보정 알고리즘을 재현한 것은 아님. |
| 14 | 로봇팔 연동 | SIM 작업시간/진행률. 실기는 설정된 XML-RPC execute/status/stop 메서드와 완료값 사용. AMR 정지 확인 후 작업하며, work 후 safe_pose 완료 전에는 AMR 이동 차단. |

## 빠른 시험

1. `Taskchain` → `Load` → `examples/extended_demo_taskchain.json`.
2. 미션을 시작하면 LM2 이동 후 DI 0의 ON을 기다립니다.
3. `확장 도구 → DI / DO`에서 채널 0을 ON으로 설정합니다.
4. DO 1 설정, 직진, 회전, 대기, 팔 work → safe_pose, LM3 이동을 확인합니다.
5. `확장 도구 → 미션 실행`에서 모든 Action의 상태와 시간 제한을 확인합니다.
6. 장애물 시험은 `SIM 장애물` 모드로 경로에 원을 배치한 뒤 이동합니다. 주행 중에도 배치·제거할 수 있습니다.
   멈춘 후 배치 원을 클릭해 제거하면 기존 이동이 재개됩니다.

경로·Point·영역 편집은 주행 중에 차단됩니다. SIM 장애물과 DI 입력은 미션 실행 중에도 바꿀 수 있습니다.
직진/회전 미션의 속도는 기존 Jog 한도인 0.30m/s와 0.60rad/s로 제한합니다.
SIM은 경로 샘플을 따라 이동하는 2D 시험 모델이며 실제 SRC 주행 엔진과 같은 모델은 아닙니다.

## Action 속성

Taskchain의 Action을 선택해 `선택 Action 속성 편집`에서 JSON 값을 편집합니다.

```json
{"type":"Wait DI Trigger","channel":0,"value":true,"timeout_s":30}
```

```json
{"type":"Branch DI","channel":0,"value":true,"target":5,"timeout_s":10}
```

target은 현재 실행 미션의 1부터 시작하는 단계 번호입니다.
조건이 맞으면 해당 단계로, 아니면 다음 단계로 진행합니다.
실패 단계 재개는 장비 동작이 이미 수행되었을 수 있음을 확인하고 수동 선택해야 합니다.

SIM Custom Action은 `set_di`, `set_battery`, `clear_obstacles`를 지원합니다.
실기 Custom Action은 장비 연결 설정의 operations에 등록한 이름만 실행합니다.

## 실기 연결 설정

`확장 도구 → 로봇팔 / 장비 연결 → 설정 예제 보기`에서 양식을 가져옵니다.
API 번호·포트·응답번호·payload는 대상 SRC 펌웨어 문서에서 확인해 입력합니다.
예제의 0번 API는 미등록 표시이며 전송되지 않습니다.

- `read_io`: role=read, DI/DO 응답 경로 지정. 배열 또는 채널번호를 키로 가진 객체 지원.
- `set_do`: role=write, `${channel}`, `${value}`를 payload에 사용.
- `read_model` / `write_model`: 로봇 모델 조회·전송. `${model}` 또는 `${length}` 등의 값 사용.
- `calibrate`: 장비에서 제공하는 보정 API 요청. 설정된 payload 사용.
- 각 operation은 `verified: true`일 때만 사용 가능.
- XML-RPC에는 endpoint, execute_method, status_method, stop_method, operations가 필요.
- 구조화된 팔 응답은 success_path/status_path로 내부 필드를 선택할 수 있음.
- `operations.work`, `operations.safe_pose`는 실제 메서드에 전달할 인수 배열.

`verified`는 사용자가 대상 문서·장비로 확인했다는 설정입니다. 소프트웨어가 펌웨어 호환성을
인증하는 값이 아닙니다. 실기 자동 충전은 charging 응답이 확인되지 않으면 실패 처리하며,
확인되지 않은 충전 릴레이 명령을 자동으로 전송하지 않습니다.

설정 위치: 사용자 홈의 `.seer_amr_console/studio_settings.json`.
지도 속성·장애물·영역·가상벽은 지도 JSON에도 저장합니다.
기록과 알람은 저장 버튼에서 선택한 경로에 저장됩니다.

## 개발 검증

```text
py -3 -X utf8 -m unittest discover -s tests
py -3 -X utf8 tools/studio_smoke.py
```

GUI smoke는 임시 설정 폴더만 사용하며 실제 로봇에 연결하지 않습니다.
초록색 캔버스 경로의 곡률, 확장 탭 생성, 모든 SIM Action 실행, Undo/Redo,
보정 설정 저장을 확인합니다. 실기 TCP/XML-RPC는 모의 응답 테스트를 포함하며
실제 AMR·로봇팔 하드웨어 시험은 수행하지 않았습니다.
