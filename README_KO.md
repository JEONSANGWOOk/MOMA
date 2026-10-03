# SEER AMR Control Studio v1.9.1 Workspace

**LM2→LM7→LM8 복귀 경로 탐색 수정**: 경로 중간에서 장애물 가까이에 멈추었을 때 추가 계획 여유 거리 때문에 출발 노드 복귀 구간까지 탈락하는 문제를 수정했습니다. 실제 충돌 반경을 유지한 구간 검사로 안전한 기존 경로 복귀를 허용합니다. 양방향 연결에서만 되돌아가며, 일방통행 경로는 임의 역주행하지 않습니다.

**동적 장애물 상호 정지 해소 수정**: 메인의 `상호 정지 자동 해소 적용`으로 자동 판단의 기본 대응을 선택합니다. 정지한 사람·AMR을 정적으로 재분류한 뒤 설정 시간만큼 기다리고, 주변 우회 → 다른 연결 경로 탐색 → 모두 불가할 때 목적지 패스 순서로 진행합니다. 기존 `다른 경로 탐색`과 `동적 대기 / 정적 우회`도 등록 종류 대신 관측한 움직임·정지 시간을 반영합니다.

**신규 장애물 마킹 / 지속 추적**: 지도에 등록된 고정 장애물은 신규 추적에서 제외합니다. 새로 관측된 장애물에는 `TRK001` 형식의 추적 ID를 부여하고, 2D·3D 지도에서 위치·관측 궤적·동적/정적 판단을 갱신합니다. 동적·정적 판단이 바뀌어도 ID를 유지합니다. 메인의 `신규 장애물 추적 목록`에서 좌표·속도·관측 상태를 확인할 수 있습니다.

**자동 장애물 판단과 대응 시나리오**: 메인의 `주행 장애물 대응 · SIM`에서 `자동 판단 / 장애물별 시나리오`를 선택하세요. `자동 판단 / 장애물별 시나리오 설정`에서 동적·정적·판단 중 각각의 행동을 지정하고, 개별 장애물에 별도 시나리오를 지정할 수 있습니다. 기본은 동적·판단 중 대기, 정적 5초 대기 후 다른 연결 경로 탐색·불가 목적지 패스입니다.

**동적 장애물 다중 포인트 이동**: 메인의 `AMR 추가`에서 기존 노드를 여러 개 선택합니다. AMR은 중간 노드와 곡선을 포함한 실제 연결 경로를 따라갑니다. `사람 추가`에서는 좌표 포인트를 여러 개 등록합니다. 포인트 추가·삭제·순서 변경, 왕복·순환, 속도·등록 포인트 대기 시간을 설정할 수 있습니다. 왕복의 돌아오는 길과 순환의 마지막→첫 노드 연결도 검증합니다. 연결 경로가 없으면 저장하지 않습니다.

**정적 장애물의 다른 경로 탐색 / 목적지 패스**: 메인의 `주행 장애물 대응 · SIM`에서 `정적 대기 → 다른 경로 / 불가 목적지 패스`를 선택합니다. 기본 5초 대기 후 지도에 연결된 다른 경로를 찾으며, 다시 막히면 재탐색합니다. 탐색 실패가 기본 3회 반복되면 목적지와 해당 위치의 팔·I/O 작업을 생략하고 다음 목적지로 이동합니다. 2D·3D 빨간 마킹과 별도 이력에 이유를 남깁니다.


**우회 후 기존 경로 복귀**: 장애물 뒤의 가까운 안전 복귀점으로 돌아온 후 원래 직선·곡선을 이어갑니다. 안전한 복귀점을 찾지 못하면 기존 목적지 우회 계획을 시도하며, 이동 경로를 찾지 못하면 대기합니다.

**노트북 UI 배율**: 화면 상단 `UI 배율`은 기본 자동입니다. 창 크기에 맞춰 글자·여백·패널 폭을 조절하며 `65% / 75% / 85% / 100% / 115%`를 직접 선택할 수 있습니다. 수동 선택은 다음 실행에도 복원합니다. 오른쪽 제어 패널에서 버튼·문구 위에서도 휠로 스크롤할 수 있습니다. 지도 확대/축소와 UI 배율은 별도입니다. [1024 창 검증 화면](artifacts/ui_compact_1024.png).

**메인 주행 화면의 장애물 정책을 네 가지로 확장**하고 사람·AMR 동적 장애물을 추가했습니다. `지도 / 제어` 오른쪽 상단 **주행 장애물 대응 · SIM**에서 피하기 / 정지하기 / 대기 후 기존 경로로 진행 / 동적 대기·정적 우회를 선택합니다. 사람 추가와 AMR 추가로 시작·끝 좌표, 속도, 끝점 대기를 설정하고, 관리에서 수정/삭제합니다. 2D/3D와 SIM LiDAR에 이동 중 위치를 표시합니다. [사용 방법](docs/SIM_OBSTACLES_KO.md).

**FAIRINO FR5 공식 Python SDK 제어**를 추가했습니다. `확장 도구 → 로봇팔 / 장비 연결`에서 FR5 연결 설정과 연결 시험을 사용하세요. 기본 IP는 `192.168.58.2`입니다. MoveJ/MoveL 등록 작업, 현재 자세 저장, 미션 Arm Action, 상태/관절 수신과 3D 표시를 지원합니다. SDK는 WebAPP 버전에 맞게 별도로 준비해야 합니다. SIM에서는 등록한 MoveJ 관절 보간을 지원하며 MoveL은 차단합니다. [FR5 연결 순서와 ROS/ROS2 안내](docs/FR5_API_KO.md).

`지도 / 제어`에도 그리드·벽·영역·노드·경로·장애물·LiDAR 레이어 스위치를 표시합니다. 편집 지도와 같은 설정을 공유하며 3D 지도에도 적용됩니다. 지도 위의 상태줄에서 LiDAR 점 수/수신 지연과 장애물 감지·정지 상태를 확인합니다. 3D 시점의 **뒤따라 보기**는 AMR 위치와 방향을 따라 뒤쪽에서 추적하며, 기존 `로봇 추적`은 고정된 방향으로 위치만 따라갑니다. 3D에서도 LiDAR 점을 표시하고 실제 수신이 2초 이상 지연되면 오래된 점을 숨깁니다.

SIM에서 **벽 만들기**와 **장애물 대응: 정지/대기 또는 우회 주행**을 추가했습니다. `노드 / 경로`에서 두 점을 클릭해 벽을 만들고, `지도 / 제어` 우측에서 대응 정책을 선택합니다. SEER 차체 크기를 반영한 충돌 여유와 이동 구간 검사로 벽 통과를 막고, 우회가 불가능하면 대기합니다. 자동 우회 경로는 기존 초록색 이동 경로에 표시됩니다. [사용 방법과 범위](docs/SIM_OBSTACLES_KO.md).

**DualSense CFI-ZCT1G 조이스틱 수동 조종**을 추가했습니다. Windows에서 인식된 컨트롤러로 SIM과 실제 SEER AMR을 조종합니다. `지도 / 제어` 우측 수동 조작에서 장치 선택·입력 확인·축/버튼 설정 후 수동 조작과 조이스틱 사용을 켜세요. 기본 조작은 왼쪽 스틱 전후진/회전, L1을 누르는 동안만 주행, ○ 소프트웨어 정지입니다. 기존 속도 제한과 제어권·안전 자세 조건을 적용하고 연결 해제와 입력 지연 시 중단합니다. [연결 및 사용 방법](docs/DUALSENSE_KO.md).

3D 기본 모델을 **SEER AMB-CSW04-CE + FAIRINO FR5**로 변경했습니다. 흰색 SEER 차체와 흰색/주황색 6축 FR5를 경량 URDF로 표시하며 기존 SIM 주행·팔 미션과 연결됩니다. 제조사 정밀 CAD 메시가 아닌 도형 기반 시각화 모델입니다. `3D 모델 / 관절`에서 기본 모델 복원 또는 사용자 정밀 모델 교체가 가능합니다.

`지도 / 제어`에 **2D / 3D 전환**을 추가했습니다. 3D 공간에서 지도·곡선 경로·현재 AMR·기본 6축 팔을 함께 표시하며, 사선/위/정면/측면/뒤/로봇 추적 시점과 원근/직교 투영을 선택할 수 있습니다. 왼쪽 드래그로 회전, 오른쪽 드래그로 이동, 휠로 확대합니다. `3D 모델 / 관절`에서 AMR·팔 URDF 및 STL/OBJ CAD 가져오기, 관절 미리보기, 팔 장착 위치와 CAD 배치를 설정합니다. 기본 팔은 SIM 작업에 연동되며, 외부 URDF의 실제 동작에는 관절 상태 수신 연결이 필요합니다. STEP/IGES는 STL/OBJ로 변환해서 사용하세요. [사용 방법과 지원 범위](docs/3D_VIEW_KO.md), [전체 지도](artifacts/world3d_map.png), [AMR와 팔 확대](artifacts/world3d_robot.png).

순환 미션 만들기 창에도 독립 지도 미리보기를 추가했습니다. 노드를 클릭하면 선택하고 더블클릭하면 방문 순서에 추가합니다. 선택한 방문 노드 사이를 지도에 연결된 중간 노드·곡선 경로로 확장해 표시하며, 없는 연결을 대각선으로 만들지 않습니다. 추가·제거·순서 변경·시작점으로 닫기 결과가 지도에 즉시 반영됩니다. 노드의 숫자는 요청한 방문 순서이며, 상단 `실제 통과 경로`는 중간 노드까지 포함한 이동 순서입니다. 같은 Point 연속 선택은 이동 없이 처리합니다. 휠 확대·우클릭 이동·지도 맞춤·로봇 위치 표시를 지원합니다. [경로 선택 화면](artifacts/loop_route_planning.png).

경로 선택: `최단 거리`는 곡선 길이까지 합산한 최단 경로, `예상 시간 우선`은 경로 제한 속도와 로봇 모델 최대 속도를 적용한 거리/속도 비용, `우회 (제외 노드)`는 쉼표로 지정한 노드를 제외한 최단 경로입니다. 제외 노드는 빨간색 ×로 표시합니다. 꼭 지나갈 우회 경유지는 방문 순서에 넣으세요. 경로가 없거나 방문 노드를 제외하면 오류를 표시하고 미션 생성을 막습니다. 예상 시간은 가감속·회전·대기·장애물 정지를 제외한 참고값이며 실시간 교통 기반 예측은 아닙니다.

생성 미션은 **정지할 목적지 하나당 Path Nav Action 하나**로 구성하며, 계산된 중간 노드는 해당 Action의 `route_nodes` 통과 경로로 저장합니다. SIM은 중간 노드에서 종점 감속·도착 방향 정렬·대기를 하지 않고 통과 이력만 갱신하며, 선택 목적지에서 정지/방향 정렬/대기를 수행합니다. 일반 목적지 이동에도 같은 통과 처리를 적용합니다. 실기는 정지 목적지에만 이동 명령을 보내고 중간 Station에 별도 명령을 보내지 않습니다. 실기 컨트롤러가 경로를 재계획할 수 있으므로 최단/우회 미리보기의 전체 경로 고정은 별도 API 연동이 필요합니다. 시작 Point로 이동하는 SIM 준비 경로에도 선택 정책과 제외 노드를 적용합니다. 이전 버전에서 자동 생성한 노드별 분할 순환 미션은 수정하지 않은 경우 실행 시 목적지 단위로 변환합니다. 장애물/비상정지/일시정지와 같은 정지는 유지합니다.

RoboShop의 지도 작업 화면을 참고한 데스크톱 디자인으로 변경했습니다. 좌측 기능 탐색, 상단 연결/상태 표시줄, 중앙 지도와 우측 제어 패널, 하단 운행 상태를 배치했습니다. 지도·노드/경로·Taskchain·실시간 데이터·운영 정보·API·로그·확장 도구의 색상과 버튼 스타일을 통일했습니다.
화면 예시: [1366×768](artifacts/workspace_1366.png), [1920×1080](artifacts/workspace_1920.png).

워크스페이스 역할을 통합 정리했습니다. `지도 / 제어`에는 목적지 주행·일시정지/재개/취소·모터·수동 조작·재배치·현재 위치를 배치합니다. 지도 클릭은 목적지 선택만 수행합니다. `노드 / 경로`에는 로컬 JSON/SMAP 열기·저장, 로봇 지도 가져오기/적용, Stations 조회, 맵 생성/스캔 저장, 지도 표시 레이어, 노드·경로·영역 편집, 맵 검증과 AMR 업데이트를 모았습니다. 편집 화면의 더블클릭은 속성 편집이며, 목적지 운행은 운행 화면에서 수행합니다. 확대/맞춤은 두 지도에서 볼 때 필요한 보기 기능으로 유지합니다.

`노드 / 경로` 워크스페이스에서도 왼쪽 편집 지도와 오른쪽 노드·경로·영역 목록 및 속성을 함께 볼 수 있습니다. 지도/목록 선택이 연동되고 선택 경로는 파란색 화살표로 강조됩니다. 여기서 Point 추가·이동, 경로 연결, 곡선 제어점 드래그와 영역 편집을 직접 수행할 수 있습니다. 상단/우측 `선택 속성`은 선택한 항목 종류에 맞는 설정을 엽니다. 지도 / 제어 화면과 같은 지도 데이터를 사용하며 로봇 위치도 계속 갱신됩니다. [편집 화면 예시](artifacts/node_editor_1366.png).

노드 이름은 배경을 붙여 읽기 쉽게 표시하며, 이름/로봇 위치 문구가 겹치면 주변으로 옮깁니다. 확대가 부족해 배치 공간이 없는 이름은 숨겨지므로 확대하면 더 확인할 수 있습니다. 경로 진단은 왕복 경로마다 중복으로 그리지 않고 선택한 경로의 방향·속도만 지도 상단에 표시합니다.

실시간 위치는 우측 `실시간 위치 / 통과 노드` 패널에서 X/Y/방향과 `직전 도착 노드 → 다음 노드`, 최종 목적지, 다음 노드까지 직선거리로 표시합니다. 상단 현재 작업에도 노드 진행을 표시하고 지도 위 로봇에 현재/직전 노드를 붙입니다. 0.1초마다 상태를 반영합니다. SIM은 도착 이벤트로 갱신합니다. 실기는 보고된 last_station을 우선하며, 없는 경우 0.18 m 이내에서 0.3초 유지된 위치를 도착으로 추정합니다(이탈 기준 0.30 m). 추정 여부와 수신 경과 시간을 표시하며, 수신이 3초 넘게 끊기면 갱신 대기로 전환합니다. 다음 경유 노드 데이터가 없을 때에는 `목표`라고 표시합니다.

지도 편집·전체 Action 실행·DI/DO·장애물 시험·자동 충전·알람과 기록 재생·로봇 모델·보정·로봇팔 연결을 추가했습니다.
사용법과 실기 연결 범위: [확장 기능 안내](docs/EXTENDED_FEATURES_KO.md).
시험용 미션: [Extended SIM Demo](examples/extended_demo_taskchain.json).
실행 중인 이전 창을 종료하고 `run_windows.bat`으로 다시 실행하면 창 제목에 `v1.9.1 Workspace`가 표시됩니다. 기존 EXE에는 소스 변경이 자동 반영되지 않습니다.

목표: RoboShop 없이 미니PC에서 SEER AMR 상태/주행/맵 편집을 수행하고, 이후 로봇팔 제어를 같은 Taskchain에 통합할 수 있는 기반을 제공합니다.

## v0.6.1 핵심 추가

### 1. 실제 로봇 SMAP 편집
1. 실기 · 제어 모드로 연결
2. `Pull Map`
3. `노드/경로` 탭에서 `Point` 선택
4. 지도에서 원하는 위치 클릭 → LM 자동 생성
5. Point 앞 방향 핸들을 드래그해 도착 방향 설정
6. `Bezier/Path` 선택 → 시작 Point 클릭 → 끝 Point 클릭
7. Path 속성 확인/수정
8. `맵 검증`
9. `AMR 업데이트`

X/Y를 직접 입력하지 않고 지도 클릭 좌표를 사용합니다.

### 2. AMR 업데이트 파이프라인
`AMR 업데이트`는 다음 순서로 동작합니다.

- 현재 Pull한 원본 SMAP을 `~/.seer_amr_console/map_backups/`에 자동 백업
- 로컬 Point/Path 편집 내용을 원본 SMAP에 병합
- API 4010 / port 19207 로 전체 JSON SMAP 업로드
- API 4011로 같은 맵을 다시 다운로드
- 추가한 Point와 Path가 실제 로봇 저장본에 존재하는지 검증
- `업로드 후 현재맵 재적용` 체크 시 API 2022로 현재 맵 다시 적용
- 검증된 로봇 맵을 다시 GUI에 표시

**중요:** 로봇이 이동/Task 수행 중일 때는 맵 Push를 차단합니다. 최초 실기에서는 반드시 저속/안전구역에서 검증하세요.

### 3. SMAP 데이터 보존 정책
- `normalPosList`, 영역, 금지선 등 편집하지 않은 원본 레이어는 그대로 보존합니다.
- 기존 Point는 원본 object를 복제하여 위치/방향/속성만 갱신합니다.
- 신규 Point는 실제 Pull된 기존 LandMark/LocationMark 구조를 템플릿으로 사용합니다.
- 기존 Path는 원본 Bezier control point와 property를 보존합니다.
- 신규 Path는 기존 실제 Path를 템플릿으로 사용하고 시작/끝 Point와 control point를 갱신합니다.
- 기존 Path의 `direction`, `movestyle`, `maxspeed` 같은 property는 가능한 그대로 상속합니다.

### 4. 실시간 운용
- 1101 All2 기반 실시간 LiDAR/blocked 정보
- 1009 Laser fallback
- Station 선택 → 3051 Path Navigation
- 4005/4006 제어권 획득/해제
- 2010/2000 수동 Jog

## MoMa 구성 방향
미니PC가 상위 Controller 역할을 합니다.

```
Mini PC
 ├─ SEER AMR Adapter
 │   ├─ State / LiDAR
 │   ├─ Navigation
 │   ├─ Manual Jog
 │   └─ SMAP Pull/Edit/Push
 │
 ├─ Taskchain / Mission Manager
 │   ├─ AMR Path Nav
 │   ├─ Wait / I/O
 │   └─ Robot Arm Action (다음 통합 단계)
 │
 └─ Robot Arm Adapter
     └─ Fairino SDK/XML-RPC 등
```

AMR가 목표 LM에 도착한 뒤 로봇팔 작업을 실행하는 인터록(AMR 정지/도착 확인 → arm enable → arm task → arm safe pose → AMR next move)을 다음 단계에서 연결하는 구조입니다.

## 테스트
- Python compile PASS
- Unit tests: 45/45 PASS
- GUI smoke test PASS (simulation)

실제 SEER 펌웨어에서 4010/2022 맵 갱신 동작은 현장 실기 로그로 최종 확인이 필요합니다. 실패 시 이벤트 로그의 `MAP_PUSH` / `ERROR` 항목을 그대로 전달해 주세요.


## v0.6.1 LiDAR diagnostics
- 1101 All2 via persistent TCP session
- recursive firmware-variant laser parser
- 1009 persistent fallback
- map footer shows LiDAR source, point count and RX age
- periodic SENSOR/LASER diagnostics in event log

## v0.7.9 LiDAR lasers[] parser fix
- 1101/1009 `lasers` 리스트 내부의 센서 객체를 재귀적으로 탐색합니다.
- beams / beam_points / points / pointcloud / ranges 형식을 추가 지원합니다.
- 파싱 실패 시 `lasers shape=...` 진단 로그를 남깁니다.


## v0.7.9 Push Map 실제 연결 수정
- 상단 Push Map 버튼이 unavailable placeholder에 연결되어 있던 버그 수정: 실제 AMR 업데이트 함수에 연결.
- 4010/2022 ret_code 검증 추가.
- 4011 재다운로드로 신규 Point/Path 저장 여부 검증.
- 2022 후 1301 Station 재조회로 활성 맵 반영 여부 2차 검증.
- 전송 직전 edited_to_push.smap 파일 자동 보존.


## v0.7.9 fixes
- Dual/multi LiDAR mounting pose compensation: each lasers[] scan uses its own x/y/yaw when present.
- RoboShop-style rays are drawn from each physical laser origin, not chassis center.
- 4010 verification now inspects raw 4011 advancedPointList/advancedCurveList directly.
- LocationMark is accepted during SMAP parsing and existing SMAP className is preserved.
- 4011 round-trip SMAP is saved for diagnostics.

## v0.7.9 변경사항
- 4010이 기존 mapName을 덮어쓰지 않는 펌웨어 대응: 저장 검증 실패 시 새 mapName으로 clone 업로드 후 4011 검증, 이후 2022로 전환.
- lasers[].beams는 센서 장착 pose가 응답에 없을 경우 map cloud와 scan을 이용해 센서별 x/y/yaw 자동 정합을 수행하고 사용자 폴더에 저장.
- 자동 정합 결과는 이벤트 로그에 `RoboShop 정합 자동보정 완료`로 기록.


## v0.7.9 Path direction editor
- 경로 생성/편집 시 통행 방향을 `양방향`, `A → B 단방향`, `B → A 단방향`으로 선택합니다.
- 양방향은 SMAP에 반대 방향 AdvancedCurve 2개로 저장하고, 단방향은 선택 방향 1개만 저장합니다.
- `direction`은 통행방향과 별개인 로봇 주행방향(전진=0/후진=1)으로 편집합니다.
- Path 목록에서 통행방향, 주행방향, movestyle, maxspeed를 표시하며 더블클릭으로 전체 속성을 편집합니다.
- 지도 위 경로 화살표: 단방향은 한쪽 화살표, 양방향은 양끝 화살표입니다.
- maxacc/maxdec/maxrot/maxrotacc/maxrotdec/reachdist/reachangle/width/length/holdDir/장애물 거리 속성을 편집하고 Push Map에 보존합니다.

## v0.7.9 Advanced Area
- RoboShop style Advanced Area editor added.
- Map/Control or Node/Path tab: choose `Advanced Area`, then drag on the map to create a rectangular work zone.
- Area properties saved to SMAP `advancedAreaList`: forbidden, maxspeed/maxacc/maxdec, maxrot/maxrotacc/maxrotdec, obsDecDist, obsStopDist, obsExpansion, ultrasonic, fallingdown, infrared, weight, collisionPointThreshold.
- Pulled RoboShop Advanced Areas are displayed and editable; Push Map round-trips them back to the robot.
- Area list displays max speed, deceleration distance, stop distance, obstacle expansion, forbidden state.
- Path/Area priority follows the robot map engine. If both exist, the controller applies the safer/more restrictive value according to SEER behavior.


## v0.7.9 LiDAR ↔ Map 정합 / 실기 SLAM / 충전 복귀
- `LiDAR 정합`: 현재 실시간 LiDAR point cloud와 4011 SMAP 정적 점군의 2D rigid transform(dx/dy/yaw)을 자동 탐색합니다. 결과는 host+map별로 저장됩니다. 이 보정은 **화면 표시용**이며 로봇의 실제 localization/navigation 좌표를 변경하지 않습니다.
- 지도 Pull 완료 후 자동 정합을 1회 시도하고, 필요하면 상단 `LiDAR 정합` 버튼으로 다시 수행할 수 있습니다.
- 실기 맵 생성 시작: Peripheral API `6100` (port 19210).
- 실기 맵 생성 종료: Peripheral API `6101` (port 19210). 종료 ACK 후 현재 맵을 다시 Pull합니다.
- 충전 노드 복귀: 현재 1301 Station 중 `ChargePoint`/dock/CP 계열 노드를 찾아 가장 가까운 충전 노드로 `3051` Path Navigation을 전송합니다. 충전 Relay(6005)를 직접 켜지는 않습니다.


## v0.7.9 UI 보강
- 맵 생성 시작 즉시 기존 맵/노드/경로를 숨기고 빈 SLAM 작업 화면으로 전환합니다.
- Point 속성 창을 620x620으로 확대하고 세로 스크롤을 지원합니다.
- 저해상도 미니PC에서도 Point Type, Spin, 위치/방향 안내, 적용/삭제 버튼을 모두 확인할 수 있습니다.


## v0.7.9 SLAM LiDAR fix
- SLAM canvas draws obstacle hit endpoints only; no radial rays.
- Filters explicit range_max and repeated maximum/no-return beam plateaus.
- Logs per-laser raw/hit/drop/min/max/cutoff as SLAM_LASER for field diagnosis.
- Keeps scans collected before 6100 ACK instead of clearing them on ACK.


## v0.7.9
- 일반 맵 LiDAR 표시를 SLAM과 동일한 obstacle hit point 전용으로 통일
- sensor ray/360도 방사선 제거
- max-range/no-return beam 제외 유지
- 일반 맵 LiDAR hit point에 저장된 LiDAR↔Map 정합 transform 적용
- Map 레이어의 별도 장애물(blocked) 표시 및 SIM 장애물 버튼 제거
- blocked/block_reason 데이터는 진단/로그용으로 내부 수신 유지

## v0.7.9 추가 변경
- LiDAR는 `lasers[].beams(angle, range)`에서 no-return/max-range를 제거한 실제 반사 beam만 표시합니다.
- 유효 beam은 센서 원점에서 hit point까지 실제 측정거리만큼 선으로 표시되므로 가까운 장애물은 짧게, 먼 벽은 길게 보입니다.
- 일반 맵과 SLAM 화면이 동일한 유효-hit beam 정책을 사용합니다.
- 자기위치 재배치 UI 추가:
  - 수동 재배치: 맵 클릭 + 드래그 → 2002 `{x,y,angle}`
  - 자동 재배치: 맵 대략 위치 클릭 → 2002 `{x,y}` (angle 생략, RBK 2π 방향 탐색)
  - 위치 확정: 2003 ConfirmLoc
  - 취소: 2004 CancelReloc


## v0.7.9 LiDAR Source Fix
- 공식 1101 `laser_beams`가 있으면 최우선 사용합니다. 이 필드는 SEER 문서상 Map/World 좌표계 X,Y입니다.
- 공식 WORLD 포인트에는 mount 보정/화면 정합 dx,dy,yaw를 재적용하지 않습니다.
- `lasers[].beams`는 공식 `laser_beams`가 없을 때만 fallback으로 사용합니다.
- 화면 하단 source에 `1101.laser_beams(WORLD)` 또는 `1009.laser_beams(WORLD)`를 표시합니다.


## v0.8.0 Obstacle Display
- Normal view: Raw LiDAR OFF by default.
- New 장애물 layer: uses 1101 blocked/block_reason/block_x/block_y.
- When blocked, shows controller obstacle marker and nearby LiDAR hit cluster (0.55 m) only.
- Raw LiDAR remains available as an optional diagnostic layer.

## v0.9.1 Dual LiDAR Static + Dynamic Obstacle Display
- 1101의 lasers[0]/lasers[1] 유효 hit를 install_info로 월드 좌표 변환 후 하나의 merged scan으로 사용합니다.
- 장애물 레이어에서 기존 Map과 일치하는 고정 장애물(벽/기둥/설비)도 제거하지 않고 표시합니다.
- 고정/기존 Map 일치 hit: amber, Map에 없는 동적/미등록 hit: red.
- blocked는 장애물 표시 조건이 아니라 별도의 controller stop marker로만 사용합니다.
- 실시간 Mapping은 동일한 merged dual-LiDAR hit 전체를 누적하여 map cloud를 생성합니다.

## v0.9.2 로봇 운영 정보
- `로봇 운영 정보` 탭 추가
- 실기 API 1100 raw telemetry를 기반으로 실행/배터리/모션/내비게이션/기본정보를 실시간 표시
- 1101의 고속 위치/LiDAR 상태는 기존 방식으로 병행 수신
- 펌웨어별 필드명 차이는 여러 alias를 허용하며, 매칭되지 않는 값은 `—`로 표시
- 전체 원본 값은 `실시간 데이터` 탭의 raw.all에서 확인 가능

## v0.9.6 RoboShop 운영정보 확장
- 1100/1101 외에 상태 API 1000(로봇 정보), 1002(실행 상태), 1007(배터리), 1020(Task)를 별도 저속 폴링합니다.
- 1021/1022는 대상 펌웨어가 지원할 경우 위치 재배치/지도 로딩 상태를 추가 표시하며, 미지원이어도 프로그램은 계속 동작합니다.
- API 1007의 voltage/current/battery_temp/charging 값을 운영정보 화면에 우선 적용합니다.
- 각 개별 API 응답을 raw 상태에 보존하여 1100 갱신 때 사라지지 않도록 했습니다.
- 운영정보 화면 하단에 실제 수신된 개별 API 번호를 표시합니다.


## v0.9.6 추가 변경
- 기준: 사용자 제공 v0.9.3 RoboShopOperationInfo / 최신 LiDAR 수정 유지
- 기존 Point X/Y/Angle/Spin/Type/Description/Name 편집
- Point rename/move 시 연결 AdvancedCurve 참조/끝점 동기화
- A→B, B→A를 독립 단방향 Path 레코드로 표시/편집
- 방향별 전진/후진(direction), 속도/가감속/장애물 거리 독립 설정
- 선택 방향만 삭제, 역방향 Path 추가 지원


## v0.9.6 Point 속성 편집 UI 수정
- Windows에서 Point 입력창이 하단으로 몰려 보이지 않던 Tkinter parent 지정 버그 수정
- Name / Point Type / Description / X / Y / Angle / Spin을 한 행씩 직접 편집 가능
- 적용 / 취소 / Point 삭제 버튼을 창 하단에 고정
- Node ID 변경 시 연결 Path 참조 갱신, X/Y 변경 시 연결 Curve 끝점 동기화 유지
- v0.9.4의 최신 LiDAR 및 운영 정보 기능은 변경하지 않음


## v0.9.6 순환 미션
- 최신 v0.9.5 LiDAR/Point 편집 기반 유지
- Map/Control 우측과 Taskchain 상단에 `순환 미션 만들기` 추가
- Point 순서를 `LM3 → LM4 → ... → LM3` 형태로 등록
- 시작 Point 자동 닫기, 단방향 Path 존재 여부 사전 검증
- 반복 횟수 설정(0=무한), 각 Point 도착 후 대기시간 설정
- 실기: 3051 목적지 명령을 순차 전송하며 1020/상태의 RUNNING→COMPLETED와 도착 거리로 다음 Step 진행
- 마지막 Point 도착 후 다음 Cycle 자동 시작
- `미션 정지` 시 Cancel Nav 전송
- 생성된 순환 미션은 일반 Taskchain JSON으로 Save/Load 가능

## v0.9.8 Responsive UI
- 업로드된 v0.9.6 LoopMission 소스를 기준으로 반응형 화면 레이아웃 적용.
- 프로그램 시작 시 현재 모니터 해상도를 읽어 창 크기를 자동 결정.
- 고정 1440x920 의존 제거, 1366x768 / 1920x1080 이상에서 화면을 더 효율적으로 사용.
- Map/Control의 지도 영역은 창 크기에 맞춰 자동 확대/축소.
- 우측 제어 패널은 세로 스크롤을 지원하여 작은 화면/Windows 125~150% 배율에서도 하단 버튼 접근 가능.
- 상단 Map/Control 리본과 Map Editor 도구막대를 2행으로 분리해 좁은 화면에서 버튼 잘림 완화.
- 노드/경로 탭의 우측 Point 속성 패널에도 세로 스크롤 적용.
- Treeview 일부 컬럼은 남는 폭을 자동 사용.
- 주요 팝업 창은 모니터 해상도에 맞춰 자동 크기 제한/중앙 배치.
- 기존 LiDAR, SLAM, Relocate, Advanced Area, 방향별 Path, 순환 미션 기능 유지.


## v0.9.8 마지막 위치 복구 / 시뮬레이터
- 1초 주기 및 정상 종료 시 마지막 map/x/y/theta/confidence 저장
- 현재 Map과 저장 Map이 다르면 자동 재배치 차단
- 실기: 저장 Pose로 2002 Reloc, Confidence 기준 확인, 옵션 시 2003 자동 확정
- SIM: 전원 OFF/재부팅 시뮬레이션과 저장 Pose 복구
- 현재 위치에서 접근 가능한 가장 가까운 노드 검색/이동
- 옵션: 위치복구 후 순환미션의 가장 가까운 미션 노드에서 재개