# 통합 화면 안내

권장 실행은 `D455_Key_Panel_Twin.cmd`입니다. 같은 창에서 SIM / REAL / SIM+REAL을 선택합니다. [통합 실행 설명](D455_KEY_PANEL_TWIN_KO.md)을 참고하세요. 아래 보정·SDK 조건은 통합 화면에도 동일하게 적용되며 기존 별도 콘솔은 유지합니다.

# FR5 실제 로봇과 D455 열쇠 작업

주소: **192.168.57.2**. `D455_Key_Panel_Real.cmd`로 실기 창을 열거나 기존 판넬 SIM 화면의 **FR5 실기 연결** 버튼을 누릅니다. 실행 직후는 미연결 상태이며, 연결 버튼은 상태를 읽습니다. 모터 활성화, 운전 모드 변경, 원점 복귀, 그리퍼 출력은 자동으로 실행하지 않습니다.

## 먼저 실제 자세를 읽기

1. 로봇과 노트북을 같은 LAN에 연결합니다. 로봇이 192.168.57.2/24라면 연결한 노트북 유선 어댑터에 사용하지 않는 192.168.57.10/24 같은 주소를 설정합니다. 중복 주소는 피합니다. 확인: PowerShell `Test-NetConnection 192.168.57.2 -Port 20003`.
2. 제어기의 실제 버전에 맞는 공식 Python SDK `windows` 폴더를 선택합니다. 이 노트북에는 SDK 2.2.8 / 제어기 3.9.8용 코드를 `.delivery/fairino-sdk/windows`에 준비했습니다. 로봇 버전이 다르면 해당 버전 SDK를 선택해야 합니다. 새 환경 설치: `py -3 tools/install_fairino_sdk.py --tag v2.2.8_robot_v3.9.8`. 설치는 파일 준비만 하고 로봇에 연결하지 않습니다.
3. **연결 · 상태 읽기**를 누릅니다. 실제 관절, 실제 TCP 좌표, 현재 TCP/워크 좌표 번호, 정지·오류 상태와 힘/토크 데이터를 표시합니다. 3D 로봇은 SDK가 읽은 관절각으로 갱신됩니다. 미연결 화면은 초기 모델이며 실제 자세로 해석하지 않습니다.

SDK 연결은 별도 프로세스에서 실행하며 GUI는 비동기로 응답을 받습니다. 연결 실패 시 UI가 재접속 루프에 갇히지 않습니다. 현재 노트북에서 개발 검증 시 20003 접속은 시간 초과였으므로 실제 로봇 상태/움직임은 확인하지 못했습니다.

## 보정 파일

실기 동작에는 `config/fr5_key_calibration.template.json`을 복사해서 **측정한 값**을 입력합니다. 실기 창의 **보정 템플릿 생성**은 `.delivery/fr5_key_calibration.json`을 만들며 기존 파일을 덮어쓰지 않습니다. SIM 치수/가상 기준을 불러오는 경로는 없습니다.

- `camera_mount`: `fixed_external`. 작업대에 고정된 D455 전용입니다. 카메라를 손으로 움직이면 이 보정이 유효하지 않습니다.
- `T_base_camera_mm`: 카메라 좌표를 로봇 베이스 좌표로 옮기는 4×4 강체 행렬. 회전 3×3, 이동 **mm**, 마지막 행 `[0,0,0,1]`. 카메라 원본 위치의 m는 코드에서 한 번 mm로 변환합니다.
- `T_board_socket_mm`: 두 마커 기준판에서 슬롯 입구 원점으로의 4×4 행렬. 슬롯 +Z는 판넬 밖으로 향합니다. 위치와 각도는 실물에서 보정합니다.
- `T_socket_key_zero_mm`: 슬롯 원점에 열쇠 끝 TCP를 정렬했을 때의 자세. 원점 이동은 0, 열쇠 TCP +Z는 슬롯 -Z 방향이어야 합니다. 회전은 키 날이 슬롯에 맞는 실제 방향으로 보정합니다.
- `board_revision`: 기존 D455 기준판 등록 JSON의 revision. **measured** 치수로 등록한 두 마커만 허용합니다. 기존 깊이 추정값은 실기에 사용할 수 없습니다.
- `tool`: 제어기에 교시한 **열쇠 끝 TCP** 번호 1~14. `user`는 베이스 좌표 0. 현재 제어기의 실제 TCP와 user 번호도 동일해야 합니다. 플랜지 TCP에 SIM의 +165 mm를 자동으로 붙이지 않습니다.
- `workspace_min_mm`, `workspace_max_mm`: 실제로 경로가 확보된 열쇠 끝의 베이스 좌표 작업 범위. 전체 팔의 장애물 모델을 대체하지 않습니다.
- `standby_mm`, `insert_mm`, `turn_deg`: 실제 슬롯/열쇠에 맞춘 대기 거리, 삽입 깊이와 회전 각도. 22 mm / 90°를 자동 입력하지 않습니다.
- `verified`, `key_tcp_confirmed`, `controller_version`, `version_confirmed`: 실제 보정과 SDK 호환성을 확인한 뒤 입력합니다. true로 변경하는 것 자체가 보정을 수행하지 않습니다.

카메라→베이스 보정, 열쇠 TCP 교시, 마커→슬롯 보정 값이 없으면 연결·자세 읽기까지 진행합니다. 보정 파일은 연결 해제 상태에서 불러오고 다시 연결합니다. **슬롯 목표 계산 · 전송 없음**으로 베이스 좌표를 확인할 수 있습니다.

## 실기 정렬과 접촉 동작

**실기 정렬·접근 시작**은 최근 D455 영상과 실제 TCP 피드백을 사용합니다. 삽입 옵션을 끄면 슬롯 5 mm 앞에서 멈추고 추종합니다. 명령은 최대 5 Hz의 작은 비동기 `MoveL` 보정입니다. 단일 이동은 위치 0.5 mm / 회전 0.25° 이내이며, 이전 명령의 실제 도달을 확인합니다. 고주기 `ServoCart` 구현이나 산업용 실시간 제어 주기를 보장하는 기능은 아닙니다.

삽입·회전 옵션을 켜려면 `contact_verified`, `force_sensor_verified`, `controller_guard_verified`와 실물에 맞는 `force_limit_n`, `torque_limit_nm`가 필요합니다. 실제 힘센서 데이터를 읽을 수 있어야 합니다. 제어기의 충돌/접촉 보호는 현장에서 설정·검증해야 하며 프로그램은 이를 자동으로 활성화하거나 영점 보정하지 않습니다. 힘/토크 한계를 넘으면 정지를 요청합니다. 이 기능은 힘 제어·컴플라이언스 제어가 아니라 위치 보정과 힘/토크 감시입니다.

실제 삽입 깊이와 축 정렬을 확인한 뒤 열쇠 끝을 회전 중심으로 회전합니다. 최종 `HOLD`는 실제 TCP가 목표에 도달한 상태이며, 별도 잠금 센서가 없으므로 실제 잠금해제 성공을 센서로 확인한 상태는 아닙니다. 실기 자동 후퇴는 이 버전에 포함되지 않습니다.

영상은 250 ms 이내, 로봇 피드백은 600 ms 이내여야 합니다. 두 마커 소실, 원본 자세 급변, 기준판 변경, TCP/user 변경, 작업 범위 초과, 명령 거절, 힘/토크 초과는 독립 `StopMotion` 채널로 정지를 요청합니다. 정지 뒤에는 재연결해야 하며 자동 재개하지 않습니다. 네트워크가 끊기면 소프트웨어 정지 응답을 확인할 수 없으므로 UI도 정지 성공을 표시하지 않습니다. 현장 제어기의 보호 기능과 물리 비상정지가 필요한 이유입니다.

판단 로그: `.delivery/d455_key_panel_real/decisions.jsonl`, `decisions.txt`. 실제 TCP/목표 TCP, 슬롯 좌표, 영상 시각/revision, 단계, 깊이·회전·축 오차, 연결/오류/정지 결과를 남깁니다.

## 개발 검증과 공식 문서

`py -3 -m unittest tests.test_key_panel_real tests.test_fairino -v`

`py -3 tools/d455_key_panel_real.py --smoke`는 통신/동작 없이 GUI를 확인합니다. `--probe`는 192.168.57.2로 읽기 전용 연결 시험을 합니다.

공식 근거: [Python SDK](https://github.com/FAIR-INNOVATION/fairino-python-sdk), [MoveL과 좌표계](https://fairino-doc-en.readthedocs.io/3.9.8/SDKManual/PythonRobotMovement.html), [힘/토크 조회와 보호](https://fairino-doc-en.readthedocs.io/3.9.8/SDKManual/PythonRobotForceControl.html).
