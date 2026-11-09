# FR5 공식 SDK 직접 IO 수신 설치 결과

2026-11-09 사용자 요청으로 이 PC에 공식 FAIRINO Python SDK를 설치했습니다.

- 제어기 실제 응답: FR5-V1-002(V6.0), **v3.9.7**, V3.9.33-QX.
- 설치 SDK: **v2.2.7_robot_v3.9.7**.
- 위치: `D:/FAIRINO_SDK/v2.2.7_robot_v3.9.7/windows/fairino/Robot.py`.
- 공식 원본: https://github.com/FAIR-INNOVATION/fairino-python-sdk/releases/tag/v2.2.7_robot_v3.9.7
- Python 3.12 사용자 site-packages에 pth를 등록해 `from fairino import Robot`이 동작합니다. 원본 파일/라이선스/파일별 SHA256은 SDK 폴더에 보관합니다.

## 현재 연결

이 PC는 192.168.57.2로 직접 라우팅되지 않습니다. 기존 제어 PC 192.168.30.13을 경유하는 SSH 포트 전달로 공식 SDK의 20003 RPC와 20005 CNDE 상태 수신을 연결했습니다. 비밀번호는 설정에 저장하지 않았고, 전용 SSH 키는 두 목적지 포트의 전달만 허용하도록 등록했습니다. 그 키의 원격 셸 실행은 차단했습니다. 기존 Viewer를 종료하거나 재시작하지 않았습니다.

별도 프로세스 `seer_control.fr5_readonly_service`가 SDK를 통해 GetSoftwareVersion, GetActualJointPosDegree, GetActualTCPNum, GetDO, GetToolDO만 읽습니다. CNDE 상태 구독을 사용하며 이동/서보/모드 변경/DO 출력/StopMotion을 호출하지 않습니다. 기존 Runtime 제어 드라이버를 FAIRINO 제어 드라이버로 바꾸지 않았습니다. 실제 시나리오 제어는 기존 프로그램이 유지합니다.

실제 SDK에서 관절 6축, DO0~15, ToolDO가 수신되는 것을 확인했습니다. 확인 시 DO7은 OFF였습니다. Runtime 관절값과 SDK 관절값도 최대 약 0.01도 이내로 일치했고 연속 표본 30/30개가 최신 상태였습니다. 실제 DO7 ON/OFF를 테스트 목적으로 출력하지는 않았습니다.

## 사용

**이 PC Python 프로그램만 완전히 종료 후 재시작**하세요. 제어 PC Viewer는 계속 실행해도 됩니다.

실기 3D에서 문 표시의 DO7 데이터는 새 공식 SDK 수신값을 우선 사용합니다. 베큠 DO2/4 ON 및 DO3/5 OFF 표시도 같은 직접 수신을 우선합니다. 기존 사용자 지정 Runtime IO 경로는 그대로 존중합니다. FR5 제어 화면의 **FR5 SDK IO 수신 확인** 버튼에서 버전, DO2/3/4/5/7 및 6축 값을 확인할 수 있습니다.

제어기 Tool 번호가 1/2이면 후크/롤러로 사용합니다. 현재 제어기 Tool 번호는 0이고 MoMa의 Tool은 SDK 내 가상 좌표일 수 있어, 이 경우 기존 Runtime의 Tool 정보로 후크/롤러 종류를 구분합니다. DO7 자체는 Viewer를 거치지 않습니다. 마커·기존 Runtime 시나리오·카메라 경로는 바꾸지 않았습니다.

다음 실행 시 REAL 화면에서 수신 프로세스가 없으면 자동 시작합니다. SIM에서는 실제 IO를 화면 동작에 사용하지 않습니다. 통신 단절/오래된 값/다른 로봇 호스트/다른 수신 프로세스의 잔여 파일은 사용하지 않으며 기존 경로로 전환하거나 상태를 불명으로 유지합니다. 0.8초를 넘은 값으로 ON/OFF를 추측하지 않습니다.

## 로컬 파일

`%USERPROFILE%/.seer_amr_console/` 아래:

- fr5_readonly.json: SDK 경로·제어기/중계 IP·버전·키 경로 설정.
- fr5_direct_feedback.json: 최신 읽기 전용 상태.
- fr5_readonly.pid / .lock: 전용 수신 프로세스 관리.
- fr5_readonly_service.log: SDK 접속 및 오류 로그.
- keys/fr5-readonly-ed25519: 현재 Windows 사용자만 읽고 쓸 수 있도록 ACL을 제한한 터널 키.

SDK 직접 수신기만 별도로 실행하므로 Viewer 재시작이나 로봇 제어기 재부팅이 필요 없습니다. 이전 문서에서 DO7 수신을 위해 Viewer 재시작이 필요하다고 한 부분은 Runtime 중계 경로에만 해당합니다.

## 다른 PC에서 준비

공식 SDK는 제어기 펌웨어에 맞는 릴리스를 별도로 설치하세요. 이 저장소는 SDK 바이너리·개인 SSH 키·사용자 설정을 포함하지 않습니다. 읽기 전용 SSH 수신기는 `python -m pip install -r requirements-fr5-readonly.txt`로 추가 의존성을 설치하고, 해당 PC의 SDK 경로와 제한된 포트 전달 키를 설정해야 합니다. 기존 PC의 개인 키나 설정을 GitHub에 올리지 마세요.
