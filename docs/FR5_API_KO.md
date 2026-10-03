# FR5 공식 API 연결

이 프로그램의 FR5 실기 제어는 제조사 공식 Python SDK (`fairino.Robot.RPC`)를 사용합니다. Windows에서 ROS 없이 연결할 수 있습니다. SDK는 프로그램에 포함하지 않으며 제어기 WebAPP 버전에 맞는 제조사 배포본을 지정합니다. 실제 FR5에서의 검증은 아직 수행하지 않았습니다.

## 연결 순서

1. FR5 WebAPP에서 제어기 버전을 확인합니다. 해당 버전에 맞는 [공식 SDK 릴리스](https://github.com/FAIR-INNOVATION/fairino-python-sdk/releases)를 받습니다. 최신 SDK를 임의로 선택하지 마세요.
2. 제조사의 [Windows SDK 설치 안내](https://support.fairino.us/portal/en/kb/articles/how-to-install-and-start-working-with-pythonsdk)에 따라 SDK와 그 버전의 의존성을 준비합니다. 빌드한 `.pyd`를 사용하면 프로그램을 실행하는 Python 버전과 일치해야 합니다. 실행 환경은 `run_windows.bat`이 선택한 Python입니다.
3. 프로그램을 REAL 모드로 바꿉니다. **확장 도구 → 로봇팔 / 장비 연결 → FR5 연결 설정**에서 IP(기본 예시 `192.168.58.2`), SDK의 `windows` 폴더 또는 그 아래 `fairino` 폴더, WebAPP 버전을 입력합니다. SDK가 실행 Python에 이미 설치되어 있으면 폴더는 비워도 됩니다.
4. 설정을 저장하고 **FR5 연결 시험 / 관절 수신**을 누릅니다. 연결은 상태 조회만 수행합니다. 로봇 활성화, 자동 모드 전환, 원점 이동은 자동으로 실행하지 않습니다. 실기 활성화/모드는 제조사 WebAPP에서 준비합니다.
5. 실제 팔을 안전하게 접은 상태로 정지시킨 후 설정 화면에서 **수신한 현재 자세로 작업 등록**을 사용해 `safe_pose`라는 `MoveJ` 작업을 등록합니다. 사용자는 이 자세가 AMR 이동에 적합한지 확인해야 합니다. 프로그램의 3D 미리보기 자세는 실기 안전 자세로 사용하지 않습니다.
6. 같은 방식으로 `work` 등 작업을 등록합니다. tool/user 번호와 속도는 실제 설비에 맞게 편집합니다. 현재 자세 등록의 기본값은 tool/user 0, 속도 10%입니다.
7. SDK 호환성을 확인하고 **등록한 작업의 실기 동작 허용**을 켠 후 저장하고 다시 연결합니다. **FR5 작업 실행** 또는 미션의 **Arm Action**으로 등록한 작업 이름을 실행합니다.

FR5 연결과 AMR 연결은 별개입니다. 현재 미션 엔진에서 팔 작업을 실행하려면 AMR도 연결되어 있고, 제어권과 최신 상태가 있으며, 정지 상태여야 합니다. FR5 상태 조회는 AMR 연결 전에도 가능합니다.

## 등록 작업 형식

`operations`는 작업 이름별 객체입니다. 프로그램에서 실제 수신한 자세를 등록하세요. 동작 가능한 임의 예제 좌표는 제공하지 않습니다.

| 필드 | 의미 |
|---|---|
| method | `MoveJ` 또는 `MoveL` |
| target | MoveJ: J1~J6 각도 6개 [도]. MoveL: X,Y,Z [mm]와 RX,RY,RZ [도] 6개 |
| tool / user | 제조사에 등록된 공구 / 작업 좌표계 번호 0~14 |
| vel | 제조사 SDK 속도 비율 %, 0 초과~100. 기본 10 |

각 작업은 하나의 이동입니다. 여러 이동, 대기, I/O 및 AMR 이동은 여러 Arm Action 등으로 미션에 순서대로 구성합니다. 이 버전에는 ServoJ 스트리밍, 그리퍼 전용 명령, 임의 SDK 메서드 실행, IK/충돌 계획 기능이 없습니다.

## 상태와 3D 표시

- SDK의 관절값(도)을 라디안으로 변환하여 3D 모델에 반영합니다. 기본 FR5 모델의 6축 순서와 대응합니다. 외부 URDF는 movable joint 순서가 J1~J6와 일치해야 합니다.
- J1~J6, TCP, 완료 상태, 비상 정지, 안전 정지, 오류 코드를 표시합니다. 미션 중 상태 조회는 약 2 Hz, 대기/일시정지는 약 5 Hz입니다. 통신 지연이 있으면 주기가 늘어납니다.
- 완료 비트만으로 다음 미션을 실행하지 않습니다. 실제 목표와 일치하고 완료 상태여야 합니다. 관절은 0.5도, TCP는 위치 1mm/자세 0.5도 허용 오차를 사용합니다. 적용한 좌표계의 실제 반환 좌표를 확인하세요. 불일치하면 완료로 간주하지 않고 미션 제한 시간 후 정지를 요청합니다.
- `safe_pose`의 측정 관절과 일치하고 정지/오류 없음이 확인되어야 AMR 이동 안전 상태로 표시됩니다. 수신이 2초 이상 지연되면 안전 상태를 해제하고 3D 실시간 연동을 중단합니다.

## 정지와 통신

이동은 SDK의 비동기 옵션(MoveJ `blendT=0`, MoveL `blendR=0`)을 사용합니다. 미션 일시정지/재개는 `PauseMotion`/`ResumeMotion`에 연결합니다. 미션 취소 및 FR5 정지는 SDK 작업 대기열과 별도 통신으로 제조사 `StopMotion`을 요청합니다(공식 SDK의 XML-RPC 20003 호출과 동일). 정지 후에는 연결 시험으로 다시 연결해야 합니다.

일부 SDK 버전은 통신 오류를 내부에서 무한 재시도하므로 SDK는 별도 프로세스로 실행합니다. 연결 조회는 15초, 이후 요청은 6초 제한입니다. 제한을 넘으면 프로세스를 종료하고 자동으로 명령을 재전송하지 않습니다. 소프트웨어 정지 응답에 실패하면 화면과 로그에 표시합니다. 제어기의 물리적 비상정지 기능을 대체하지 않습니다. 암호화 TLS/DTLS 연결은 이 어댑터에 포함하지 않았습니다.

현재 FR5 연결은 Python 소스로 실행할 때 지원합니다. 기존 EXE에 SDK를 추가한 빌드는 제공하지 않습니다. SIM 모드에서는 실기에 전송하지 않습니다. 등록한 MoveJ 목표는 시뮬레이션 관절 보간으로 확인할 수 있습니다. SIM MoveL은 역기구학이 없어 실행을 차단합니다.

## ROS / ROS2도 가능한가?

제조사 [ROS1 패키지](https://github.com/FAIR-INNOVATION/frcobot_ros)와 [ROS2 패키지](https://github.com/FAIR-INNOVATION/frcobot_ros2)가 있습니다. ROS2에서는 제조사 서비스 API 또는 MoveIt2/ros2_control 경로를 사용할 수 있습니다. 제어기 버전에 맞는 하드웨어 패키지를 선택해야 합니다. 이 Windows 프로그램에 ROS 명령 입력/ROS2 브리지가 추가된 것은 아닙니다.

제조사 [FR5 MoveIt2 안내](https://support.fairino.us/portal/en/kb/articles/getting-started-with-the-fairino-moveit2-plugin)는 Ubuntu 22.04와 ROS2 Humble을 기준으로 합니다. 해당 패키지를 설치/빌드하고 환경을 source한 후의 **RViz 계획 데모** 명령은 다음과 같습니다. 이 명령만으로 실기가 연결되는 것은 아닙니다.

```bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

실기 연결은 제조사 안내에 따라 컨트롤러 IP, 하드웨어 플러그인, 제어기 버전을 맞추어야 합니다. 서비스 명령을 사용하려면 설치한 버전의 `fairino_msgs` 타입과 실제 서비스 목록을 먼저 확인하세요. 이 프로그램과 ROS 제어기는 동시에 팔 동작을 지시하지 않도록 운영하세요.

## 참고 공식 자료

- [Python SDK](https://github.com/FAIR-INNOVATION/fairino-python-sdk)
- [SDK 이동 API](https://fairino-doc-en.readthedocs.io/latest/SDKManual/PythonRobotMovement.html)
- [SDK 상태 API](https://fairino-doc-en.readthedocs.io/latest/SDKManual/PythonRobotStatusInquiry.html)
- [SDK 소스의 XML-RPC / StopMotion 구현](https://github.com/FAIR-INNOVATION/fairino-python-sdk/blob/main/windows/fairino/Robot.py)
