# 공식 ROS2 FR5 모델 — Windows 사용

[FAIR-INNOVATION/frcobot_ros2](https://github.com/FAIR-INNOVATION/frcobot_ros2)에서 FR5 `fairino_description/urdf/fairino5_v6.urdf`와 STL 메시 7개를 사용합니다. 고정된 원본 커밋은 `fcf0c7f0d60d949d8a9a4238f929a44d07f60379`입니다. 원본 URDF와 STL을 수정하지 않고 사용자 로컬 모델 캐시에 설치합니다. 프로그램의 manifest로 각 파일의 SHA256을 확인합니다.

## 프로그램에서 사용
- WORKSPACE → 로봇팔 개발 → **공식 FR5 모델**을 누릅니다. 이미 설치되었으면 즉시 적용하고, 없으면 공식 GitHub 원본에서 받습니다. 로봇팔 재생/미션을 종료하고 변경하세요.
- 현재 노트북에는 위 공식 저장소에서 모델을 받아 설치했습니다. 프로그램 재시작 시 기본 모델로 사용됩니다. 이전 경량 기본 모델 경로는 공식 설치 모델로 전환하며, 다른 사용자가 가져온 모델은 유지합니다.
- 적용 후 전용 워크스페이스와 AMR+로봇팔 3D 화면에 같은 모델을 사용하고 설정을 저장합니다.
- 기본 관절 이동 예제 재생 및 관절 티칭/MoveL 직선 미리보기를 사용할 수 있습니다. 로봇 연결 없이 모델 표시와 SIM 개발이 가능합니다.

## 명령으로 설치
프로그램 폴더에서 실행합니다.

```powershell
py -3 tools/install_fairino_model.py
```

이미 Git으로 받은 저장소를 사용할 때:

```powershell
py -3 tools/install_fairino_model.py --repo C:\path\frcobot_ros2
```

캐시는 `%USERPROFILE%\.seer_amr_console\models\frcobot_ros2\<commit>\fairino_description`에 있습니다. `urdf`와 `meshes` 구조를 유지하여 ROS `package://fairino_description/...` 경로를 그대로 해석합니다. 설치된 `urdf\fairino5_v6.urdf`를 기존 URDF 가져오기에서도 선택할 수 있습니다. 다운로드 후 모델 표시에는 네트워크가 필요하지 않습니다. 배포 ZIP에는 모델 다운로더와 manifest가 포함되며 ROS 패키지 전체나 메시 원본을 복사해 넣지 않습니다.

## 범위
이 연결은 ROS2 패키지의 3D 자산/관절 구조를 Windows 뷰어에 사용합니다. ROS 노드, MoveIt, 실기 드라이버를 실행하는 기능이 아닙니다. FR5 실기 동작은 기존 공식 SDK/API 연결에서 별도로 실행합니다.

기본으로 선택한 모델은 fairino5_v6이며 같은 저장소의 FR5WML 등 다른 모델로 임의 대체하지 않습니다. 실제 장비 형식 및 tool/TCP 오프셋 일치는 현장에서 확인해야 합니다. SIM TCP는 모델 끝단 wrist3_link 기준이며 설정한 실기 TCP와 자동으로 같아지지 않습니다. 화면 메시만 근처 정점을 병합하여 단순화합니다. 관절 축·원점·관절 한계는 원본 URDF를 사용합니다. 시각 메시 단순화는 충돌/정밀 CAD 검증을 대신하지 않습니다.
