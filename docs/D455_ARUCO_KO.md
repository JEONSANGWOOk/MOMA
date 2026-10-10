# D455로 ArUco 마커 위치 확인

이 도구는 RGB 영상의 마커 ID와 카메라 기준 위치·방향을 표시합니다.
로봇팔 통신이나 이동 명령은 포함하지 않습니다. 로봇팔 접근에는 별도 좌표 보정과 TCP 설정이 필요합니다.

## 준비와 실행

RealSense Viewer를 닫고 D455를 USB 3 포트에 연결하세요.
USB 2 연결에서는 RGB만 사용하며 깊이 비교값은 기록하지 않습니다.
프로젝트 폴더에서 별도 환경을 만들고 선택 라이브러리를 설치합니다.

```powershell
python -m venv .venv-vision
.\.venv-vision\Scripts\python.exe -m pip install -r requirements-vision.txt
.\.venv-vision\Scripts\python.exe tools/d455_aruco_preview.py --generate-marker artifacts/aruco
```

`artifacts/aruco/print_marker.html`을 브라우저로 열고 배율 100% / 실제 크기로 출력하세요.
마커는 DICT_4X4_50, ID 0이며 검은 외곽 사각형 한 변이 100mm입니다.
출력 후 실제 검은 사각형의 한 변을 자로 측정하세요. 흰 여백은 크기에 포함하지 않습니다.
평평한 판에 붙여 카메라에서 네 모서리가 모두 보이게 놓으세요.

`D455_ArUco_Preview.cmd`를 더블클릭하면 100mm 기준으로 인식을 시작합니다.
실제 출력 크기가 다르다면 아래 명령의 `100`을 측정한 mm 값으로 바꾸세요.

```powershell
.\.venv-vision\Scripts\python.exe tools/d455_aruco_preview.py --marker-mm 100 --log .delivery/d455_aruco.jsonl
```

ESC 또는 Q로 종료합니다. USB 카메라는 하나의 프로그램에서 사용하세요.
여러 D455가 연결되었다면 `--list-devices`로 확인하고 `--serial`로 선택할 수 있습니다.

## 표시와 기록

각 마커의 오른쪽 패널에는 `Tilt`와 `RX / RY / RZ` 각도(도)도 표시합니다.
`Tilt`는 마커 평면의 법선과 카메라 광축 사이 각도이며 정면 0°, 옆면 90°입니다.
카메라 위치에서 마커 중심으로 향하는 시선과의 각도가 아니라 광축 기준입니다.
각도는 PnP 자세에서 계산하는 추정값이며 평면 자세의 모호성 때문에 흔들릴 수 있습니다.

RX/RY/RZ는 정면에서 마커 위쪽이 영상 위쪽을 향할 때 모두 0°가 되도록 보정한
Euler 각도입니다. 카메라 좌표 X 오른쪽/Y 아래/Z 앞쪽, 회전 행렬
`R = Rz(RZ) Ry(RY) Rx(RX)` 기준입니다. RX는 상하 기울기(양수이면 법선이 영상 위쪽),
RY는 좌우 기울기(양수이면 법선이 영상 오른쪽), RZ는 평면 회전(양수이면 영상 시계 방향)을
표시합니다. 복합 회전에서는 이 순서에 따라 각도를 해석하세요. RY가 ±90° 부근이면
RX/RZ를 독립적으로 결정할 수 없어 화면에 모호성을 표시합니다. Tilt는 계속 유효합니다.
로그의 `orientation_deg`에 `tilt_deg`, `rx_deg`, `ry_deg`, `rz_deg`,
`euler_singular`를 저장합니다. 자세 추정 실패 시 `orientation_deg=null`입니다.
기존 `rotation_vector_rad`는 그대로 유지하며 로봇 Euler 각도로 직접 사용하지 않습니다.

- X: 카메라 기준 오른쪽, Y: 아래쪽, Z: 앞쪽. 화면 XYZ는 mm입니다.
- XYZ는 마커 중심의 위치이며, 로봇 베이스 좌표나 공구 목표 좌표가 아닙니다.
- 로그의 `reprojection_px`는 모서리 재투영 오차(px)이며 실제 위치 정확도(mm)를 보장하지 않습니다.
  역 Brown 모델에서는 SDK로 보정한 가상 픽셀 좌표의 오차입니다.
- 자세는 마커 좌표계이며, 회전 벡터를 로봇 Euler 각도로 그대로 사용하면 안 됩니다.
  역 Brown RGB 모델에서는 영상에 자세 축을 그리지 않고 좌표와 로그의 회전 벡터만 표시합니다.
- 깊이는 컬러에 정렬한 마커 중심 주변의 유효 깊이 중앙값입니다. 깊이값 0은 유효하지 않습니다.
- JSONL은 최대 초당 5회 기록합니다. XYZ와 깊이는 m, 회전 벡터는 rad입니다.
- 마커 미검출 시 빈 markers와 detected=false를 기록합니다. 이전 위치를 현재 위치로 재사용하지 않습니다.

마커 크기를 잘못 입력하면 거리도 비례해서 틀어집니다.
단일 평면 마커의 자세에는 모호성이 있을 수 있으며, 여기서는 양의 깊이를 가진 PnP 후보 중
재투영 오차가 작은 후보를 표시합니다. 시간 안정성이나 로봇 작업 가능 여부를 판정하는 제어기는 아닙니다.
RGB 내부 파라미터는 현재 스트림에서 읽습니다. D455의 역 Brown 왜곡은 SDK의
`rs2_deproject_pixel_to_point`로 네 모서리를 광선으로 변환한 뒤 왜곡 없는 가상 픽셀에서 PnP를 풉니다.
순방향 Brown 모델은 OpenCV에 적용하며, 그 외 지원하지 않는 비영(非零) 왜곡 모델은 오류로 중단합니다.

## 로봇팔 접근으로 확장

작업대 고정 카메라는 카메라→로봇 베이스 변환을 보정합니다.
손목 카메라는 카메라→손목 변환을 보정하고 관측 시점의 로봇 자세와 결합합니다.
이후 마커→작업 위치 오프셋, TCP, 접근 방향·거리, 로봇 API 단위와 회전 표현을 설정합니다.
보정 결과는 별도 위치에서 검증한 뒤 사용해야 합니다.

실제 로봇이 없을 때는 마커 검출, 카메라 좌표의 변화·안정성, 로그 및 시뮬레이터 목표 표시를 검증할 수 있습니다.
실제 로봇팔과의 좌표 보정은 현재 카메라만으로 완료할 수 없습니다.

## 확인 명령

```powershell
.\.venv-vision\Scripts\python.exe tools/d455_aruco_preview.py --self-test
.\.venv-vision\Scripts\python.exe tools/d455_aruco_preview.py --list-devices
.\.venv-vision\Scripts\python.exe tools/d455_aruco_preview.py --probe
```

`--self-test`는 합성 마커 인식, 알려진 위치 복원, 마커 크기와 거리 비례 및 비정상 자세 거부를 확인합니다.
`--probe`는 카메라 프레임을 5초 수신하며 화면이나 이미지 파일을 저장하지 않습니다.

참고: [RealSense Python SDK](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/readme.md),
[OpenCV ArUco](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html),
[RealSense 깊이 정렬](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/align-depth2color.py).

현재 노트북에서 장치 이름, 펌웨어 및 USB 연결 조회를 확인했습니다. 합성 마커와 위치 계산 자체 검증을 통과했습니다.
## 여러 마커 동시 인식

인식 화면은 보이는 마커를 ID 개수 제한 없이 함께 검색합니다. 지원 계열은
4×4·5×5·6×6·7×7(각 1000개 ID), ArUco Original, ArUco MIP 36h12입니다.
50/100/250개 사전은 같은 계열 1000개 사전에 포함되므로 화면과 로그에는
대표 이름 `DICT_4X4_1000` 등이 표시됩니다. 출력용 기본 마커는 계속 4×4 ID 0입니다.
같은 ID의 마커가 여러 위치에 있어도 각각 기록합니다. 서로 다른 계열에서 ID가
같을 수 있으므로 로그의 `dictionary`와 `id`, `corners_px`를 함께 확인하세요.

로그에는 `marker_count`와 각 마커의 종류·ID·영상 모서리 좌표가 저장됩니다.
위치 추정 실패 시에도 인식 결과를 남기며 `pose_valid=false`, 좌표는 `null`입니다.
미검출 프레임은 `marker_count=0`, `markers=[]`로 남겨 이전 위치를 재사용하지 않습니다.
가려지거나 너무 작고 흐린 마커는 인식되지 않을 수 있습니다. AprilTag 및 사용자 정의
사전은 이 ArUco 자동 검색의 대상이 아닙니다.

현재 모든 마커의 위치 추정에는 실행 시 지정한 동일한 `--marker-mm`를 적용합니다.
크기가 다른 마커도 ID 인식은 가능하지만 XYZ 거리는 실제 크기를 설정해야 맞습니다.
로봇팔 제어 좌표로 사용하기 전 마커별 크기 및 카메라-로봇 좌표 보정이 필요합니다.

## 카메라 이동으로 가상 로봇팔 테스트

`D455_ArUco_Preview.cmd`로 영상/로그를 실행한 상태에서 `D455_Arm_Simulator.cmd`를
실행하세요. 기존 FR5 URDF와 팔 시뮬레이터를 사용하는 별도 창이 열립니다.
실제 로봇 연결 기능 없이 JSONL 카메라 로그만 읽습니다.

마커를 고정하고 카메라를 손으로 천천히 움직이세요. 처음 보이는 기준 마커를 자동
선택하고, 새로운 유효 프레임 3개를 받으면 현재 카메라/팔 위치를 기준으로 추적을
시작합니다. 마커 선택을 바꾸면 정지합니다. `추적 시작`은 현재 위치를 다시 기준으로
설정한 뒤 추적합니다. `기준 위치 설정`은 기준만 저장하고 정지 상태를 유지합니다.

마커 자세를 역변환하여 카메라 원점 이동을 계산합니다. 초기 카메라 축 기준으로
Z → 가상 팔 X, -X → Y, -Y → Z를 적용하고 이동량의 50%를 반영합니다.
TCP 회전 자세는 유지합니다. 이는 테스트용 축 대응이며 실제 카메라-로봇 보정이
아닙니다. 마커도 움직이면 상대 이동이 반영되므로 카메라 이동만 테스트할 때는
기준 마커를 고정해야 합니다. 실제 마커 크기가 맞아야 이동량도 맞습니다.

기준 대비 가상 목표 거리 150mm, TCP 최대 40mm/s, 관절 최대 30도/s로 제한하며
필터와 1.5mm 데드밴드로 흔들림을 줄입니다. 기존 역기구학과 시뮬레이터 충돌 검사도
적용합니다. 이 시뮬레이터 충돌 검사는 실제 장비 안전 검증을 대신하지 않습니다.
기준 마커 미검출/동일 ID 중복, 0.7초 수신 지연, 재투영 오차 2px 초과,
이동 범위 초과 또는 IK/충돌 검사 실패 시 현재 자세를 유지하고 추적을 정지합니다.
다시 유효한 마커가 보이면 `추적 시작`으로 재개하세요.

판단 로그는 `.delivery/d455_arm_sim/decisions.jsonl`과 `decisions.txt`에
상황·근거·결론·동작 형태로 저장됩니다. 카메라와 가상 팔 창은 각각 닫을 수 있습니다.

