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

- X: 카메라 기준 오른쪽, Y: 아래쪽, Z: 앞쪽. 화면 XYZ는 mm입니다.
- XYZ는 마커 중심의 위치이며, 로봇 베이스 좌표나 공구 목표 좌표가 아닙니다.
- `err`는 모서리 재투영 오차(px)이며 실제 위치 정확도(mm)를 보장하지 않습니다.
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
