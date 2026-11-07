# 외부 Windows PC 설치 안내

## 압축 파일에 포함된 항목

- `run_windows.bat`, Python 앱 소스, 기본 지도와 설정 템플릿
- `models/`의 MoMa AMR·FR5·그리퍼 모델
- `SIMPLE_WHOLE.glb` 설비 3D 모델
- 공식 FR5 모델의 고정 버전과 해시를 확인하는 설치 도구

실행 캐시, 테스트 로그와 영상, CAD 원본 `SIMPLE_WHOLE.stp` 및 USDZ는 제외했습니다. 앱 실행에는 GLB 설비 모델을 사용합니다.

## Windows에서 실행

1. ZIP을 원하는 경로에 풀어 `MOMA-main` 폴더를 유지합니다.
2. Python 3.10 이상과 Tcl/Tk가 설치된 Windows용 Python을 준비합니다.
3. `MOMA-main\run_windows.bat`를 실행합니다. 필요한 Pillow 패키지는 누락된 경우 설치를 시도합니다.

화면이 열리지 않으면 먼저 아래 명령이 성공하는지 확인합니다.

```powershell
py -3 -c "import tkinter; print(tkinter.Tcl().eval('info patchlevel'))"
```

## FR5 준비

- 로봇팔 개발 화면에서 **공식 FR5 모델**을 설치한 뒤 앱을 재시작합니다. 이 설치에는 외부 PC의 인터넷 연결이 필요합니다. 인터넷이 없으면 공식 저장소에서 받은 `frcobot_ros2` 폴더를 사용해 `tools\install_fairino_model.py --repo <저장소 경로>`를 실행할 수 있습니다.
- FR5 연결 설정에서 실제 컨트롤러 펌웨어와 호환되는 FAIRINO SDK 경로를 지정하고, 상태 수신과 정지 동작을 확인합니다.
- 실제 관절 범위 검증에는 공식 모델이 필요합니다. 기본 경량 URDF는 실기 동작 보증용이 아닙니다.

## 실제 동작에 관한 제한

현재 두 문 열기 경로는 SIM 전용입니다. REAL 모드에서는 하드웨어 전송을 차단합니다. 이 ZIP을 외부 PC에 옮기는 것만으로 FR5 실기 동작이 활성화되지는 않습니다. 실제 동작을 연결하기 전에는 현장 FR5 모델·펌웨어·TCP/툴 프레임·팔 장착 위치를 확인하고, 별도의 실기 경로 검증이 필요합니다.

컨트롤러 IP, SDK 설정, 현장 보정값은 ZIP에 넣지 않았습니다. 외부 PC에서 해당 장비 설정을 직접 입력하세요.
