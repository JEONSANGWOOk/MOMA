# MOMA

MoMa development code — SEER AMR Control Studio v1.2 Workspace.

[한국어 사용 설명](README_KO.md) · [최종 변경 사항](RELEASE_NOTES_KO.md) · [검증 기록](TEST_RESULTS.txt)

Windows: Python 3.10 이상(Tcl/Tk 포함)을 설치한 뒤 `run_windows.bat`을 실행하세요. 기본 앱은 Python 표준 라이브러리만 사용합니다. Linux에서는 Tcl/Tk 설치 후 `run_linux.sh`를 사용합니다.

2D/3D 지도와 경로 편집, SEER AMR SIM/실기 제어, 미션, FAIRINO FR5 공식 SDK 제어와 시각화, DualSense CFI-ZCT1G 수동 조종과 SIM 벽 충돌·장애물 우회를 제공합니다. [FR5 연결 / ROS 안내](docs/FR5_API_KO.md)를 참고하세요.

새로 빌드한 EXE는 포함하지 않은 소스 배포입니다. FR5 실기 제어에는 제어기 버전에 맞는 공식 SDK가 별도로 필요합니다. 실제 AMR·로봇팔 하드웨어 호환성은 별도 검증이 필요합니다. 제조사 정밀 CAD와 ROS 연결은 포함하지 않습니다.

단위 테스트:

```powershell
py -3 -X utf8 -m unittest discover -s tests
```
