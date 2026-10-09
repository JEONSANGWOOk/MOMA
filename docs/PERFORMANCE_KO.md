# 3D 렌더링 성능 개선 — 2026-10-09

증상: 3D 화면에서 AMR 또는 로봇팔이 움직일 때 화면 갱신이 느리고 입력이 버벅임.

## 확인한 원인과 수정

- OpenGL이 화면 가로·세로의 2배 크기로 렌더링하고, 매 프레임 약 4배의 픽셀을 읽은 뒤 Pillow Lanczos로 축소했다. 화면의 실제 픽셀 크기로 렌더링하도록 변경했다.
- 메인 tick, 팔 workspace tick, 미션의 팔 상태 갱신이 각각 world 화면을 렌더링할 수 있었다. 정기 팔 갱신·미션은 상태만 동기화하고 메인 tick에서 world 화면을 그리도록 변경했다. 수동 관절 조절의 즉시 갱신은 유지한다.
- 카메라 basis를 정점마다 다시 계산했다. 위치·거리·yaw·pitch가 달라질 때만 계산하도록 캐시했다. 원근/직교 투영의 좌표 왕복 검증을 유지했다.
- 지도 등의 생성 polygon 하나마다 smooth vertex adjacency를 다시 구성했다. 단일 면 법선을 사용하고 모델 자산의 smooth shading은 유지한다.
- 매 프레임 Tk 이미지 객체를 새로 생성했다. 같은 크기에서는 기존 PhotoImage에 새 픽셀을 넣는다.
- 고정 크기 4×4 좌표 변환과 점 변환에서 내부 generator/sum 호출을 줄였다.
- 마우스·크기 변경 이벤트가 모이면 after_idle로 화면 갱신을 합친다.
- SIM 센서 안내와 3D 표시에서 LiDAR scan을 중복 계산하지 않고 주기적으로 수집한 scan_points를 사용한다.
- 메인 tick은 활성 AMR 3D 화면에서 50ms 주기를 목표로 처리 시간을 뺀 다음 예약한다. 팔 tick도 처리 시간을 제외한 50ms 주기를 목표로 한다. 부하가 높으면 최소 10ms의 이벤트 처리 여유를 둔다.

## 측정

로봇 접속 없이 임시 사용자 설정, 기본 지도·모델, 1100×700 world viewport, NVIDIA GeForce RTX 2070 Super with Max-Q Design에서 측정했다. 기존 실행 중인 앱을 종료하거나 사용자 설정을 수정하지 않았다.

| 지표 | 변경 전 | 변경 후 |
|---|---:|---:|
| 반복 렌더링 중앙값 | 87.61ms | 24.03ms |
| 반복 렌더링 최대값 | 89.00ms | 31.26ms |
| 첫 프레임 | 920.31ms | 962.94ms |

반복 렌더링 중앙값은 약 73% 감소했다. 변경 전은 6프레임, 변경 후는 12프레임 표본이며 CPU/GPU 부하에 따라 달라질 수 있다. 첫 모델의 법선 계산·OpenGL display list 생성 비용은 남아 있어 첫 3D 표시에는 약 1초의 준비 시간이 있었다.

변경 후 Tk 메인 루프에서 AMR과 팔을 동시에 움직이는 2초 SIM 검증에서 world 갱신 호출은 약 15.5회/초였다. 이것은 해당 시험의 callback 빈도이며 모든 화면·지도에서 같은 FPS를 보장하지 않는다. 픽셀당 반복 렌더링 시간과 실제 프로그램 전체 FPS를 구분해야 한다.

원본 결과:

- `artifacts/render_performance_before.json`
- `artifacts/render_performance_after.json`

2배 supersampling을 제거했으므로 기존 화면과 가장자리의 안티에일리어싱 정도가 달라질 수 있다. 지도·로봇 모델의 전체 형상과 smooth shading은 유지한다.

## 검증

- `python -X utf8 -m unittest discover -s tests`: 393 tests OK.
- 추가 회귀 테스트: 카메라 캐시 무효화·투영 왕복, 생성 면 법선, 실제 픽셀 크기의 readback·작은 viewport 출력, 렌더링 없이 팔 상태를 동기화하는 경로.
- `tools/render_performance_smoke.py`: 실제 OpenGL 이미지 크기, AMR 이동, 팔 관절 변화, world/팔 동기화, Tk callback 오류 없음 확인.
- `tools/fr5_smoke.py`: SIM 등록 MoveJ 미션·안전 자세·합성 REAL 관절 표시·설정 화면 PASS. 초기 운영 페이지에서 3D 검증 페이지를 명시적으로 선택하도록 smoke 도구도 수정했다.
- 실제 로봇 연결·명령 전송은 수행하지 않았다.

### GitHub 최신 버전 통합

GitHub main의 v1.38(`fe5a0b08c511`)을 기준으로 성능 개선을 합쳤다. 지도 가져오기·진단, 영역 검증, 실기 장애물 복구와 명령 확인 기능을 유지했다. 실기 미션 초기화는 simulator가 없는 구성에서도 실행되도록 보호하고, simulator가 있는 경우 기존 건너뛴 목표 표시를 초기화한다.

- 통합본: `python -X utf8 -m unittest discover -s tests` — 409 tests OK.
- 통합본 FR5 GUI smoke 및 OpenGL/AMR·팔 동시 움직임 smoke 통과.
- 통합본 6프레임 반복 렌더링 중앙값 21.35ms. 환경 부하에 따른 측정 변동이 있으므로 기존 전후 결과 파일은 유지했다.

실행 가능한 Python은 `C:/Users/never/AppData/Local/Programs/Python/Python313/python.exe`에서 확인했다. `py -3` launcher의 설치 검색 실패와 별개로 해당 인터프리터를 직접 사용하여 검증했다.

## 적용

현재 실행 중인 창을 종료하고 프로젝트의 `run_windows.bat`으로 다시 실행한다. 별도의 기존 EXE를 실행하면 소스 변경이 반영되지 않으며 재빌드가 필요하다.
