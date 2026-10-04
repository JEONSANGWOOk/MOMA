# v1.21 검증 (2026-10-04)

- unittest discover: 286 tests OK.
- 모델 설치 신규 6개: 로컬 설치, 원본 해시 불일치 차단, 캐시 재사용/변조 검사, 고정 커밋 다운로드, 잘못된 다운로드 차단, 정점 병합 후 닫힌 메시 유지.
- official_fr5_smoke: 실제 공식 URDF, 원본 STL 7개/6관절/패키지 경로, 독립 MoveJ 및 10mm MoveL, 공유 3D 뷰어 적용 PASS.
- 공식 저장소 커밋 fcf0c7f0d60d949d8a9a4238f929a44d07f60379. 제어기/로봇 네트워크 접속 없음.

# v1.20 검증 (2026-10-04)

- unittest discover: 280 tests OK.
- 신규 10개: 독립 MoveJ 완료/궤적, 자세 포함 수치 IK, 도달 불가 IK, MoveL 직선 오차/도착, 실패 목표에서 이동 차단, I/O 출력·센서 분리, 미등록 참조, 시간 초과, 일시정지/정지/관절 한계, 미지원 툴 좌표 차단.
- arm_workspace_smoke: WORKSPACE 등록, 독립 예제 재생, 1366×768/1100×650 화면, 설정 저장 PASS. SDK 요청/AMR 상태 변경 없음.
- 실제 FR5 연결/명령 전송 없음. 기본 URDF의 실제 제품 기구학/충돌 물리 일치 여부 미검증.

# v1.19 검증 (2026-10-04)

- unittest discover: 270 tests OK.
- 신규 17개: 프로그램 확장/분기 번호, 초안 미등록 참조/형식 검사, I/O 채널·논리 검증, 실제 입력 대기, 오류/정지/일시정지, 라이브러리 연결정보 제외, 입력 시간 초과 시 후속 이동 차단, AMR 상태 소실 정지, FR5 단독 실행 및 연결된 AMR 우회 차단.
- fr5_program_smoke: 작업 개발 창과 픽앤플레이스 초안 표시 PASS.
- fr5_smoke: 기존 FR5 설정, SIM MoveJ 미션, 안전 자세, 합성 실기 3D 상태 표시 PASS.
- API 호출은 인메모리 가짜 SDK로 검증. 실제 장비 네트워크 접속/동작 없음.

# v1.18 검증 (2026-10-04)

- unittest discover: 253 tests OK.
- 공식 3050 포트/Station/좌표/각도 요청, 비정상 값 네트워크 전송 차단 확인.
- hardware_report_smoke: GUI PASS. 실제 로봇 연결/명령 전송 없음.
- 최신 Feishu 문서 본문과 대상 제어기 펌웨어 호환성은 미검증.

