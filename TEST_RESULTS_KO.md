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

