# 2026-11-07 업데이트 검증

- 현재 IDE 작업 폴더의 unittest 208개: Python 3.12 UTF-8 모드에서 통과.
- 웹 TypeScript 검사 및 Vite production build: 통과.
- 설치된 Isaac Sim 6.2 USD/PhysX 스키마 검증: 통과. 실제 시뮬레이션 stepping 검증은 아님.
- Windows OpenGL 배열 디스플레이 목록 생성: 오류 없음 확인.
- 전체 저장소에 보존된 기존 테스트의 최초 일괄 실행: 697개 중 실패 2개/오류 30개. Windows 기본 인코딩, 테스트 패키지 충돌, 소켓 모의 객체와 이전 도착 판정 fixture 문제를 일부 수정함.
- 수정 후 전체 저장소 재실행은 일부 비전 테스트 오류가 있었고 test_key_panel 실행 중 프로세스가 중단되어 전체 통과를 확인하지 못함. 208개 통과는 전체 저장소 테스트 통과를 뜻하지 않음.
- 실제 로봇 주행, main2/main3 실행, IO 출력, 물리 접촉 개방은 이번 GitHub 업데이트 검증에서 수행하지 않음.
- 로봇 PC 변경 재현용 패치는 tools/runtime_viewer_patches에 포함. 최신 원격 파일 재조회는 접속 시간 초과로 못 했으며 기존 검토 스냅샷 기준.
