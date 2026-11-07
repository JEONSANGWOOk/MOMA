# SEER AMR DI/DO
확장 도구 → DI / DO에서 확인합니다. 실기 연결 후 1초 간격으로 조회합니다.
기존 IO 매핑이 있다면 SEER 기본 API 적용 버튼으로 저장하세요. 다른 장비용 설정은 적용 전에 보관하세요.

- 조회: TCP 19204, 1013 → 11013. DI/DO 및 DI_valid 해석.
- 일반 출력: TCP 19210, 6001 → 16001. id와 boolean status 사용.
- 행 선택 후 DO ON/OFF. 제어 모드, 최신 DO 수신, 정지 및 미션 종료 상태가 필요합니다.
- DI는 읽기 전용. 미수신·만료는 —, 비활성 입력은 비활성으로 표시합니다.
- ACK 이후 실제 DO를 재조회합니다. 실패한 출력은 자동 재전송하지 않습니다.
- 안전 LiDAR나 안전 입력을 변경하는 기능은 없습니다.

공식 IO 문서: https://seer-robotics.gitbooks.io/robotkit-netprotocol-l/content/zh/chapter2/robot_status_io.html
공식 프로토콜 PDF p89: https://github.com/seer-robotics/Robokit_TCP_API_py/blob/master/robotkit-netprotocol-l-1.2.1.pdf

모의 검증 완료. 실제 펌웨어·배선과 출력 동작은 현장 확인이 필요합니다.
