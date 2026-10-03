# 확인한 공식 자료 및 구현 근거

조회일: 2026-09-19. 공식 저장소의 코드/문서를 직접 읽고 Python 표준 라이브러리로 구현했습니다.
타사 실행파일이나 예제 스크립트를 실행하지 않았으며 공식 SDK 소스 자체를 배포물에 복사하지 않았습니다.

- SEER 공식 Python TCP 예제 및 API PDF:
  https://github.com/seer-robotics/Robokit_TCP_API_py
  commit a407ea56767ec50cfea4ed301a6c2c9894de3845
  https://github.com/seer-robotics/Robokit_TCP_API_py/blob/master/robotkit-netprotocol-l-1.2.1.pdf
- 공식 Java SDK:
  https://github.com/seer-robotics/SeerSdk4j
  commit c19a0abfa2d9120d3e90e7cd80e4163279340943
- 공식 TCP 테스트 도구:
  https://github.com/seer-robotics/SeerTCPTest
  commit 47ea4fe388f35841902f4a29ace1bd1c1528f70b
- 공식 지도 스키마와 샘플:
  https://github.com/seer-robotics/smap
  commit 122b98f396a2bfd5e62add2cf86bccfbb5bdb6a5
- 공식 다운로드 안내: https://seer-robotics.ai/download
- RoboShop 화면/흐름은 사용자가 제공한 `4. AMR_Roboshop 유저 메뉴얼.pdf`를 참고했습니다.
  온라인 검색에서 SourceForge 구형 매뉴얼 1.1.0도 발견했지만 현행 매뉴얼로 간주하지 않았습니다.

## 적용 API (PDF 인쇄 페이지)

| 요청 / 응답 | 포트 | 역할 | 근거 |
|---|---|---|---|
| 1100 / 11100 | 19204 | 배치 상태·위치·센서·Task | p36 이후 |
| 1301 / 11301 | 19204 | 로봇 지도 노드 | p46 |
| 2002 / 12002 | 19205 | 재위치 | p54 |
| 2003 / 12003 | 19205 | 위치 확인 | p55 |
| 3001 / 13001 | 19206 | 일시정지 | p63 |
| 3002 / 13002 | 19206 | 재개 | p64 |
| 3003 / 13003 | 19206 | 취소 | p65 |
| 3051 / 13051 | 19206 | 고정 경로 목적지 이동 | p67 |
| 4000 / 14000 | 19207 | 수동 0 / 자동 1 | p73 |

16바이트 헤더: 0x5A, 버전 1, uint16 시퀀스, uint32 body 길이, uint16 API 번호,
예약 6바이트. 다중바이트 정수는 big-endian. 인자 없는 요청은 길이 0.
공식 Python/Java 예제에 따라 예약 바이트는 0으로 설정합니다.
C++ 최신 도구는 예약부 확장도 지원하지만 이 프로그램은 파일 전송 확장 프로토콜을 사용하지 않습니다.
ret_code는 문서상 생략 가능하며 있으면 0만 성공으로 처리합니다.

## 미완성 범위의 이유

SLAM 2020/2021은 PDF p57/58에서 deprecated로 표시됩니다.
최신 SLAM 저장/상태 흐름, 맵 전송, 조그 통신 단절 시 정지 보장, 충전 시퀀스는 이 자료만으로
대상 펌웨어의 동작을 확정할 수 없어 실기 GUI 동작으로 노출하지 않았습니다.
공식 TCP 예제 확보는 펌웨어 호환성 시험을 대체하지 않습니다.

v0.2.1: 1300/11300(19204, PDF p45) 지도 목록·현재 지도 조회,
2022/12022(19205, PDF p59) map_name을 사용한 로봇 지도 전환 추가.
1100의 loadmap_status 및 1300의 current_map으로 전환 완료를 확인합니다.


## v0.3.0 map download
- 1300 / 11300, port 19204: loaded/stored map information.
- 4011 / 14011, port 19207: download map, request body `{"map_name":"..."}`. Response body is stored as JSON SMAP and rendered read-only.
- Reference cross-check: seersdk-rs ConfigApi::DownloadMap=4011 and public Python example that receives the declared body length and saves it as map JSON.

## v0.6.0 Map Edit / Push
- 4010 Upload Map: request JSON body is the complete JSON-format map (SMAP).
- 4011 Download Map: returns the complete JSON-format map on success.
- AdvancedPoint: className, instanceName, pos, dir, property.
- AdvancedCurve: className=BezierPath (or firmware variants), instanceName, startPos/endPos, controlPos1/controlPos2, property.
- 2022 Load Map: switch/reload map after upload when explicitly selected.

References checked during development:
- SEER RoboKit NetProtocol map appendix / SMAP schema.
- SEER official GitHub TCP examples and SDK port routing.
