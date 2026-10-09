# 두 노트북으로 로봇 통신·주행 시험

현재 노트북 A는 기존 GUI를 실행합니다. 다른 노트북 B는 가상 로봇 API 서버와 지도 화면을 실행합니다.
B에서 이동을 계산하고 A는 실제 TCP 요청·응답으로 상태를 받습니다. 카메라는 필요하지 않습니다.
가상 서버는 실제 로봇이나 로봇팔에 접속하지 않습니다.

## 노트북 B에 설치

Windows용 `LAN_Robot_Emulator_Windows.zip`을 **압축 해제**한 뒤 `LAN_Robot_Emulator.exe`를 실행하세요.
Windows용 묶음은 Python을 포함하므로 별도 설치가 필요 없습니다.
가상 로봇 창이 열리고 창을 닫으면 서버도 종료됩니다.

소스를 직접 실행하거나 VS Code에서 수정하려면 `LAN_Robot_Emulator_Source.zip`을 압축 해제하세요.
[Python](https://www.python.org/downloads/windows/) 3.10 이상을 설치하고 Windows 설치 시 Tcl/Tk와 Python 실행 경로를 포함하세요.
이 가상 로봇은 Python 표준 라이브러리로 동작하며 별도 pip 설치가 필요 없습니다.
소스 묶음에서는 압축 해제한 폴더의 `LAN_Robot_Emulator.cmd`를 더블클릭합니다.
가상 로봇 창과 콘솔이 열립니다. 창을 닫으면 API 서버도 종료됩니다.
다른 로봇 서버가 같은 포트를 사용하면 시작이 실패하므로 그 서버를 먼저 종료하세요.

Linux에서는 Python과 배포판의 tkinter 패키지를 준비한 뒤 실행합니다.

```sh
python3 -m seer_control.robot_emulator
```

화면이 없는 환경에서는 `--headless`를 추가합니다. 특정 인터페이스만 사용하려면 `--host IP주소`를 지정하세요.
기본값 `0.0.0.0`은 이 노트북의 네트워크 인터페이스에서 요청을 받는 바인딩 주소이며,
노트북 A의 로봇 IP 칸에는 B의 실제 IPv4 주소를 입력해야 합니다.

## LAN과 Windows 방화벽

두 노트북을 같은 공유기/스위치에 LAN으로 연결하세요. 직접 연결하는 경우 같은 대역의 사용하지 않는 주소를 지정할 수 있습니다.
예를 들어 A는 `192.168.50.10`, B는 `192.168.50.20`, 서브넷 마스크는 둘 다 `255.255.255.0`입니다.
이미 IP가 할당되는 네트워크에서는 임의로 주소를 바꾸지 말고 `ipconfig`에 표시된 Ethernet IPv4를 사용하세요.

B에서 네트워크를 개인(Private)으로 지정합니다. 가상 로봇의 수신 포트는 TCP 19204,19205,19206,19207입니다.
방화벽 허용이 필요하면 **B의 관리자 PowerShell**에서 다음을 실행하세요.

```powershell
New-NetFirewallRule -DisplayName 'LAN Virtual AMR' -Direction Inbound -Action Allow -Protocol TCP -LocalPort 19204,19205,19206,19207 -Profile Private -RemoteAddress LocalSubnet
```

A에서 아래 주소를 B의 IPv4로 바꾸고 확인합니다. ICMP ping을 막는 환경에서도 TCP는 열릴 수 있습니다.

```powershell
Test-NetConnection 192.168.50.20 -Port 19204
Test-NetConnection 192.168.50.20 -Port 19206
```

`TcpTestSucceeded : True`이면 해당 포트로 연결할 수 있습니다.

## 노트북 A의 GUI에서 연결

1. 로봇 IP에 **B의 Ethernet IPv4 주소**를 입력합니다.
2. 상단 연결 모드를 **실기 · 제어**로 선택하고 연결합니다.
3. **제어권 가져오기**로 가상 로봇의 제어권을 획득합니다.
4. **Pull Map / 지도 다운로드**로 B의 `emulator_demo` 지도를 받습니다.
5. 사용자 모드의 **작업 운영** 화면에서 지도 노드를 클릭해 방문 목록을 만들고, 미리보기 후 시작합니다.
6. A에서 위치·속도·도착 상태와 판단 로그를 확인하고 B에서 이동 모습을 확인합니다.

기본 시작 위치는 LM1입니다. LM2 또는 LM3부터 시험하면 이동이 잘 보입니다.
2D 지도는 B의 가상 로봇 창에서 표시되고 A에서는 기존 실시간 지도와 3D 표시를 사용할 수 있습니다.
GUI의 `REAL` 표기는 실제 TCP 통신 경로를 뜻합니다. 수신한 제어기 모델은 `LAN Virtual AMR`로 표시됩니다.
실제 하드웨어 검증을 의미하지 않습니다.

## 시험 시나리오

- B의 지도 클릭: 선택한 도구에 따라 고정 장애물이나 왕복하는 사람을 배치합니다.
- 장애물 삭제 도구: 지도에서 장애물 근처를 클릭합니다. 전체 삭제 버튼도 있습니다.
- BLOCKED 켜기/끄기: 강제 차단 상태를 만들어 정지와 해제 처리를 시험합니다.
- 비상정지 켜기/끄기: 이동을 취소하고 비상정지 상태를 전달합니다. 해제 후 새 미션을 시작하세요.
- 주행 실패: FAILED 결과를 전달합니다.
- 통신 끊김 5초: TCP 응답을 끊습니다. 서버는 계속 실행되며 A의 GUI는 오류를 확인한 뒤 다시 연결해야 합니다.
- 응답 지연: 0~10초 사이의 지연을 지정합니다. 명령은 적용 후 ACK가 늦어질 수 있습니다.
- 위치 신뢰도: 낮은 localization 값의 표시·판단을 확인합니다.

3051 등록 경로 주행은 장애물 앞에서 대기합니다. 장애물을 제거하면 기존 경로로 계속 이동합니다.
3050 등록 노드 자유 주행은 가상 제어기의 우회 기능을 사용합니다.
GUI의 실기 장애물 복구를 시험하려면 개발자 설정의 해당 옵션과 대기/재탐색 정책을 설정해야 합니다.
GUI의 SIM 전용 알고리즘이 REAL 모드에서 모두 실행되는 것은 아닙니다.
강제 BLOCKED는 토글로 해제하는 시험 조건입니다. 장애물 좌표로 우회를 시험하려면 지도에 물체를 배치하세요.

## 로그와 지원 범위

B의 `emulator_logs/decisions.jsonl`과 `decisions.txt`에 서버 판단과 시뮬레이터 판단을 저장합니다.
API 수락/거부, 상황·근거·결론·동작, 장애물 판단과 주행 결과를 기록합니다.
A의 이벤트 로그와 미션 보고서에는 GUI의 REAL 통신 경로 판단이 기록됩니다.

지원: 상태 조회 1000/1002/1007/1009/1020/1021/1022/1100/1101, 지도·Station 1300/1301,
등록 노드 주행 3050/3051, 일시정지/재개/취소 3001/3002/3003, 수동 차동 주행 2010/2000,
자동·수동 모드 4000, 제어권 4005/4006, 재위치 2002/2003/2004, 지도 4010/4011/2022.
지속 TCP 연결, 분할 수신, 요청 sequence와 응답 프레임을 지원합니다.

3050의 임의 XY 목적지, 실제 SLAM, 실제 충전 도킹, FR5 SDK/로봇팔 API는 이 서버에서 구현하지 않습니다.
지도 업로드는 노드·경로 시험용이며 시험 장면의 물리 벽은 유지합니다.
미지원 API나 잘못된 요청은 오류 응답을 반환합니다.

## VS Code에서 B의 코드 수정

가상 로봇 실행 자체에는 VS Code 원격 연결이 필요 없습니다.
원격으로 수정하려면 B에 OpenSSH Server를 설치·실행하고, A의 VS Code에 Remote - SSH 확장을 설치하세요.

B의 관리자 PowerShell:

```powershell
Add-WindowsCapability -Online -Name OpenSSH.Server~~~~0.0.1.0
Start-Service sshd
Set-Service -Name sshd -StartupType Automatic
```

SSH 수신 포트 TCP 22도 연결 가능한지 확인하세요. B의 계정과 인증 수단이 필요합니다.
A의 VS Code에서 `Remote-SSH: Connect to Host...`를 실행하고 `B계정@B_IP`로 접속한 다음
압축 해제한 폴더를 열면 됩니다. 그래픽 창은 B의 바탕화면에서 실행하세요.
원격 터미널에서는 `python -m seer_control.robot_emulator --headless`로 서버를 실행할 수 있습니다.

참고: [VS Code Remote SSH](https://code.visualstudio.com/docs/remote/ssh),
[Windows OpenSSH 설치](https://learn.microsoft.com/en-us/windows-server/administration/openssh/openssh_install_firstuse).

## 현재 검증

로컬 TCP 통신으로 현재 GUI의 실기 연결, 제어권, 지도 다운로드, 사용자 미션, 위치·속도,
일시정지·재개, BLOCKED 해제 후 완료 및 로그를 확인했습니다.
다른 노트북에 설치한 뒤 실제 LAN IP·방화벽을 통한 접속은 별도 확인이 필요합니다.

검증 결과: 전체 테스트 455개 통과. Windows EXE의 내장 지도, 실제 TCP 이동 피드백,
일시정지·재개·취소, Tk 창 표시와 창 종료 후 포트·로그 정리를 확인했습니다.
