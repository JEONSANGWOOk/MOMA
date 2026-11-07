# MoMa Ubuntu AMR TCP 중계

내 PC(192.168.30.12) → ipTIME → MoMa Ubuntu(192.168.30.13) → AMR(192.168.58.102).

이 폴더의 파일을 **MoMa Ubuntu PC**로 복사하고 터미널에서 실행합니다.

```bash
cd amr_relay
sudo bash install.sh
```

Python 3 표준 라이브러리만 사용합니다. MoMa에서 AMR 상태 포트에 접근 가능한지
먼저 확인하고, systemd 서비스로 설치합니다. 192.168.30.0/24 대역의 PC 접속을 허용합니다.
중계 포트는 GUI 코드의 19204/19205/19206/19207/19210입니다.
패킷 내용은 변경하지 않으며 연결 실패 시 명령을 자동 재전송하지 않습니다.
IP forwarding, AMR 게이트웨이 변경, ipTIME 정적 경로 설정은 필요하지 않습니다.

Ubuntu 방화벽 UFW가 활성화되어 있다면 아래 규칙을 추가합니다.

```bash
sudo ufw status
sudo ufw allow from 192.168.30.0/24 to 192.168.30.13 port 19204,19205,19206,19207,19210 proto tcp
```

UFW를 새로 활성화하거나 방화벽을 비활성화할 필요는 없습니다.

내 PC에서 확인합니다(PowerShell).

```powershell
Test-NetConnection 192.168.30.13 -Port 19204
```

`TcpTestSucceeded : True` 확인 후 GUI에서 AMR IP를 **192.168.30.13**으로 지정해
조회 연결부터 확인합니다. 로봇팔 실시간 표시 주소는 기존
http://192.168.30.13:8000/runtime.html 을 그대로 사용합니다.
조회 연결 확인 후 내 PC의 AMR 테더링 Wi-Fi를 해제합니다.
MoMa PC 자체의 ipTIME 무선 연결은 유지해야 합니다.
이 구성은 내 PC↔AMR 테더링 구간을 제거하며 MoMa↔ipTIME 무선 품질의 영향을 받습니다.

진단 및 중지(MoMa Ubuntu):

```bash
sudo journalctl -u moma-amr-relay -n 50 --no-pager
sudo systemctl disable --now moma-amr-relay
```

이 패키지는 내 PC의 로컬 전달 테스트만 완료한 상태입니다.
MoMa에 설치한 뒤 실제 중계 포트 접근을 확인해야 합니다.

## PC 주소가 바뀌어도 연결하기

중계 기본 허용 범위는 `192.168.30.0/24`입니다. 내 PC가 `.9`, `.12`, `.14` 등으로 바뀌어도 허용됩니다.
변경된 `relay.py`와 `install.sh`를 로봇 Ubuntu PC에 복사하고 `sudo bash install.sh`로 재설치해야 실제 서비스에 적용됩니다.
이 설치 스크립트의 로봇 PC 주소는 기존 `.13` 기준입니다. 로봇 PC의 실제 주소가 다르면 먼저 확인해야 합니다.
이 허용 설정은 TCP 중계에만 적용됩니다. ping/ARP가 안 되는 문제는 로봇 PC의 IP, 유무선 연결, 공유기 격리 설정을 별도로 확인하세요.
