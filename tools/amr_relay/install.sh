#!/bin/bash
set -euo pipefail
if [ "$(id -u)" -ne 0 ]; then
    echo 'Run on the MoMa Ubuntu PC: sudo bash install.sh'
    exit 1
fi
command -v python3 >/dev/null
command -v systemctl >/dev/null
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if ! ip -4 addr show | grep -qE 'inet 192\.168\.30\.13/'; then
    echo 'This installer must run on the MoMa PC with address 192.168.30.13.'
    exit 1
fi
echo 'Checking the internal AMR status port (TCP connect only)...'
python3 - <<'PY'
import socket
with socket.create_connection(('192.168.58.102', 19204), 3):
    print('AMR status TCP port reachable.')
PY
install -d -m 0755 /opt/moma-amr-relay
install -m 0644 "$SCRIPT_DIR/relay.py" /opt/moma-amr-relay/relay.py
cat > /etc/systemd/system/moma-amr-relay.service <<'UNIT'
[Unit]
Description=MoMa AMR TCP relay for desktop 192.168.30.12
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 -u /opt/moma-amr-relay/relay.py
Restart=on-failure
RestartSec=3
DynamicUser=yes
NoNewPrivileges=yes
PrivateTmp=yes
ProtectSystem=strict
ProtectHome=yes

[Install]
WantedBy=multi-user.target
UNIT
systemctl daemon-reload
systemctl enable moma-amr-relay.service
systemctl restart moma-amr-relay.service
systemctl --no-pager --full status moma-amr-relay.service
echo 'Relay installed. Set desktop GUI AMR host to 192.168.30.13.'
echo 'If UFW is active, allow TCP 19204,19205,19206,19207,19210 from 192.168.30.12.'
