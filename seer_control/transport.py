"""SEER 16-byte TCP framing, based on official Python/Java examples.
Hardware and firmware compatibility must be validated on the target robot.
"""
import json
import socket
import struct
import time

HEADER = struct.Struct('>BBHIH6s')
MAX_PAYLOAD = 8 * 1024 * 1024
MAX_FILE_PAYLOAD = 128 * 1024 * 1024


def receive_exact(sock, size):
    result = bytearray()
    while len(result) < size:
        part = sock.recv(size-len(result))
        if not part:
            raise ConnectionError('TCP 연결이 응답 중 종료되었습니다.')
        result.extend(part)
    return bytes(result)


class ReadOnlyClient:
    def __init__(self, host, profile):
        if profile.get('verified') is not True:
            raise ValueError('API 프로파일 미검증: 문서 확인 후 verified 및 조회 API 설정이 필요합니다.')
        if profile.get('transport') not in ('candidate-seer-16-byte', 'seer-16-byte'):
            raise ValueError('지원하지 않는 프레임 형식입니다.')
        self.host = host
        self.profile = profile
        self.seq = 0
        self.timeout = 2.0
        if not profile.get('status_queries'):
            raise ValueError('status_queries에 검증된 조회 API를 등록하세요.')
        for q in profile['status_queries']:
            if q.get('read_only') is not True:
                raise ValueError('조회 전용 API만 허용합니다.')
            for key in ('port', 'request_type', 'response_type'):
                value = q.get(key)
                if type(value) is not int or not (0 <= value <= 65535):
                    raise ValueError(f'{key}: 16비트 정수가 필요합니다.')
            if q['port'] == 0:
                raise ValueError('port는 1 이상이어야 합니다.')
        self.fields = profile.get('fields', {})

    def request(self, query):
        self.seq = (self.seq+1) % 65536
        seq = self.seq
        payload = query.get('payload')
        data = json.dumps(payload, separators=(',', ':'), ensure_ascii=True, allow_nan=False).encode('ascii') if payload else b''
        packet = HEADER.pack(0x5A, 1, seq, len(data), query['request_type'], b'\0'*6) + data
        with socket.create_connection((self.host, query['port']), self.timeout) as sock:
            sock.settimeout(self.timeout)
            sock.sendall(packet)
            magic, version, rx_seq, length, kind, reserved = HEADER.unpack(receive_exact(sock, HEADER.size))
            if magic != 0x5A or version != 1 or rx_seq != seq:
                raise ValueError('응답 헤더 또는 요청 식별자가 일치하지 않습니다.')
            if not query.get('accept_any_response', False) and kind != query['response_type']:
                raise ValueError(f"응답 API 번호 불일치: expected={query['response_type']} actual={kind}")
            limit = query.get('max_payload', MAX_PAYLOAD)
            if type(limit) is not int or not 1 <= limit <= MAX_FILE_PAYLOAD:
                raise ValueError('응답 크기 제한 설정 오류')
            if length > limit:
                raise ValueError(f'응답 크기 제한 초과: {length} > {limit}')
            response = json.loads(receive_exact(sock, length).decode('utf-8')) if length else {}
            if not isinstance(response, dict):
                raise ValueError('JSON object 응답이 필요합니다.')
            if query.get('accept_any_response', False):
                response = dict(response)
                response['_response_type'] = kind
            key = query.get('return_code_field')
            if response.get('ret_code', 0) != 0 or (key and response.get(key, 0) != query.get('success_value', 0)):
                raise ValueError(f'컨트롤러 오류: {response}')
            return response

    def snapshot(self):
        raw = {}
        for query in self.profile['status_queries']:
            raw[query['name']] = self.request(query)
        mapped = {}
        for target, spec in self.fields.items():
            value = raw
            try:
                for key in spec['path'].split('.'):
                    value = value[key]
                if 'scale' in spec:
                    value = float(value) * float(spec['scale'])
                mapped[target] = value
            except (KeyError, TypeError, ValueError) as error:
                if not spec.get('optional', False):
                    raise ValueError(f'상태 필드 매핑 오류: {target}') from error
        return mapped, raw


def port_probe(host, ports, timeout=1.0):
    results = []
    for port in ports:
        start = time.monotonic()
        try:
            with socket.create_connection((host, int(port)), timeout):
                results.append((port, 'TCP OPEN', round((time.monotonic()-start)*1000)))
        except OSError as error:
            results.append((port, str(error), None))
    return results

class PersistentSeerSession:
    """Persistent single-port SEER TCP session for high-rate control (e.g. API 2010 jog)."""
    def __init__(self, host, port, timeout=3.0):
        self.host = host
        self.port = int(port)
        self.timeout = float(timeout)
        self.seq = 0
        self.sock = None

    def connect(self):
        self.close()
        self.sock = socket.create_connection((self.host, self.port), self.timeout)
        self.sock.settimeout(self.timeout)
        return self

    def close(self):
        sock, self.sock = self.sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def request(self, api, payload=None, max_payload=MAX_PAYLOAD):
        if self.sock is None:
            self.connect()
        self.seq = (self.seq + 1) % 65536
        seq = self.seq
        data = json.dumps(payload, separators=(',', ':'), ensure_ascii=True, allow_nan=False).encode('ascii') if payload else b''
        packet = HEADER.pack(0x5A, 1, seq, len(data), int(api), b'\0'*6) + data
        try:
            self.sock.sendall(packet)
            magic, version, rx_seq, length, kind, reserved = HEADER.unpack(receive_exact(self.sock, HEADER.size))
            if magic != 0x5A or version != 1 or rx_seq != seq:
                raise ValueError('지속 TCP 응답 헤더/sequence 불일치')
            if length > max_payload:
                raise ValueError(f'응답 크기 제한 초과: {length} > {max_payload}')
            response = json.loads(receive_exact(self.sock, length).decode('utf-8')) if length else {}
            if not isinstance(response, dict):
                raise ValueError('JSON object 응답이 필요합니다.')
            response = dict(response)
            response['_response_type'] = kind
            if response.get('ret_code', 0) != 0:
                raise ValueError(f'컨트롤러 오류: {response}')
            return response
        except Exception:
            self.close()
            raise
