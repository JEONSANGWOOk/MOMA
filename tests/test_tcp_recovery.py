import ast
import socket
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from seer_control.transport import HEADER, ReadOnlyClient, PersistentSeerSession, receive_exact


class TCPRecoveryTests(unittest.TestCase):
    def test_failed_status_socket_is_discarded_without_replay(self):
        query = dict(name='pose', read_only=True, port=19204,
                     request_type=1100, response_type=11100)
        client = ReadOnlyClient('localhost', dict(verified=True, transport='seer-16-byte',
                                status_queries=[query]))
        client.persistent_status = True
        broken, healthy = Mock(), Mock()
        broken.recv.side_effect = TimeoutError('late')
        healthy.recv.side_effect = [HEADER.pack(0x5A, 1, 2, 2, 11100, bytes(6)), b'{}']
        with patch('seer_control.transport.socket.create_connection', side_effect=[broken, healthy]) as connect:
            with self.assertRaises(TimeoutError):
                client.request(query)
            self.assertEqual(client._status_sockets, {})
            broken.close.assert_called_once()
            broken.sendall.assert_called_once()
            self.assertEqual(connect.call_count, 1)
            self.assertEqual(client.request(query), {})
            self.assertEqual(connect.call_count, 2)
        client.close()
        healthy.close.assert_called_once()

    def test_read_only_polling_reuses_one_connection(self):
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen()
        listener.settimeout(3)
        errors = []
        seen = []

        def serve():
            try:
                with listener.accept()[0] as conn:
                    conn.settimeout(2)
                    for _ in range(2):
                        header = HEADER.unpack(receive_exact(conn, HEADER.size))
                        receive_exact(conn, header[3])
                        seen.append(header[2])
                        data = b'{"x":1}'
                        conn.sendall(HEADER.pack(0x5A, 1, header[2], len(data), 11100, bytes(6)) + data)
                    self.assertEqual(conn.recv(1), b'')
            except Exception as error:
                errors.append(error)
            finally:
                listener.close()

        query = dict(name='pose', read_only=True, port=listener.getsockname()[1],
                     request_type=1100, response_type=11100)
        client = ReadOnlyClient('127.0.0.1', dict(verified=True, transport='seer-16-byte',
                                status_queries=[query], fields={'x': {'path': 'pose.x'}}))
        client.persistent_status = True
        worker = threading.Thread(target=serve, daemon=True)
        worker.start()
        try:
            self.assertEqual(client.snapshot()[0], {'x': 1})
            self.assertEqual(client.snapshot()[0], {'x': 1})
            self.assertEqual(len(client._status_sockets), 1)
        finally:
            client.close()
            worker.join(3)
        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(seen, [1, 2])

    def test_fragmented_reply_timeout_and_reconnect_without_replay(self):
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen()
        listener.settimeout(3)
        seen = []
        errors = []

        def serve():
            try:
                for index in range(2):
                    with listener.accept()[0] as conn:
                        header = HEADER.unpack(receive_exact(conn, HEADER.size))
                        receive_exact(conn, header[3])
                        seen.append(header[4])
                        if index == 0:
                            time.sleep(.15)
                        else:
                            data = b'{"ret_code":0}'
                            packet = HEADER.pack(0x5A, 1, header[2], len(data), header[4]+10000, bytes(6)) + data
                            for offset in range(0, len(packet), 3):
                                conn.sendall(packet[offset:offset+3])
                                time.sleep(.001)
            except Exception as error:
                errors.append(error)
            finally:
                listener.close()

        worker = threading.Thread(target=serve, daemon=True)
        worker.start()
        session = PersistentSeerSession('127.0.0.1', listener.getsockname()[1], timeout=.05)
        try:
            with self.assertRaises(TimeoutError):
                session.request(2010)
            self.assertIsNone(session.sock)
            self.assertEqual(seen, [2010])
            session.timeout = 1
            self.assertEqual(session.request(1100)['ret_code'], 0)
            self.assertEqual(session.sock.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY), 1)
            self.assertEqual(session.sock.getsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE), 1)
        finally:
            session.close()
            worker.join(3)
        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(seen, [2010, 1100])

    def test_status_worker_recovers_after_timeout(self):
        source = (Path(__file__).resolve().parents[1]/'seer_control/app.py').read_text(encoding='utf-8')
        node = next(n for n in ast.walk(ast.parse(source)) if isinstance(n, ast.FunctionDef) and n.name == 'state_work')
        events = []
        stopped = False
        waits = []

        def put(event):
            nonlocal stopped
            events.append(event)
            if event[0] == 'state':
                stopped = True

        client = SimpleNamespace(snapshot=Mock(side_effect=[TimeoutError('late'), ({'x': 1}, {})]), close=Mock())
        env = dict(stop=SimpleNamespace(is_set=lambda: stopped, wait=waits.append),
                   state_client=client, self=SimpleNamespace(events=SimpleNamespace(put=put)),
                   generation=7, time=time)
        exec(compile(ast.Module(body=[node], type_ignores=[]), '<state-worker>', 'exec'), env)
        env['state_work']()
        self.assertEqual([event[0] for event in events], ['state_retry', 'state'])
        self.assertEqual(client.snapshot.call_count, 2)
        client.close.assert_called_once()
        self.assertEqual(waits[0], .2)


if __name__ == '__main__':
    unittest.main()
