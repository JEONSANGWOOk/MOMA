import io
import json
import unittest
from unittest.mock import Mock
from seer_control.runtime_scenario import RuntimeScenario

def response(data):return io.BytesIO(json.dumps(data).encode())

class RuntimeScenarioTests(unittest.TestCase):
    def test_connected_one_post_only(self):
        client=RuntimeScenario();client.opener=Mock()
        client.opener.open.side_effect=[response({'connected':True}),response({'status':'success'})]
        self.assertEqual(client.start('http://localhost:8000/runtime.html').result(2)['status'],'success')
        self.assertEqual(client.opener.open.call_count,2)
        request=client.opener.open.call_args_list[1].args[0]
        self.assertEqual(request.get_method(),'POST')
        self.assertEqual(request.full_url,'http://localhost:8000/api/execute_scenario')
        self.assertEqual(json.loads(request.data),{'func':'main2'})

    def test_disconnected_never_posts(self):
        client=RuntimeScenario();client.opener=Mock()
        client.opener.open.return_value=response({'connected':False})
        with self.assertRaises(ValueError):client.start('http://localhost:8000').result(2)
        self.assertEqual(client.opener.open.call_count,1)
        self.assertFalse(client.uncertain)

    def test_timeout_locks_out_retry(self):
        client=RuntimeScenario();client.opener=Mock()
        client.opener.open.side_effect=[response({'connected':True}),TimeoutError('lost response')]
        with self.assertRaises(TimeoutError):client.start('http://localhost:8000').result(2)
        with self.assertRaises(ValueError):client.start('http://localhost:8000')
        self.assertEqual(client.opener.open.call_count,2)

    def test_duplicate_running_blocked(self):
        from concurrent.futures import Future
        client=RuntimeScenario();client.future=Future()
        with self.assertRaises(ValueError):client.start('http://localhost:8000')

    def test_main3_request_and_invalid_name_block(self):
        client=RuntimeScenario();client.opener=Mock()
        client.opener.open.side_effect=[response({'connected':True}),response({'status':'success'})]
        client.start('http://localhost:8000','main3').result(2)
        request=client.opener.open.call_args_list[1].args[0]
        self.assertEqual(json.loads(request.data),{'func':'main3'})
        with self.assertRaises(ValueError):client.start('http://localhost:8000','main9')
