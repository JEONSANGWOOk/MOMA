import unittest
from unittest.mock import patch
from seer_control.studio_devices import parse_io, seer_io_operations, tcp_operation

class SeerIOTests(unittest.TestCase):
    def test_documented_banks(self):
        banks,valid=parse_io({'DI':[True,False],'DO':[False],'DI_valid':[False,True]},seer_io_operations()['read_io'])
        self.assertEqual(banks,{'di':{0:True,1:False},'do':{0:False}})
        self.assertFalse(valid[0])
    def test_sparse_ids(self):
        banks,_=parse_io({'DI':[{'id':7,'status':True}],'DO':{'4':0}},seer_io_operations()['read_io'])
        self.assertEqual(banks,{'di':{7:True},'do':{4:False}})
    def test_missing_is_unknown(self):
        self.assertEqual(parse_io({'DI':[]},seer_io_operations()['read_io'])[0]['do'],{})
    def test_reject_bad_states(self):
        for value in ['false',{},2,None]:
            with self.subTest(value=value),self.assertRaises(ValueError):
                parse_io({'DI':[value]},seer_io_operations()['read_io'])
    def test_output_packet_no_retry(self):
        with patch('seer_control.studio_devices.PersistentSeerSession') as session:
            client=session.return_value;client.request.return_value={'_response_type':16001}
            tcp_operation('localhost',{'operations':seer_io_operations()},'set_do',{'channel':3,'value':False})
            client.request.assert_called_once_with(6001,{'id':3,'status':False})
            client.request.reset_mock();client.request.side_effect=TimeoutError('late')
            with self.assertRaises(TimeoutError):
                tcp_operation('localhost',{'operations':seer_io_operations()},'set_do',{'channel':3,'value':True})
            client.request.assert_called_once()
            self.assertEqual(client.close.call_count,2)
