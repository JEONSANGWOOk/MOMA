import copy
import unittest

from seer_control.opentcs_bridge import OpenTCSRobot, convert_map, LOOPBACK


class FakeAPI:
    def __init__(self):
        self.calls=[];self.adapter=LOOPBACK;self.fail=False
        self.model=dict(name='demo',points=[dict(name=n,position=dict(x=x,y=0),type='HALT_POSITION') for n,x in [('A',1000),('B',3000)]],
                        paths=[dict(name='AB',srcPointName='A',destPointName='B',maxVelocity=1000,maxReverseVelocity=0,locked=False)])
        self.vehicle=dict(name='AGV-01',currentPosition='A',state='IDLE',paused=False,energyLevel=80,transportOrder=None)
        self.order=dict(state='BEING_PROCESSED',processingVehicle='AGV-01')
    def call(self,method,path,body=None):
        self.calls.append((method,path,body))
        if self.fail:raise ConnectionError('offline')
        if path=='plantModel':return copy.deepcopy(self.model)
        if path.endswith('/attachmentInformation'):return dict(attachedCommAdapter=self.adapter)
        if path=='vehicles/AGV-01':return copy.deepcopy(self.vehicle)
        if '/paused?' in path:self.vehicle['paused']=path.endswith('true');return None
        if '/withdrawal?' in path:self.order['state']='FAILED';self.vehicle['transportOrder']=None;return None
        if path.startswith('transportOrders/'):
            if method=='POST':self.order=dict(state='RAW',processingVehicle=None);return copy.deepcopy(self.order)
            return copy.deepcopy(self.order)
        raise AssertionError(path)


class OpenTCSBridgeTests(unittest.TestCase):
    def setUp(self):self.api=FakeAPI();self.robot=OpenTCSRobot(api=self.api);self.robot.request(4005,dict(nick_name='test'))
    def tearDown(self):self.robot.close()
    def test_map_units_and_one_way_paths(self):
        model=convert_map(self.api.model)
        self.assertEqual(model.nodes['A']['x'],1)
        self.assertEqual(model.route('A','B'),['A','B'])
        with self.assertRaises(ValueError):model.route('B','A')
    def test_locked_and_reverse_paths(self):
        self.api.model['paths'][0]['maxReverseVelocity']=1000
        self.assertEqual(convert_map(self.api.model).route('B','A'),['B','A'])
        self.api.model['paths'][0]['locked']=True
        with self.assertRaises(ValueError):convert_map(self.api.model).route('A','B')
    def test_navigation_is_real_acs_order_and_not_early_complete(self):
        result=self.robot.request(3051,dict(id='B'))
        self.assertEqual(result['ret_code'],0)
        command=next(c for c in self.api.calls if c[0]=='POST')
        self.assertEqual(command[2],dict(intendedVehicle='AGV-01',destinations=[dict(locationName='B',operation='MOVE')]))
        self.assertEqual(self.robot.status()['task_status'],1)
        self.api.order['state']='BEING_PROCESSED';self.robot.refresh()
        self.assertEqual(self.robot.status()['task_status'],2)
        self.api.order['state']='FINISHED';self.api.vehicle['currentPosition']='B';self.robot.refresh()
        self.assertEqual(self.robot.status()['task_status'],4)
        self.assertEqual(self.robot.status()['x'],3)
    def test_pause_resume_cancel_status(self):
        self.robot.request(3051,dict(id='B'));self.api.order['state']='BEING_PROCESSED';self.robot.refresh()
        self.assertEqual(self.robot.request(3001)['ret_code'],0)
        self.assertEqual(self.robot.status()['task_status'],3)
        self.robot.request(3002);self.assertEqual(self.robot.status()['task_status'],2)
        self.robot.request(3003);self.robot.refresh()
        self.assertEqual(self.robot.status()['task_status'],6)
    def test_external_orders_not_overwritten_or_canceled(self):
        self.api.vehicle['transportOrder']='other';self.robot.refresh()
        self.assertNotEqual(self.robot.request(3051,dict(id='B'))['ret_code'],0)
        self.robot.request(3003)
        self.assertFalse(any('/withdrawal' in c[1] for c in self.api.calls))
        self.assertEqual(self.robot.status()['task_status'],2)
        self.assertEqual(self.robot.status()['acs_order'],'other')
    def test_actual_vehicle_adapter_rejected_before_post(self):
        self.api.adapter='actual-driver'
        self.assertNotEqual(self.robot.request(3051,dict(id='B'))['ret_code'],0)
        self.assertFalse(any(c[0]=='POST' for c in self.api.calls))
    def test_connection_failure_cannot_be_success(self):
        self.api.fail=True
        self.assertNotEqual(self.robot.request(3051,dict(id='B'))['ret_code'],0)
        self.assertFalse(self.robot.order_name)
    def test_unsupported_operations_invalid_goal_and_port(self):
        for api,p in [(2010,dict(vx=.1)),(2002,dict(x=0,y=0,angle=0)),(4010,{}),(3051,dict(id='missing'))]:
            self.assertNotEqual(self.robot.request(api,p)['ret_code'],0)
        self.assertNotEqual(self.robot.request(3051,dict(id='B'),19204)['ret_code'],0)
    def test_sensor_and_speed_are_explicitly_unavailable(self):
        self.assertFalse(self.robot.request(1101)['sensor_available'])
        self.assertFalse(self.robot.status()['speed_available'])
        self.assertEqual(self.robot.status()['pose_source'],'node')


if __name__=='__main__':unittest.main()
