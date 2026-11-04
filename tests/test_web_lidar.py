import asyncio
import math
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch,AsyncMock
from web_control.server import Control

class LidarTests(unittest.IsolatedAsyncioTestCase):
    def control(self):
        c=Control();c.mode='read';c.connected=True;c.rx=time.monotonic()
        c.state={'x':10.,'y':20.,'theta':math.pi/2};return c
    async def test_world_points_are_flat_xy_and_not_transformed_twice(self):
        c=self.control();c.lidar_alignment=(5,5,.5)
        self.assertTrue(c.receive_lidar({'laser_beams':[[1,2],[3,4]]},'1009',c.epoch))
        self.assertEqual(c.snapshot()['laser'],[[1.,2.],[3.,4.]])
        self.assertEqual(c.snapshot()['lidar']['count'],2)
    async def test_both_scanners_mounts_pose_and_saved_alignment(self):
        c=self.control();c.lidar_alignment=(.1,.2,0)
        payload={'lasers':[
            {'install_info':{'x':1,'y':0,'yaw':0},'beams':[{'angle':0,'range':1}]},
            {'install_info':{'x':-1,'y':0,'yaw':0},'beams':[{'angle':math.pi,'range':2}]}]}
        self.assertTrue(c.receive_lidar(payload,'1009',c.epoch))
        self.assertEqual(len(c.laser),2)
        self.assertAlmostEqual(c.laser[0][0],10.1);self.assertAlmostEqual(c.laser[0][1],22.2)
        self.assertAlmostEqual(c.laser[1][0],10.1);self.assertAlmostEqual(c.laser[1][1],17.2)
    async def test_stale_pose_and_previous_connection_do_not_publish(self):
        c=self.control();c.laser=[(1,2)]
        self.assertFalse(c.receive_lidar({'laser_beams':[[9,9]]},'1009',c.epoch-1))
        self.assertEqual(c.laser,[(1,2)])
        c.rx=0
        self.assertFalse(c.receive_lidar({'laser_beams':[[9,9]]},'1009',c.epoch))
    async def test_expired_lidar_not_displayed_as_live(self):
        c=self.control();c.laser=[(1,2)];c.laser_rx=time.monotonic()-3
        self.assertEqual(c.snapshot()['laser'],[])
        self.assertFalse(c.snapshot()['lidar']['connected'])
    async def test_1101_fallback_is_read_only_and_persistent(self):
        c=self.control();c.client=object()
        def request(api,payload):
            if api==1009:raise OSError('1009 unavailable')
            return {'_response_type':11101,'laser_beams':[[1,2]]}
        async def read(api,epoch):return request(api,None)
        c.read_status_api=AsyncMock(side_effect=read)
        task=asyncio.create_task(c.poll_lidar())
        try:
            end=time.monotonic()+1
            while not c.laser_rx and time.monotonic()<end:await asyncio.sleep(.01)
            self.assertEqual(c.laser,[(1.,2.)])
            self.assertEqual([call.args[0] for call in c.read_status_api.call_args_list],[1009,1101])
        finally:task.cancel();await asyncio.gather(task,return_exceptions=True)

    async def test_queries_reuse_the_same_status_socket_and_lock(self):
        c=self.control();c.client=SimpleNamespace(request=Mock(return_value={'DI':[True]}))
        result=await c.read_status_api(1013,c.epoch)
        self.assertEqual(result['_response_type'],11013)
        spec=c.client.request.call_args.args[0]
        self.assertTrue(spec['read_only']);self.assertEqual(spec['port'],19204)

if __name__=='__main__':unittest.main()
