import asyncio
import importlib.util
import unittest
from pathlib import Path

path=Path(__file__).resolve().parents[1]/'tools/amr_relay/relay.py'
spec=importlib.util.spec_from_file_location('amr_relay',path)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class RelayTests(unittest.IsolatedAsyncioTestCase):
    async def test_large_payload_and_delayed_reply_after_half_close(self):
        received=[]
        async def backend(reader,writer):
            data=await reader.read()
            received.append(data)
            await asyncio.sleep(.02)
            writer.write(data[::-1])
            await writer.drain()
            writer.close()
            await writer.wait_closed()

        remote=await asyncio.start_server(backend,'127.0.0.1',0)
        port=remote.sockets[0].getsockname()[1]
        pending=[]
        def handle(reader,writer):
            pending.append(asyncio.create_task(module.relay(reader,writer,'127.0.0.1',port,'127.0.0.1')))
        server=await asyncio.start_server(handle,'127.0.0.1',0)
        try:
            reader,writer=await asyncio.open_connection('127.0.0.1',server.sockets[0].getsockname()[1])
            data=bytes(range(256))*4096
            writer.write(data)
            await writer.drain()
            writer.write_eof()
            reply=await asyncio.wait_for(reader.read(),3)
            self.assertEqual(reply,data[::-1])
            self.assertEqual(received,[data])
            writer.close()
            await writer.wait_closed()
            await asyncio.wait_for(asyncio.gather(*pending),3)
        finally:
            server.close();remote.close()
            await server.wait_closed();await remote.wait_closed()

    async def test_other_client_rejected(self):
        pending=[]
        def handle(reader,writer):
            pending.append(asyncio.create_task(module.relay(reader,writer,'127.0.0.1',1,'192.168.30.12')))
        server=await asyncio.start_server(handle,'127.0.0.1',0)
        try:
            reader,writer=await asyncio.open_connection('127.0.0.1',server.sockets[0].getsockname()[1])
            self.assertEqual(await asyncio.wait_for(reader.read(),1),b'')
            writer.close()
            await writer.wait_closed()
            await asyncio.gather(*pending)
        finally:
            server.close()
            await server.wait_closed()


if __name__=='__main__':unittest.main()
