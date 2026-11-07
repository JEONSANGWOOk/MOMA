#!/usr/bin/env python3
"""Forward the desktop's SEER TCP connections through the MoMa PC."""
import argparse
import asyncio
import logging
import ipaddress
import signal
import socket


def client_allowed(address,allowed):
    try:return ipaddress.ip_address(address) in ipaddress.ip_network(allowed,strict=False)
    except ValueError:return False


def configure(sock):
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    for name, value in (('TCP_KEEPIDLE', 30), ('TCP_KEEPINTVL', 10), ('TCP_KEEPCNT', 3)):
        if hasattr(socket, name):
            sock.setsockopt(socket.IPPROTO_TCP, getattr(socket, name), value)


async def copy_stream(reader, writer):
    while True:
        data = await reader.read(65536)
        if not data:
            if writer.can_write_eof():
                writer.write_eof()
                await writer.drain()
            return
        writer.write(data)
        await writer.drain()


async def relay(reader, writer, target, port, allowed):
    peer = writer.get_extra_info('peername')
    upstream = None
    tasks = []
    try:
        if not peer or not client_allowed(peer[0],allowed):
            logging.warning('Rejected peer %s', peer)
            return
        configure(writer.get_extra_info('socket'))
        remote, upstream = await asyncio.wait_for(asyncio.open_connection(target, port), 3.)
        configure(upstream.get_extra_info('socket'))
        tasks = [asyncio.create_task(copy_stream(reader, upstream)),
                 asyncio.create_task(copy_stream(remote, writer))]
        # Preserve half-close: a complete request may still have a pending reply.
        await asyncio.gather(*tasks)
    except (OSError, asyncio.TimeoutError) as error:
        logging.warning('Connection %s -> %s:%s ended: %s', peer, target, port, error)
    finally:
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        for stream in (upstream, writer):
            if stream is not None:
                stream.close()
                try:
                    await stream.wait_closed()
                except OSError:
                    pass


async def serve(args):
    stopped = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stopped.set)
    servers = []
    try:
        for port in args.ports:
            def connected(reader, writer, port=port):
                return relay(reader, writer, args.target, port, args.allow_client)
            server = await asyncio.start_server(connected, args.listen, port)
            servers.append(server)
            logging.info('%s:%s -> %s:%s; client=%s', args.listen, port,
                         args.target, port, args.allow_client)
        await stopped.wait()
    finally:
        for server in servers:
            server.close()
        await asyncio.gather(*(server.wait_closed() for server in servers))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--listen', default='192.168.30.13')
    parser.add_argument('--target', default='192.168.58.102')
    parser.add_argument('--allow-client', default='192.168.30.0/24',help='Allowed client IP or CIDR subnet')
    parser.add_argument('--ports', nargs='+', type=int, default=[19204,19205,19206,19207,19210])
    args = parser.parse_args()
    try:ipaddress.ip_network(args.allow_client,strict=False)
    except ValueError:parser.error('--allow-client must be an IP address or CIDR subnet')
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    asyncio.run(serve(args))


if __name__ == '__main__':
    main()
