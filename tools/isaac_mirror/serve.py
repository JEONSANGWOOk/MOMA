"""Read-only HTTP bridge for an Isaac Sim desktop mirror."""
import argparse
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
import time
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from seer_control.isaac_mirror import directory


def handler(root):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path in ('/health','/client-ready'):
                if self.path=='/client-ready':self.server.client_seen=time.monotonic()
                data=json.dumps(dict(service='moma-isaac-mirror',client_age_s=(time.monotonic()-self.server.client_seen) if getattr(self.server,'client_seen',0) else None)).encode()
                self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data);return
            name={'/manifest':'manifest.json','/frame':'frame.json'}.get(self.path)
            if not name:self.send_error(404);return
            compressed=name=='manifest.json' and (root/'manifest.json.gz').is_file()
            try:data=(root/('manifest.json.gz' if compressed else name)).read_bytes()
            except OSError:self.send_error(503,'Start the desktop mirror and open the 3D view');return
            self.send_response(200);self.send_header('Content-Type','application/json')
            if compressed:self.send_header('Content-Encoding','gzip')
            self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
        def log_message(self,*args):pass
    return Handler


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default='127.0.0.1');parser.add_argument('--port',type=int,default=8770)
    parser.add_argument('--directory',type=Path,default=directory());args=parser.parse_args()
    server=ThreadingHTTPServer((args.host,args.port),handler(args.directory))
    print(f'Read-only Isaac mirror: http://{args.host}:{args.port} ; data={args.directory}',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()

if __name__=='__main__':main()
