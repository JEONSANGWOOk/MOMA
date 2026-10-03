"""Private JSON pipe endpoint; SDK stdout must not corrupt responses."""
import contextlib
import json
import sys
from .fairino_api import SDKEngine


def main():
    output=sys.stdout
    engine=None
    for line in sys.stdin:
        try:
            request=json.loads(line)
            with contextlib.redirect_stdout(sys.stderr):
                if engine is None:engine=SDKEngine(request['config'])
                result=engine.call(request['kind'],request.get('operation'))
            response=dict(ok=True,result=result)
        except Exception as e:response=dict(ok=False,error=str(e))
        output.write(json.dumps(response,ensure_ascii=False)+'\n');output.flush()


if __name__=='__main__':main()
