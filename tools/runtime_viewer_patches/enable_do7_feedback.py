"""Add read-only DO7 feedback, isolated from vacuum getter failures."""
import ast
import shutil
import sys
import time
from pathlib import Path

def install(path):
    path=Path(path);text=path.read_text(encoding='utf-8');tree=ast.parse(text)
    fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='read_vacuum_io')
    replacement='''def read_vacuum_io(moma):
    # OutputSignal overload with two arguments is GET only.
    try:
        outputs={str(i):moma.OutputSignal(moma.CNTMOD.CNTROB,i) for i in (2,3,4,5)}
        if any(type(v) is not bool for v in outputs.values()):
            raise ValueError('Unexpected vacuum output feedback type')
        try:
            do7=moma.OutputSignal(moma.CNTMOD.CNTROB,7)
            if type(do7) is bool:outputs['7']=do7
        except Exception:
            pass  # DO7 unavailable: preserve independent vacuum feedback.
        return {'valid':True,'outputs':outputs,'sample_time':time.monotonic()}
    except Exception:
        return {'valid':False,'outputs':{},'sample_time':time.monotonic()}
'''
    old=ast.get_source_segment(text,fn)
    if old.strip()==replacement.strip():return
    lines=text.splitlines(True);lines[fn.lineno-1:fn.end_lineno]=[replacement]
    updated=''.join(lines);compile(updated,str(path),'exec')
    shutil.copy2(path,str(path)+'.bak-do7-'+time.strftime('%Y%m%d-%H%M%S'))
    path.write_text(updated,encoding='utf-8')

if __name__=='__main__':install(sys.argv[1])
