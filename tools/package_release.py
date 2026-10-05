"""Reproducible clean source ZIP, excluding logs, caches and credentials."""
import hashlib
import zipfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
EXCLUDED={'__pycache__','.pytest_cache','.git','.venv','venv','.delivery','build','dist','release'}


def release_files():
    for path in sorted(ROOT.rglob('*')):
        relative=path.relative_to(ROOT)
        if not path.is_file() or any(part in EXCLUDED for part in relative.parts):continue
        if path.suffix.lower() in ('.pyc','.pyo','.log','.spec') or path.name.startswith('.env'):continue
        if relative.as_posix()=='artifacts/fairino_tree.json':continue
        yield path


def main():
    target=ROOT.parent/'release';target.mkdir(exist_ok=True)
    archive=target/'MOMA_AMR_Control_Studio_v1.25_2026-10-05_source.zip'
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=9) as output:
        for path in release_files():
            info=zipfile.ZipInfo('MOMA/'+path.relative_to(ROOT).as_posix(),date_time=(2026,10,5,0,0,0))
            info.compress_type=zipfile.ZIP_DEFLATED;info.external_attr=(0o755 if path.suffix=='.sh' else 0o644)<<16
            output.writestr(info,path.read_bytes())
    with zipfile.ZipFile(archive) as verify:
        assert verify.testzip() is None
        count=len(verify.infolist())
    digest=hashlib.sha256(archive.read_bytes()).hexdigest()
    archive.with_suffix('.zip.sha256').write_text(digest+'  '+archive.name+'\n',encoding='utf-8')
    print(f'{archive}\n{count} files / {archive.stat().st_size} bytes\nSHA256 {digest}')


if __name__=='__main__':main()
