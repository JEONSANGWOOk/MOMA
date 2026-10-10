"""Prepare this supplied product picture as a local, labelled SIM reference."""
import argparse,json,sys,uuid
from pathlib import Path
from PIL import Image
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from seer_control.vision_stream import publish_frame


def main():
    parser=argparse.ArgumentParser();parser.add_argument('photo',type=Path);args=parser.parse_args()
    root=Path(__file__).resolve().parents[1];folder=root/'.delivery';folder.mkdir(exist_ok=True)
    image=Image.open(args.photo)
    if image.size!=(709,1536):raise ValueError('이 등록 도구는 제공한 709×1536 제품 사진용입니다.')
    reference=folder/'kahl_screen_reference.png';image.crop((36,406,674,954)).save(reference)
    publish_frame(folder/'d455_photo_screen.json',dict(version=1,revision=uuid.uuid4().hex,reference_file=str(reference),
        hole_center_px=[318.,111.],reference_px_per_mm=2.8,dimension_source='photo_screen_estimate',product='KAHL-1057-B(R)'))
    print('Prepared local product picture / labelled upper lock center / SIM-only assumed scale')


if __name__=='__main__':main()
