"""Download official FR5 assets for Windows rendering; ROS is not required."""
import argparse,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from seer_control.fairino_model import install
parser=argparse.ArgumentParser();parser.add_argument('--repo',help='existing frcobot_ros2 checkout');parser.add_argument('--target',default=str(Path.home()/'.seer_amr_console'))
args=parser.parse_args();print(install(args.target,args.repo))
