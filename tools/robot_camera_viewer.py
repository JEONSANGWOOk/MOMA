"""Open the robot-mounted camera without restarting the main GUI."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from seer_control.robot_camera import main

if __name__=='__main__':main()
