"""Standalone entry point for the shared MOMA vision/panel workspace."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from seer_control.key_panel_workspace import main
if __name__=='__main__':main()
