#!/usr/bin/env python3
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from orchard_drone.runner import Simulation
import argparse
p=argparse.ArgumentParser(); p.add_argument('--max-time',type=float,default=240); p.add_argument('--headless',action='store_true'); p.add_argument('--map',action='store_true'); p.add_argument('--reset-mission',action='store_true'); p.add_argument('--manual',action='store_true',help='open the ADRC-controlled manual marker approach'); a=p.parse_args()
s=Simulation()
if a.reset_mission: s.mission.reset()
s.print_scene_poses()
print(s.run_manual(a.max_time) if a.manual else s.run(a.max_time,a.headless,a.map))
