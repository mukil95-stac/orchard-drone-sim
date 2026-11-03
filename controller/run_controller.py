#!/usr/bin/env python3
"""Run the drone controller against the orchard world."""

from pathlib import Path
import argparse
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from orchard_drone.runner import Simulation


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the orchard drone controller")
    parser.add_argument("--max-time", type=float, default=240)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--map", action="store_true")
    parser.add_argument("--reset-mission", action="store_true")
    parser.add_argument("--manual", action="store_true")
    args = parser.parse_args()

    simulation = Simulation(ROOT)
    if args.reset_mission:
        simulation.mission.reset()
    simulation.print_scene_poses()
    result = (
        simulation.run_manual(args.max_time)
        if args.manual
        else simulation.run(args.max_time, args.headless, args.map)
    )
    print(result)


if __name__ == "__main__":
    main()

