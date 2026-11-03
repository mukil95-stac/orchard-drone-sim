#!/usr/bin/env python3
"""Open the orchard world without running a flight controller."""

from pathlib import Path
import argparse
import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    parser = argparse.ArgumentParser(description="View the orchard MuJoCo world")
    parser.add_argument("--model", type=Path, default=ROOT / "models" / "orchard.xml")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.model))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()

