# Controller

This part owns the drone behavior running on the MuJoCo world:

- `../orchard_drone/core.py` contains ADRC, EKF, mission, battery, mixer, and
  vision logic.
- `../orchard_drone/armed_controller.py` contains the armed/manual controller.
- The manual waypoint controller uses PID position control with velocity
  damping and integral anti-windup.
- `../orchard_drone/runner.py` connects the controller to the world simulator.
- `run_controller.py` is the controller entry point.

Run autonomous control:

```bash
cd /home/mukil/orchard_drone_sim
python3 controller/run_controller.py
```

Run manual control:

```bash
cd /home/mukil/orchard_drone_sim
python3 controller/run_controller.py --manual
```

Run headless control:

```bash
cd /home/mukil/orchard_drone_sim
python3 controller/run_controller.py --headless --reset-mission
```

