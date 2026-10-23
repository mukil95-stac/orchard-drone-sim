# Orchard Whoop MuJoCo simulation

Runnable modular simulation of a 3×3 apple orchard inspection mission. Python 3.10+, MuJoCo, NumPy, SciPy and OpenCV contrib are required. ROS 2 Humble is supported by the optional ROS node adapters.

## Run

```bash
cd ~/orchard_drone_sim
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
python scripts/generate_aruco_markers.py
python scripts/run_simulation.py --headless --max-time 240 --reset-mission
pytest -q
```

To view the live MuJoCo viewer, omit `--headless`; to see the map use `--map`. In ROS 2 Humble, `source /opt/ros/humble/setup.bash`, then `colcon build --symlink-install --base-paths ros2_ws/src`; run `ros2 launch orchard_bringup simulation.launch.py`. The same runner works without ROS.

For manual flight, run `python3 scripts/run_simulation.py --manual`. The simulation opens the MuJoCo scene and a separate live front-camera window. Press **X** to arm: the drone rises vertically, then autonomously flies to the first configured tree (currently tree 04, row 2 column 1). It uses nearby ArUco observations for position correction and approaches a point 4 m in front of the target marker. At arrival, it scans the apple on tree 04, saves an annotated camera image and JSON report under `data/scans/`, then disarms. Adjust the flight profile and ADRC gains in `config/armed_controller.yaml`. Press **Esc** or close the viewer to stop.

## Components and data flow

`simulator` steps MuJoCo and applies delayed motor thrust and reaction torque. Noisy IMU and rendered RGB frames feed `aruco`/`ekf`; marker poses come from OpenCV PnP and a surveyed marker map. EKF state feeds the ADRC controller; its wrench request passes through the explicit X quad mixer and motor lag. The inspection order starts at middle-row marker 04, continues across 05 and 06, then visits 03, 02, 01, 07, 08, and 09. Battery power is estimated from all four rotor speeds. A* or Dijkstra operates on the orchard graph and selects a reachable safe node on low energy. JSON records completion and both trajectories.

The orchard and helipad sit inside a 1.35 m tall perimeter fence spanning x = −12 to 12 m and y = −16.5 to 11.5 m. The runner prints each tree and helipad world pose at startup as position in metres and quaternion in w,x,y,z order.

## ROS 2 topics

`/imu` (`sensor_msgs/Imu`), `/motor_rpm` (`Float32MultiArray`), `/camera/image_raw` (`Image`), `/aruco_detections` (`MarkerArray`), `/drone_pose` (`Odometry`), `/motor_commands` (`Float32MultiArray`), `/waypoint_command` (`PoseStamped`), `/battery_state` (`BatteryState`), `/planned_path` (`Path`). `ros2 launch orchard_bringup simulation.launch.py` starts the simulation bridge and modular ROS nodes.

## Configuration

`config/drone.yaml` includes mass, arm, thrust/drag coefficients, RPM bounds, rotor time constant and noise. `config/adrc.yaml` controls autonomous mission tracking. `config/armed_controller.yaml` tunes the armed manual ADRC controller. `config/battery.yaml` sets pack energy and reserve. `config/mission.yaml` controls view offset, altitude, inspection dwell and safe-node policy. `config/ekf.yaml` sets measurement covariance.

## Krishi Drone review and attribution

Reviewed the requested public repo (branch `kd_hw`) and the locally available matching `controller_tuner`, `swift_pico_hw`, `swift_testing`, and `whycon` packages. Adapted concepts only: six-component desired/current state and ROS topic separation from `swift_pico` controllers; explicit rotor speed and odometry/image interfaces from the bridge config; parameterized controller tuning and ROS launch layout; vision detector packaging from WhyCon. No source code or Gazebo files were copied. The repository root has no visible license file on its default branch; no source code from it or its vendored WhyCon package was copied, so this project does not rely on that repository's license. Repository: https://github.com/eYantra-Robotics-Competition/eyrc-25-26_krishi_drone

## Limitations

The model is a low-speed research-grade rigid-body quadrotor, not a CFD propeller/duct simulation; rotor aerodynamics are represented by thrust/drag coefficients and lag. Marker world locations are surveyed map data, while image corners and relative pose are obtained from the rendered image. OpenCV ArUco support requires `opencv-contrib-python`; standard `opencv-python` is insufficient. ROS image and localization adapters are modular, while the single-process runner owns MuJoCo stepping for deterministic headless tests.

## 9 m × 5 m wheeled robot arena

A separate four-wheel differential-drive robot model is provided in `models/arena_bot.xml`. The chassis is 40 × 30 × 15 cm, with 12 cm diameter wheels. The camera, LiDAR, and IMU are functional MuJoCo sensors: the front camera is 25 cm above the chassis top; the 36-ray horizontal rangefinder array is 5 cm above it; and the accelerometer, gyroscope, and orientation sensor are 2 cm above it. Sensor height is measured from the chassis top.

Run it with `python3 scripts/run_arena_bot.py`. In the MuJoCo viewer, tap ↑/↓ to raise/lower forward speed, ←/→ to steer, and Space to stop and clear steering. A separate window shows the live front camera.
