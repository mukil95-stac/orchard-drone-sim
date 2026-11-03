# 🤖 AI-Picking: Robust Vision-Based Robotic Picking System

A computer vision and robotics project designed to improve the reliability of robotic pick-and-place operations using ArUco markers, durable marker surfaces, and intelligent visual localization.

## 📌 Overview

AI-Picking aims to develop a robotic picking system that identifies target objects, estimates their position and orientation, and guides a robotic manipulator to perform accurate picking operations.

Traditional vision-based systems often depend on printed visual markers, which can become damaged, dirty, or difficult to detect under challenging environmental conditions. This project explores the use of durable, high-contrast engraved markers and reliable vision-processing techniques to improve localization robustness.

The long-term goal is to build a cost-effective and adaptable robotic picking system suitable for real-world environments.

## 🎯 Objectives

- Detect and identify ArUco markers using computer vision.
- Estimate marker position and orientation using camera calibration.
- Improve marker durability through engraving or laser marking on suitable rigid materials.
- Investigate marker performance under scratches, contamination, partial occlusion, and changing lighting.
- Integrate vision-based localization with robotic pick-and-place operations.
- Develop safe recovery strategies when marker detection or pose estimation becomes unreliable.

## 💡 Proposed Solution

The project combines computer vision, physical marker design, and robotic manipulation.

### 1. Vision-Based Localization
Use a camera and OpenCV's ArUco detection capabilities to identify marker corners and estimate their pose.

### 2. Durable Marker Design
Explore engraved or laser-marked, high-contrast patterns as an alternative to conventional printed markers. Potential materials include rigid plastic and metal. PVC may also be evaluated as a marker carrier, preferably with a flat mounting surface.

### 3. Reliability Monitoring
Evaluate marker detection quality, pose stability, and geometric consistency before sending coordinates to the robot.

### 4. Robotic Picking
Transform the estimated coordinates into the robot's reference frame and use them to guide a robotic arm and gripper toward the target.

### 5. Safe Recovery
If the marker cannot be detected reliably, the system should attempt reacquisition, use another validated reference if available, or stop the picking operation safely.

## ⚙️ System Workflow

1. Capture an image using the camera.
2. Detect the ArUco marker using OpenCV.
3. Validate the detected marker and its geometry.
4. Estimate the marker's position and orientation.
5. Transform the pose into the robot coordinate frame.
6. Plan a collision-aware approach to the target.
7. Execute the picking operation.
8. Verify success or enter a safe recovery state.

## 🧰 Technologies

- **Python** — Main programming language for vision processing and system integration.
- **OpenCV** — Image processing, ArUco detection, camera calibration, and pose estimation.
- **ROS 2** — Communication between vision, planning, and robot-control components, if used in the implementation.
- **Camera** — Captures the scene for visual localization.
- **Robotic Arm and Gripper** — Performs the physical pick-and-place operation.
- **Engraved Marker Prototype** — A proposed durable alternative to a conventional printed marker.
- **Simulation Tools** — Optional tools for validating robot movement, coordinate transformations, and collision avoidance.

## 🏗️ System Architecture

```text
Camera Input
     |
     v
Image Processing (OpenCV)
     |
     v
ArUco Marker Detection
     |
     v
Detection and Pose Validation
     |
     v
3D Pose Estimation
     |
     v
Camera-to-Robot Coordinate Transform
     |
     v
Robot Motion Planning
     |
     v
Robotic Arm and Gripper
     |
     v
Pick Verification / Safe Recovery
```

The project is also exposed as two explicit parts:

- `world/` contains the MuJoCo world viewer and points to the generated files
  in `models/`.
- `controller/` contains the controller entry point and connects it to the
  world through `orchard_drone/runner.py`.

View only the world with `python3 world/run_world.py`, or run the controller
with `python3 controller/run_controller.py`. The original
`python3 scripts/run_simulation.py` command remains supported.

To view the live MuJoCo viewer, omit `--headless`; to see the map use `--map`. In ROS 2 Humble, `source /opt/ros/humble/setup.bash`, then `colcon build --symlink-install --base-paths ros2_ws/src`; run `ros2 launch orchard_bringup simulation.launch.py`. The same runner works without ROS.

For manual flight, run `python3 scripts/run_simulation.py --manual`. The simulation opens the MuJoCo scene and a separate live front-camera window. Press **X** to arm: the drone rises vertically, then autonomously flies to a randomly selected remaining orchard tree. It uses nearby ArUco observations for position correction and approaches a point 4 m in front of the target marker. The top-mounted LiDAR adds horizontal repulsion when an obstacle is within 2 m, helping the controller steer around trees and the fence. At arrival, it scans the apple, saves an annotated camera image and JSON report under `data/scans/`, then disarms. The 16 LiDAR readings are available through `MuJoCoSim.lidar()` (`-1` means no hit). Press **Esc** or close the viewer to stop.

## 🛠️ Marker Durability Approach

Conventional printed markers are easy to produce but may deteriorate with repeated handling and environmental exposure.

This project investigates alternatives:

- **Engraved markers:** Physically mark the pattern into a rigid surface.
- **Laser-marked markers:** Create a durable, high-contrast pattern on compatible materials.
- **Rigid plastic or metal plates:** Provide a stable, flat mounting surface.
- **PVC-based mounting:** Explore an affordable carrier or protective structure, while avoiding excessive curvature in the camera-facing marker.
- **Multiple-marker arrangements:** Use several known markers or a marker board to improve localization availability when individual markers are obscured.

Durability alone is not sufficient. The pattern must retain clear black-and-white contrast, recognizable boundaries, and suitable dimensions for reliable camera detection.

## 🧪 Testing and Evaluation

The system can be evaluated under different conditions:

- Normal and low-light environments.
- Different camera distances and viewing angles.
- Surface scratches and contamination.
- Partial marker occlusion.
- Repeated handling and wear.
- Repeated robotic picking trials.

### Evaluation Metrics

| Metric | Purpose |
|---|---|
| Detection Rate | Measures how often the correct marker is detected. |
| Pose Estimation Error | Measures the difference between estimated and reference position and orientation. |
| Picking Success Rate | Measures the percentage of successful pick attempts. |
| Recovery Rate | Measures whether the system recovers safely from disturbed-marker conditions. |
| Processing Time | Measures the time required for detection and localization. |
| Durability | Evaluates marker performance after controlled wear and environmental exposure. |

Results should be recorded experimentally before making claims about improvements over conventional markers.

## ✅ Expected Benefits

- Improved physical durability compared with ordinary printed markers, depending on material and fabrication.
- More reliable localization under challenging conditions when supported by suitable validation and recovery mechanisms.
- A modular architecture that can be extended with additional markers and sensing methods.
- Compatibility with widely used computer vision and robotics technologies.
- A practical platform for studying robust robotic manipulation.

## 🚧 Challenges

- Maintaining sufficient contrast in engraved patterns.
- Preventing reflections and glare from marker surfaces.
- Handling partial damage and occlusion.
- Achieving accurate camera calibration and coordinate transformations.
- Preventing unsafe motion when pose estimates are uncertain.
- Ensuring that localization accuracy is sufficient for the target object's size and gripper.

## 🔮 Future Enhancements

- Integrate multiple ArUco markers or marker boards for more robust localization.
- Add object detection to identify targets without requiring a marker on every object.
- Explore alternative fiducial markers or complementary sensing techniques when a marker becomes unreadable.
- Implement automatic marker-quality monitoring and recovery logic.
- Add force, proximity, or gripper feedback to verify successful picks.
- Evaluate the system in a simulated environment before deployment on physical hardware.
- Conduct comparative durability and picking experiments across different marker materials.

## 🚀 Getting Started

### Prerequisites

- Python 3
- OpenCV with ArUco support
- A compatible camera
- Optional: ROS 2 and a supported robotic arm
- Optional: Simulation software for testing

### Installation

Create and activate a Python virtual environment:

```bash
python -m venv .venv
```

Activate it on Linux:

```bash
source .venv/bin/activate
```

On Windows:

```powershell
.venv\Scripts\activate
```

Install OpenCV with the required ArUco functionality:

```bash
pip install opencv-contrib-python
```

### Running the Project

The exact execution command depends on the implementation. Once the source files are added, document the appropriate script or ROS 2 launch command here.

For example:

```bash
python main.py
```

> Note: This command is illustrative. It will work only if the repository contains a compatible `main.py` entry point.

## 📁 Suggested Repository Structure

```text
AI-Picking/
├── README.md
├── requirements.txt
├── config/
│   └── camera_config.yaml
├── markers/
│   └── marker_generation.py
├── vision/
│   ├── detection.py
│   ├── pose_estimation.py
│   └── calibration.py
├── robot/
│   └── pick_and_place.py
├── tests/
│   └── evaluation.py
└── docs/
    └── system_architecture.md
```

This is a suggested structure; adjust it to match the actual files in the repository.

## 🤝 Contributions

Contributions and suggestions are welcome. Potential areas include marker fabrication, computer vision, pose estimation, robotics integration, testing, and reliability improvements.

## 📄 License

A license has not yet been specified. Add an appropriate license file before distributing the project for reuse.

## 👨‍💻 Project Status

**Status:** Prototype / Development

The project explores robust marker-based localization for robotic picking. Features should be marked as implemented only after they have been integrated and tested.

---

**AI-Picking — Building more reliable vision-based robotic manipulation, one pick at a time.** 🤖
