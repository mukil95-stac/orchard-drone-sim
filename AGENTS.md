# Development notes

- Run `python3 models/generate_models.py` after changing the orchard MJCF generator.
- Run `python3 scripts/generate_aruco_markers.py` to regenerate DICT_4X4_50 assets; requires OpenCV contrib.
- Keep state estimation and all navigation/controller inputs sensor-derived. Ground truth is telemetry and evaluation only.
- Keep return-energy behavior graph-based; do not add a fixed home position.
- Run `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q` in this host image, whose user pytest plugins are incompatible with system pytest.
- ROS 2 entry point is `orchard_bringup.simulation_node`; MuJoCo render uses EGL for headless execution.
