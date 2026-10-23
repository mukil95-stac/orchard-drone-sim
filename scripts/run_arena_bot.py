#!/usr/bin/env python3
"""Run the four-wheel 9 m by 5 m arena robot with the MuJoCo viewer."""
from pathlib import Path
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "models" / "arena_bot.xml"
model = mujoco.MjModel.from_xml_path(str(MODEL))
data = mujoco.MjData(model)
wheel_joints = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"wheel_{name}_hinge")
                for name in ("fl", "fr", "rl", "rr")]
wheel_dofs = [model.jnt_dofadr[joint] for joint in wheel_joints]
lidar_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, f"lidar_{i:02d}_range")
             for i in range(36)]
lidar_adr = np.array([model.sensor_adr[i] for i in lidar_ids])
imu_accel_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_accelerometer")
imu_gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyroscope")
imu_accel_adr = model.sensor_adr[imu_accel_id]
imu_gyro_adr = model.sensor_adr[imu_gyro_id]
throttle = 0.0
steering = 0.0


def key_callback(key):
    global throttle, steering
    # MuJoCo's passive viewer callback passes one GLFW key code per press.
    # GLFW arrow codes are stable: left 263, right 262, down 264, up 265.
    if key == 265:
        throttle = min(8.0, throttle + 1.0)
    elif key == 264:
        throttle = max(-8.0, throttle - 1.0)
    elif key == 263:
        steering = min(5.0, steering + 1.0)
    elif key == 262:
        steering = max(-5.0, steering - 1.0)
    elif key in (32, ord(" ")):
        throttle = steering = 0.0
    print(f"\rWheel targets [rad/s]: left={throttle-steering:.1f} right={throttle+steering:.1f}     ", end="", flush=True)


print("Arena bot | ↑ forward, ↓ reverse, ←/→ steer, Space stop, Esc close")
print("Tap arrows to adjust speed and steering; Space clears both commands.")
with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    viewer.cam.lookat[:] = [0.0, 0.0, 0.4]
    viewer.cam.distance = 8.5
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -35
    renderer = mujoco.Renderer(model, height=480, width=640)
    try:
        while viewer.is_running():
            start = time.perf_counter()
            left, right = throttle - steering, throttle + steering
            targets = (left, right, left, right)
            # Wheel-speed feedback with torque limits keeps the command in rad/s.
            for i, dof in enumerate(wheel_dofs):
                error = targets[i] - data.qvel[dof]
                data.ctrl[i] = np.clip(2.0 * error, -12.0, 12.0)
            mujoco.mj_step(model, data)
            viewer.sync()
            if int(data.time) != int(data.time - model.opt.timestep):
                front_range = data.sensordata[lidar_adr[0]]
                accel = data.sensordata[imu_accel_adr:imu_accel_adr + 3]
                gyro = data.sensordata[imu_gyro_adr:imu_gyro_adr + 3]
                print(f"\nLiDAR forward={front_range:.2f} m | IMU accel={accel.round(2)} m/s² gyro={gyro.round(2)} rad/s")
            if int(data.time * 10) != int((data.time - model.opt.timestep) * 10):
                renderer.update_scene(data, camera="front_webcam")
                rgb = renderer.render()
                try:
                    import cv2
                    cv2.imshow("Front webcam", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                    cv2.waitKey(1)
                except ImportError:
                    pass
            time.sleep(max(0.0, model.opt.timestep - (time.perf_counter() - start)))
    finally:
        renderer.close()
        try:
            import cv2
            cv2.destroyAllWindows()
        except ImportError:
            pass
        print("\nStopped.")
