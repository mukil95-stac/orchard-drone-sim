"""ADRC position and attitude control used by the manually armed simulator."""

import math
import numpy as np

from .core import MotorMixer


def lidar_avoidance_acceleration(ranges, yaw, safe_distance=2.0,
                                 max_distance=3.0, gain=1.8,
                                 max_acceleration=1.0):
    """Convert top LiDAR ranges into a world-frame horizontal repulsion."""
    values = np.asarray(ranges, dtype=float).reshape(-1)
    if values.size == 0:
        return np.zeros(3)
    acceleration_body = np.zeros(2)
    for beam, distance in enumerate(values):
        # MuJoCo reports -1 when a rangefinder has no return.
        if not np.isfinite(distance) or distance < 0.05 or distance > max_distance:
            continue
        angle = 2. * math.pi * beam / values.size
        strength = gain * ((safe_distance - distance) / safe_distance) ** 2 if distance < safe_distance else 0.
        if strength <= 0.:
            continue
        direction = np.array([math.cos(angle), math.sin(angle)])
        acceleration_body -= strength * direction
    cy, sy = math.cos(yaw), math.sin(yaw)
    world = np.array([cy * acceleration_body[0] - sy * acceleration_body[1],
                      sy * acceleration_body[0] + cy * acceleration_body[1]])
    norm = np.linalg.norm(world)
    if norm > max_acceleration:
        world *= max_acceleration / norm
    return np.array([world[0], world[1], 0.])


class PIDPositionController:
    """World-frame PID position controller returning requested acceleration."""

    def __init__(self, tuning):
        self.kp = float(tuning.get('position_kp', 0.55))
        self.ki = float(tuning.get('position_ki', 0.03))
        self.kd = float(tuning.get('position_kd', 1.35))
        self.max_acceleration = float(tuning.get('max_acceleration', 0.65))
        self.integral_limit = float(tuning.get('integral_limit', 1.5))
        self.integral = np.zeros(3)

    def reset(self):
        self.integral.fill(0.)

    def update(self, position, velocity, target, dt):
        error = np.asarray(target, dtype=float) - np.asarray(position, dtype=float)
        velocity = np.asarray(velocity, dtype=float)
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        acceleration = self.kp * error + self.ki * self.integral - self.kd * velocity
        return np.clip(acceleration, -self.max_acceleration, self.max_acceleration)


class ArmedADRCController:
    def __init__(self, params, tuning):
        self.params = params
        self.mixer = MotorMixer(params)
        self.position = PIDPositionController(tuning)
        self.max_tilt = tuning['max_tilt']
        self.acceleration_slew_rate = tuning.get('acceleration_slew_rate',float('inf'))
        self.target_slew_rate = tuning.get('target_slew_rate', float('inf'))
        self.rpm_slew_rate = tuning.get('rpm_slew_rate', float('inf'))
        self.horizontal_acceleration_limit = tuning.get('horizontal_acceleration_limit', float('inf'))
        self.lidar_safe_distance = tuning.get('lidar_safe_distance', 2.0)
        self.lidar_max_distance = tuning.get('lidar_max_distance', 3.0)
        self.lidar_avoidance_gain = tuning.get('lidar_avoidance_gain', 1.8)
        self.lidar_avoidance_max_acceleration = tuning.get('lidar_avoidance_max_acceleration', 1.0)
        self.acceleration_command = np.zeros(3)
        self.target_command = None
        self.rpm_command = np.zeros(4)
        self.roll_kp = tuning['roll_kp']
        self.pitch_kp = tuning['pitch_kp']
        self.yaw_kp = tuning['yaw_kp']
        self.rate_kd = tuning['rate_kd']

    def reset(self):
        self.position.reset()
        self.acceleration_command.fill(0.)
        self.target_command = None
        self.rpm_command.fill(0.)

    def update(self, position, velocity, attitude, gyro, target, dt, desired_yaw=None, lidar=None):
        """Return motor RPM for an EKF position/velocity target."""
        roll, pitch, yaw = attitude
        # Interpolate waypoint changes so takeoff, transit, and final approach
        # do not create a sudden acceleration request.
        target = np.asarray(target, dtype=float)
        if self.target_command is None:
            self.target_command = np.asarray(position, dtype=float).copy()
        max_target_change = self.target_slew_rate * dt
        self.target_command += np.clip(target - self.target_command,
                                       -max_target_change, max_target_change)
        requested_acceleration = self.position.update(position, velocity, self.target_command, dt)
        if lidar is not None:
            requested_acceleration += lidar_avoidance_acceleration(
                lidar, yaw, self.lidar_safe_distance, self.lidar_max_distance,
                self.lidar_avoidance_gain, self.lidar_avoidance_max_acceleration)
        max_change = self.acceleration_slew_rate * dt
        acceleration_delta = np.clip(requested_acceleration-self.acceleration_command,-max_change,max_change)
        self.acceleration_command += acceleration_delta
        acceleration = self.acceleration_command
        horizontal_limit = self.horizontal_acceleration_limit
        acceleration[:2] = np.clip(acceleration[:2], -horizontal_limit, horizontal_limit)
        if desired_yaw is None:
            desired_yaw = yaw
        # Compensate using the current heading so position control stays correct while yaw is turning.
        cy, sy = math.cos(yaw), math.sin(yaw)
        desired_roll = np.clip((sy * acceleration[0] - cy * acceleration[1]) / 9.81, -self.max_tilt, self.max_tilt)
        desired_pitch = np.clip((cy * acceleration[0] + sy * acceleration[1]) / 9.81, -self.max_tilt, self.max_tilt)
        thrust = self.params.mass * (9.81 + acceleration[2]) / max(.75, math.cos(roll) * math.cos(pitch))
        yaw_error = (desired_yaw - yaw + math.pi) % (2. * math.pi) - math.pi
        torques = np.array([
            self.roll_kp * (desired_roll - roll) - self.rate_kd * gyro[0],
            self.pitch_kp * (desired_pitch - pitch) - self.rate_kd * gyro[1],
            self.yaw_kp * yaw_error - self.rate_kd * gyro[2],
        ])
        requested_rpm = self.mixer.mix(thrust, *torques)
        max_rpm_change = self.rpm_slew_rate * dt
        self.rpm_command += np.clip(requested_rpm - self.rpm_command,
                                    -max_rpm_change, max_rpm_change)
        return np.clip(self.rpm_command, 0., self.params.maximum_rpm)
