"""ADRC position and attitude control used by the manually armed simulator."""

import math
import numpy as np

from .core import ADRC, MotorMixer


class ArmedADRCController:
    def __init__(self, params, tuning):
        self.params = params
        self.mixer = MotorMixer(params)
        self.position = ADRC(
            bw=tuning['position_bandwidth'],
            obs=tuning['observer_bandwidth'],
            vel_gain=tuning['velocity_gain'],
            max_acc=tuning['max_acceleration'],
        )
        self.max_tilt = tuning['max_tilt']
        self.acceleration_slew_rate = tuning.get('acceleration_slew_rate',float('inf'))
        self.acceleration_command = np.zeros(3)
        self.roll_kp = tuning['roll_kp']
        self.pitch_kp = tuning['pitch_kp']
        self.yaw_kp = tuning['yaw_kp']
        self.rate_kd = tuning['rate_kd']

    def reset(self):
        self.position.z1.fill(0.)
        self.position.z2.fill(0.)
        self.position.z3.fill(0.)
        self.position.initialized = False
        self.acceleration_command.fill(0.)

    def update(self, position, velocity, attitude, gyro, target, dt, desired_yaw=None):
        """Return motor RPM for an EKF position/velocity target."""
        roll, pitch, yaw = attitude
        requested_acceleration = self.position.update(position, velocity, target, dt)
        max_change = self.acceleration_slew_rate * dt
        acceleration_delta = np.clip(requested_acceleration-self.acceleration_command,-max_change,max_change)
        self.acceleration_command += acceleration_delta
        acceleration = self.acceleration_command
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
        return self.mixer.mix(thrust, *torques)
