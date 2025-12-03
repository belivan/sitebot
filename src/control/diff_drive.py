"""
Differential Drive Controller for SiteBot.

Converts velocity commands (linear, angular) to wheel velocities.
Similar to ROS 2 diff_drive_controller.
"""

import numpy as np
from dataclasses import dataclass


@dataclass
class DiffDriveParams:
    """Differential drive robot parameters."""
    wheel_radius: float = 0.1      # meters
    wheel_separation: float = 0.6  # meters (distance between wheels)
    max_wheel_vel: float = 10.0    # rad/s
    max_linear_vel: float = 1.0    # m/s
    max_angular_vel: float = 2.0   # rad/s


class DiffDriveController:
    """
    Differential drive controller.

    Converts cmd_vel (linear_x, angular_z) to wheel velocities.
    """

    def __init__(self, params: DiffDriveParams = None):
        self.params = params or DiffDriveParams()

    def cmd_vel_to_wheel_vel(self, linear_x: float, angular_z: float) -> tuple[float, float]:
        """
        Convert velocity command to wheel velocities.

        Args:
            linear_x: Forward velocity (m/s)
            angular_z: Angular velocity (rad/s), positive = CCW

        Returns:
            (left_wheel_vel, right_wheel_vel) in rad/s
        """
        # Clamp input velocities
        linear_x = np.clip(linear_x, -self.params.max_linear_vel, self.params.max_linear_vel)
        angular_z = np.clip(angular_z, -self.params.max_angular_vel, self.params.max_angular_vel)

        # Differential drive kinematics
        # v_left = (2*v - w*L) / (2*r)
        # v_right = (2*v + w*L) / (2*r)
        r = self.params.wheel_radius
        L = self.params.wheel_separation

        left_vel = (2 * linear_x - angular_z * L) / (2 * r)
        right_vel = (2 * linear_x + angular_z * L) / (2 * r)

        # Clamp wheel velocities
        left_vel = np.clip(left_vel, -self.params.max_wheel_vel, self.params.max_wheel_vel)
        right_vel = np.clip(right_vel, -self.params.max_wheel_vel, self.params.max_wheel_vel)

        return left_vel, right_vel

    def wheel_vel_to_cmd_vel(self, left_vel: float, right_vel: float) -> tuple[float, float]:
        """
        Convert wheel velocities to cmd_vel (for odometry).

        Args:
            left_vel: Left wheel velocity (rad/s)
            right_vel: Right wheel velocity (rad/s)

        Returns:
            (linear_x, angular_z)
        """
        r = self.params.wheel_radius
        L = self.params.wheel_separation

        linear_x = r * (left_vel + right_vel) / 2
        angular_z = r * (right_vel - left_vel) / L

        return linear_x, angular_z


class PIDController:
    """Simple PID controller for velocity/position control."""

    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.1,
                 max_output: float = float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output

        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error: float, dt: float) -> float:
        """
        Compute control output.

        Args:
            error: Current error (setpoint - measured)
            dt: Time step

        Returns:
            Control output
        """
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = np.clip(output, -self.max_output, self.max_output)

        self.prev_error = error
        return output
