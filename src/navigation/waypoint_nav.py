"""
Waypoint Navigation for SiteBot.

Navigates robot to target waypoints using pure pursuit or simple go-to-goal.
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional


class PIDController:
    """Simple PID controller."""

    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.1,
                 max_output: float = float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error: float, dt: float) -> float:
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = np.clip(output, -self.max_output, self.max_output)
        self.prev_error = error
        return output


@dataclass
class Pose2D:
    """2D pose representation."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # heading in radians

    def distance_to(self, other: 'Pose2D') -> float:
        """Euclidean distance to another pose."""
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def angle_to(self, other: 'Pose2D') -> float:
        """Angle from this pose to another (in world frame)."""
        return np.arctan2(other.y - self.y, other.x - self.x)


@dataclass
class NavigationParams:
    """Navigation parameters."""
    goal_tolerance: float = 0.1       # meters
    angle_tolerance: float = 0.1      # radians
    linear_speed: float = 0.5         # m/s
    angular_speed: float = 1.0        # rad/s
    lookahead_distance: float = 0.5   # pure pursuit lookahead


class WaypointNavigator:
    """
    Waypoint-following navigator.

    Uses simple go-to-goal with rotate-then-drive behavior.
    """

    def __init__(self, params: NavigationParams = None):
        self.params = params or NavigationParams()

        # PID controllers
        self.heading_pid = PIDController(kp=2.0, ki=0.0, kd=0.3, max_output=self.params.angular_speed)
        self.distance_pid = PIDController(kp=1.0, ki=0.0, kd=0.1, max_output=self.params.linear_speed)

        self.current_waypoint: Optional[Pose2D] = None
        self.waypoints: list[Pose2D] = []
        self.waypoint_index: int = 0

    def set_waypoints(self, waypoints: list[tuple[float, float]]):
        """Set list of waypoints to navigate."""
        self.waypoints = [Pose2D(x=wp[0], y=wp[1]) for wp in waypoints]
        self.waypoint_index = 0
        if self.waypoints:
            self.current_waypoint = self.waypoints[0]
            self.heading_pid.reset()
            self.distance_pid.reset()

    def compute_velocity(self, current_pose: Pose2D, dt: float) -> tuple[float, float]:
        """
        Compute velocity command to reach current waypoint.

        Args:
            current_pose: Current robot pose
            dt: Time step

        Returns:
            (linear_x, angular_z) velocity command
        """
        if self.current_waypoint is None:
            return 0.0, 0.0

        # Distance and angle to goal
        distance = current_pose.distance_to(self.current_waypoint)
        desired_heading = current_pose.angle_to(self.current_waypoint)

        # Normalize heading error to [-pi, pi]
        heading_error = self._normalize_angle(desired_heading - current_pose.theta)

        # Check if reached waypoint
        if distance < self.params.goal_tolerance:
            # Move to next waypoint
            self.waypoint_index += 1
            if self.waypoint_index < len(self.waypoints):
                self.current_waypoint = self.waypoints[self.waypoint_index]
                self.heading_pid.reset()
                self.distance_pid.reset()
                return 0.0, 0.0
            else:
                # All waypoints reached
                self.current_waypoint = None
                return 0.0, 0.0

        # Rotate-then-drive behavior
        if abs(heading_error) > self.params.angle_tolerance:
            # Rotate in place
            angular_z = self.heading_pid.compute(heading_error, dt)
            return 0.0, angular_z
        else:
            # Drive forward while correcting heading
            linear_x = self.distance_pid.compute(distance, dt)
            angular_z = self.heading_pid.compute(heading_error, dt)
            return linear_x, angular_z

    def is_navigation_complete(self) -> bool:
        """Check if all waypoints have been reached."""
        return self.current_waypoint is None and self.waypoint_index >= len(self.waypoints)

    def get_progress(self) -> tuple[int, int]:
        """Get navigation progress (current_index, total)."""
        return self.waypoint_index, len(self.waypoints)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
