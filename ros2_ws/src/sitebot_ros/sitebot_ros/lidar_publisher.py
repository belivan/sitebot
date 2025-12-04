#!/usr/bin/env python3
"""
Simulated Lidar Publisher Node

Computes 2D laser scan ranges based on robot position (from odom) and
known obstacles in the construction site world. Publishes to /scan for
Nav2/SLAM integration.
"""

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# Obstacle definitions from construction_site.xml
# Format: (x, y, type, dimensions)
# Box: (cx, cy, 'box', (half_x, half_y))
# Cylinder: (cx, cy, 'cylinder', radius)
OBSTACLES = [
    # Pallets (boxes)
    (10.0, 8.0, 'box', (0.6, 0.4)),
    (-8.0, -12.0, 'box', (0.5, 0.5)),
    # Concrete blocks
    (-12.0, 3.0, 'box', (0.3, 0.3)),
    (-12.0, 5.0, 'box', (0.3, 0.3)),
    # Pipes (cylinder, treat as circle in 2D)
    (12.0, -5.0, 'cylinder', 0.1),
    # Boundary posts (cylinders)
    (-20.0, 20.0, 'cylinder', 0.1),
    (20.0, 20.0, 'cylinder', 0.1),
    (-20.0, -20.0, 'cylinder', 0.1),
    (20.0, -20.0, 'cylinder', 0.1),
]

# World boundary (rectangular)
WORLD_BOUNDS = (-25.0, -25.0, 25.0, 25.0)  # (min_x, min_y, max_x, max_y)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw angle from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def ray_box_intersection(
    ox: float, oy: float,  # Ray origin
    dx: float, dy: float,  # Ray direction (normalized)
    bx: float, by: float,  # Box center
    hx: float, hy: float   # Box half-extents
) -> float:
    """
    Compute intersection distance of ray with axis-aligned box.
    Returns distance or inf if no intersection.
    """
    # Box bounds
    min_x, max_x = bx - hx, bx + hx
    min_y, max_y = by - hy, by + hy

    # Handle near-zero direction components
    eps = 1e-10

    if abs(dx) < eps:
        if ox < min_x or ox > max_x:
            return float('inf')
        t_min_x, t_max_x = float('-inf'), float('inf')
    else:
        t1 = (min_x - ox) / dx
        t2 = (max_x - ox) / dx
        t_min_x, t_max_x = min(t1, t2), max(t1, t2)

    if abs(dy) < eps:
        if oy < min_y or oy > max_y:
            return float('inf')
        t_min_y, t_max_y = float('-inf'), float('inf')
    else:
        t1 = (min_y - oy) / dy
        t2 = (max_y - oy) / dy
        t_min_y, t_max_y = min(t1, t2), max(t1, t2)

    t_enter = max(t_min_x, t_min_y)
    t_exit = min(t_max_x, t_max_y)

    if t_enter > t_exit or t_exit < 0:
        return float('inf')

    return t_enter if t_enter > 0 else t_exit


def ray_circle_intersection(
    ox: float, oy: float,  # Ray origin
    dx: float, dy: float,  # Ray direction (normalized)
    cx: float, cy: float,  # Circle center
    r: float              # Circle radius
) -> float:
    """
    Compute intersection distance of ray with circle.
    Returns distance or inf if no intersection.
    """
    # Vector from circle center to ray origin
    fx = ox - cx
    fy = oy - cy

    a = dx * dx + dy * dy
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - r * r

    discriminant = b * b - 4.0 * a * c

    if discriminant < 0:
        return float('inf')

    sqrt_disc = math.sqrt(discriminant)
    t1 = (-b - sqrt_disc) / (2.0 * a)
    t2 = (-b + sqrt_disc) / (2.0 * a)

    if t1 > 0:
        return t1
    elif t2 > 0:
        return t2
    else:
        return float('inf')


def ray_bounds_intersection(
    ox: float, oy: float,
    dx: float, dy: float,
    bounds: Tuple[float, float, float, float]
) -> float:
    """Intersection with world boundary box (from inside)."""
    min_x, min_y, max_x, max_y = bounds
    return ray_box_intersection(
        ox, oy, dx, dy,
        (min_x + max_x) / 2, (min_y + max_y) / 2,
        (max_x - min_x) / 2, (max_y - min_y) / 2
    )


class SimulatedLidarNode(Node):
    """Publishes simulated laser scan based on robot position and known obstacles."""

    def __init__(self):
        super().__init__('simulated_lidar')

        # Parameters
        self.declare_parameter('num_beams', 360)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 30.0)
        self.declare_parameter('scan_rate', 10.0)  # Hz
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('odom_topic', '/odom')  # Gazebo uses /odom

        self.num_beams = self.get_parameter('num_beams').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.scan_rate = self.get_parameter('scan_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        odom_topic = self.get_parameter('odom_topic').value

        # Current robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos
        )

        # Publisher for laser scan
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos)

        # Timer for publishing scans
        self.scan_timer = self.create_timer(1.0 / self.scan_rate, self.publish_scan)

        self.get_logger().info(
            f'Simulated Lidar started: {self.num_beams} beams, '
            f'range [{self.range_min:.1f}, {self.range_max:.1f}]m'
        )

    def odom_callback(self, msg: Odometry):
        """Update robot pose from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.robot_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def compute_range(self, angle: float) -> float:
        """Compute range for a single beam at given angle (world frame)."""
        # Ray direction
        dx = math.cos(angle)
        dy = math.sin(angle)

        min_range = self.range_max

        # Check each obstacle
        for obs in OBSTACLES:
            ox, oy, obs_type, dims = obs

            if obs_type == 'box':
                hx, hy = dims
                dist = ray_box_intersection(
                    self.robot_x, self.robot_y, dx, dy, ox, oy, hx, hy
                )
            elif obs_type == 'cylinder':
                radius = dims
                dist = ray_circle_intersection(
                    self.robot_x, self.robot_y, dx, dy, ox, oy, radius
                )
            else:
                continue

            if dist < min_range and dist > self.range_min:
                min_range = dist

        # Check world boundaries
        bounds_dist = ray_bounds_intersection(
            self.robot_x, self.robot_y, dx, dy, WORLD_BOUNDS
        )
        if bounds_dist < min_range and bounds_dist > self.range_min:
            min_range = bounds_dist

        return min_range

    def publish_scan(self):
        """Compute and publish laser scan."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = (self.angle_max - self.angle_min) / self.num_beams
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.scan_rate
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        ranges: List[float] = []

        for i in range(self.num_beams):
            # Beam angle in robot frame
            beam_angle_local = self.angle_min + i * scan.angle_increment
            # Transform to world frame
            beam_angle_world = self.robot_yaw + beam_angle_local

            r = self.compute_range(beam_angle_world)
            ranges.append(r)

        scan.ranges = ranges
        scan.intensities = []  # Not simulated

        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedLidarNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
