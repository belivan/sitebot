#!/usr/bin/env python3
"""
MuJoCo-ROS2 Bridge Node

Bridges the MuJoCo simulation to ROS2:
- Subscribes to /cmd_vel and sends motor commands to MuJoCo
- Publishes /odom from MuJoCo position/velocity sensors
- Broadcasts tf (odom -> base_link)
- Publishes /joint_states for wheel positions
"""

import json
import math
import socket
import threading
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class MuJoCoClient:
    """Socket client for communicating with MuJoCo viewer server."""

    def __init__(self, host: str = 'localhost', port: int = 8888):
        self.host = host
        self.port = port
        self.socket: Optional[socket.socket] = None
        self.lock = threading.Lock()

    def connect(self) -> bool:
        """Connect to MuJoCo viewer server."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.socket.settimeout(1.0)
            return True
        except Exception as e:
            print(f"Failed to connect to MuJoCo server: {e}")
            return False

    def disconnect(self):
        """Disconnect from server."""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None

    def send_command(self, command: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Send command and receive response."""
        if not self.socket:
            return None

        with self.lock:
            try:
                # Send command
                msg = json.dumps(command) + '\n'
                self.socket.sendall(msg.encode())

                # Receive response
                buffer = b''
                while True:
                    chunk = self.socket.recv(4096)
                    if not chunk:
                        break
                    buffer += chunk
                    if b'\n' in buffer:
                        break

                if buffer:
                    return json.loads(buffer.decode().strip())
                return None

            except socket.timeout:
                return None
            except Exception as e:
                print(f"Command error: {e}")
                return None


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def quaternion_to_yaw(quat: list) -> float:
    """Extract yaw from quaternion [w, x, y, z]."""
    w, x, y, z = quat
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class MuJoCoBridge(Node):
    """ROS2 node bridging MuJoCo simulation."""

    def __init__(self):
        super().__init__('mujoco_bridge')

        # Parameters
        self.declare_parameter('mujoco_host', 'host.docker.internal')  # For Docker
        self.declare_parameter('mujoco_port', 8888)
        self.declare_parameter('model_id', 'sitebot')
        self.declare_parameter('update_rate', 50.0)  # Hz
        self.declare_parameter('wheel_separation', 0.6)  # meters
        self.declare_parameter('wheel_radius', 0.1)  # meters

        self.mujoco_host = self.get_parameter('mujoco_host').value
        self.mujoco_port = self.get_parameter('mujoco_port').value
        self.model_id = self.get_parameter('model_id').value
        self.update_rate = self.get_parameter('update_rate').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # MuJoCo client
        self.mujoco = MuJoCoClient(self.mujoco_host, self.mujoco_port)

        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', qos)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos)

        # State
        self.last_cmd_vel = Twist()
        self.connected = False

        # Connect to MuJoCo
        self.connect_timer = self.create_timer(1.0, self.try_connect)

        self.get_logger().info('MuJoCo Bridge node initialized')

    def try_connect(self):
        """Try to connect to MuJoCo server."""
        if self.connected:
            return

        self.get_logger().info(f'Connecting to MuJoCo at {self.mujoco_host}:{self.mujoco_port}...')

        if self.mujoco.connect():
            self.connected = True
            self.get_logger().info('Connected to MuJoCo server!')

            # Cancel connect timer, start update timer
            self.connect_timer.cancel()
            period = 1.0 / self.update_rate
            self.update_timer = self.create_timer(period, self.update_callback)
        else:
            self.get_logger().warn('Failed to connect, retrying...')

    def cmd_vel_callback(self, msg: Twist):
        """Handle incoming velocity commands."""
        self.last_cmd_vel = msg

        # Convert cmd_vel to wheel velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Differential drive kinematics
        left_vel = (2 * linear_x - angular_z * self.wheel_separation) / (2 * self.wheel_radius)
        right_vel = (2 * linear_x + angular_z * self.wheel_separation) / (2 * self.wheel_radius)

        # Clamp to max wheel velocity (10 rad/s)
        max_vel = 10.0
        left_vel = max(-max_vel, min(max_vel, left_vel))
        right_vel = max(-max_vel, min(max_vel, right_vel))

        # Send to MuJoCo
        if self.connected:
            self.mujoco.send_command({
                'type': 'set_control',
                'model_id': self.model_id,
                'control': [left_vel, right_vel]
            })

    def update_callback(self):
        """Periodic update: read state from MuJoCo, publish to ROS2."""
        if not self.connected:
            return

        # Get sensor data from MuJoCo
        response = self.mujoco.send_command({
            'type': 'get_sensor_data',
            'model_id': self.model_id
        })

        if not response or not response.get('success'):
            return

        sensors = response.get('sensors', {})
        now = self.get_clock().now().to_msg()

        # Extract position and velocity
        pos = sensors.get('chassis_pos', [0, 0, 0])
        quat = sensors.get('chassis_quat', [1, 0, 0, 0])  # w, x, y, z
        lin_vel = sensors.get('chassis_vel', [0, 0, 0])
        ang_vel = sensors.get('chassis_angvel', [0, 0, 0])

        # Get wheel velocities
        left_wheel_vel = sensors.get('left_wheel_vel', [0])[0] if 'left_wheel_vel' in sensors else 0
        right_wheel_vel = sensors.get('right_wheel_vel', [0])[0] if 'right_wheel_vel' in sensors else 0

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position
        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]

        # Orientation (MuJoCo uses w,x,y,z but ROS uses x,y,z,w)
        odom.pose.pose.orientation.w = quat[0]
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]

        # Velocity (in body frame)
        yaw = quaternion_to_yaw(quat)
        # Transform world velocity to body frame
        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)
        odom.twist.twist.linear.x = lin_vel[0] * cos_yaw - lin_vel[1] * sin_yaw
        odom.twist.twist.linear.y = lin_vel[0] * sin_yaw + lin_vel[1] * cos_yaw
        odom.twist.twist.angular.z = ang_vel[2]

        self.odom_pub.publish(odom)

        # Broadcast TF: odom -> base_link
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = pos[0]
        tf.transform.translation.y = pos[1]
        tf.transform.translation.z = pos[2]
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

        # Publish joint states
        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.velocity = [left_wheel_vel, right_wheel_vel]
        # Position would require integrating velocity over time
        joint_state.position = [0.0, 0.0]  # Placeholder
        self.joint_pub.publish(joint_state)

    def destroy_node(self):
        """Clean up on shutdown."""
        self.mujoco.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
