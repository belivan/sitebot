#!/usr/bin/env python3
"""
ROS2 Bridge Client for MCP Server

Connects to ROS2 via rosbridge WebSocket to:
- Publish velocity commands (/cmd_vel)
- Subscribe to odometry (/odom)
- Call Nav2 actions (/navigate_to_pose)
"""

import json
import logging
import threading
import time
from typing import Dict, Any, Optional, Callable

logger = logging.getLogger("mujoco-mcp.rosbridge")

# Try to import roslibpy, but make it optional
try:
    import roslibpy
    ROSLIBPY_AVAILABLE = True
except ImportError:
    ROSLIBPY_AVAILABLE = False
    logger.warning("roslibpy not installed - ROS2 integration disabled")


class RosbridgeClient:
    """Client for communicating with ROS2 via rosbridge WebSocket."""

    def __init__(self, host: str = 'localhost', port: int = 9090):
        self.host = host
        self.port = port
        self.client: Optional['roslibpy.Ros'] = None
        self.connected = False

        # Topic publishers/subscribers
        self._cmd_vel_pub = None
        self._odom_sub = None

        # Cached state
        self._latest_odom: Optional[Dict[str, Any]] = None
        self._odom_lock = threading.Lock()

        # Continuous velocity publishing
        self._vel_thread: Optional[threading.Thread] = None
        self._vel_lock = threading.Lock()
        self._current_linear_x = 0.0
        self._current_angular_z = 0.0
        self._vel_running = False

    def connect(self) -> bool:
        """Connect to rosbridge WebSocket server."""
        if not ROSLIBPY_AVAILABLE:
            logger.error("roslibpy not available")
            return False

        try:
            self.client = roslibpy.Ros(host=self.host, port=self.port)
            self.client.run()

            # Wait for connection
            timeout = 5.0
            start = time.time()
            while not self.client.is_connected and (time.time() - start) < timeout:
                time.sleep(0.1)

            if self.client.is_connected:
                self.connected = True
                self._setup_topics()
                logger.info(f"Connected to rosbridge at {self.host}:{self.port}")
                return True
            else:
                logger.error("Timeout connecting to rosbridge")
                return False

        except Exception as e:
            logger.error(f"Failed to connect to rosbridge: {e}")
            return False

    def disconnect(self):
        """Disconnect from rosbridge."""
        # Stop velocity publisher first
        self._stop_velocity_publisher()
        if self._odom_sub:
            self._odom_sub.unsubscribe()
        if self._cmd_vel_pub:
            self._cmd_vel_pub.unadvertise()
        if self.client:
            self.client.terminate()
        self.connected = False
        logger.info("Disconnected from rosbridge")

    def _setup_topics(self):
        """Set up ROS2 topic publishers and subscribers."""
        if not self.client or not self.connected:
            return

        # Publisher for velocity commands
        self._cmd_vel_pub = roslibpy.Topic(
            self.client,
            '/diff_drive_controller/cmd_vel_unstamped',
            'geometry_msgs/Twist'
        )
        self._cmd_vel_pub.advertise()

        # Subscriber for odometry
        self._odom_sub = roslibpy.Topic(
            self.client,
            '/diff_drive_controller/odom',
            'nav_msgs/Odometry'
        )
        self._odom_sub.subscribe(self._odom_callback)

    def _odom_callback(self, message: Dict[str, Any]):
        """Handle incoming odometry messages."""
        with self._odom_lock:
            self._latest_odom = message

    def _velocity_publisher_loop(self):
        """Background loop that continuously publishes velocity commands."""
        while self._vel_running and self.connected:
            with self._vel_lock:
                linear_x = self._current_linear_x
                angular_z = self._current_angular_z

            if self._cmd_vel_pub:
                msg = {
                    'linear': {'x': linear_x, 'y': 0.0, 'z': 0.0},
                    'angular': {'x': 0.0, 'y': 0.0, 'z': angular_z}
                }
                try:
                    self._cmd_vel_pub.publish(roslibpy.Message(msg))
                except Exception as e:
                    logger.warning(f"Error publishing cmd_vel: {e}")

            time.sleep(0.1)  # 10 Hz publishing rate

    def _start_velocity_publisher(self):
        """Start the background velocity publisher thread."""
        if self._vel_thread is None or not self._vel_thread.is_alive():
            self._vel_running = True
            self._vel_thread = threading.Thread(
                target=self._velocity_publisher_loop,
                daemon=True
            )
            self._vel_thread.start()

    def _stop_velocity_publisher(self):
        """Stop the background velocity publisher thread."""
        self._vel_running = False
        if self._vel_thread:
            self._vel_thread.join(timeout=1.0)
            self._vel_thread = None

    # -------------------------------------------------------------------------
    # Public API for MCP tools
    # -------------------------------------------------------------------------

    def publish_cmd_vel(self, linear_x: float, angular_z: float) -> Dict[str, Any]:
        """
        Set continuous velocity command.

        Uses a background thread to continuously publish velocity commands
        at 10Hz to prevent cmd_vel timeout (diff_drive_controller times out
        after 0.5s without new commands).

        Args:
            linear_x: Forward velocity (m/s)
            angular_z: Angular velocity (rad/s)

        Returns:
            Success/error dict
        """
        if not self.connected or not self._cmd_vel_pub:
            return {"success": False, "error": "Not connected to ROS2"}

        try:
            # Update target velocity
            with self._vel_lock:
                self._current_linear_x = linear_x
                self._current_angular_z = angular_z

            # Start continuous publisher if not running
            self._start_velocity_publisher()

            return {
                "success": True,
                "message": "Velocity command set (continuous publishing)",
                "cmd_vel": {"linear_x": linear_x, "angular_z": angular_z}
            }
        except Exception as e:
            return {"success": False, "error": str(e)}

    def get_odom(self) -> Dict[str, Any]:
        """
        Get latest odometry data.

        Returns:
            Dict with position, orientation, velocity
        """
        if not self.connected:
            return {"success": False, "error": "Not connected to ROS2"}

        with self._odom_lock:
            if self._latest_odom is None:
                return {"success": False, "error": "No odometry data received yet"}

            odom = self._latest_odom
            pose = odom.get('pose', {}).get('pose', {})
            twist = odom.get('twist', {}).get('twist', {})

            return {
                "success": True,
                "odom": {
                    "position": pose.get('position', {}),
                    "orientation": pose.get('orientation', {}),
                    "linear_velocity": twist.get('linear', {}),
                    "angular_velocity": twist.get('angular', {}),
                },
                "frame_id": odom.get('header', {}).get('frame_id', 'odom'),
                "child_frame_id": odom.get('child_frame_id', 'base_link'),
            }

    def navigate_to_pose(
        self,
        x: float,
        y: float,
        theta: float = 0.0,
        frame_id: str = 'map'
    ) -> Dict[str, Any]:
        """
        Send navigation goal to Nav2.

        Args:
            x: Target X position (meters)
            y: Target Y position (meters)
            theta: Target heading (radians)
            frame_id: Reference frame

        Returns:
            Success/error dict
        """
        if not self.connected or not self.client:
            return {"success": False, "error": "Not connected to ROS2"}

        try:
            # Create action client for Nav2
            action_client = roslibpy.actionlib.ActionClient(
                self.client,
                '/navigate_to_pose',
                'nav2_msgs/NavigateToPose'
            )

            # Build goal message
            import math
            goal = {
                'pose': {
                    'header': {
                        'frame_id': frame_id,
                        'stamp': {'sec': 0, 'nanosec': 0}
                    },
                    'pose': {
                        'position': {'x': x, 'y': y, 'z': 0.0},
                        'orientation': {
                            'x': 0.0,
                            'y': 0.0,
                            'z': math.sin(theta / 2),
                            'w': math.cos(theta / 2)
                        }
                    }
                }
            }

            # Send goal (non-blocking)
            goal_handle = action_client.send_goal(roslibpy.Message(goal))

            return {
                "success": True,
                "message": f"Navigation goal sent to ({x}, {y})",
                "goal": {"x": x, "y": y, "theta": theta, "frame_id": frame_id}
            }

        except Exception as e:
            return {"success": False, "error": str(e)}

    def stop_robot(self) -> Dict[str, Any]:
        """Stop the robot by publishing zero velocity."""
        return self.publish_cmd_vel(0.0, 0.0)

    def get_connection_status(self) -> Dict[str, Any]:
        """Get rosbridge connection status."""
        return {
            "connected": self.connected,
            "host": self.host,
            "port": self.port,
            "roslibpy_available": ROSLIBPY_AVAILABLE,
        }


# Singleton instance
_rosbridge_client: Optional[RosbridgeClient] = None


def get_rosbridge_client(host: str = 'localhost', port: int = 9090) -> RosbridgeClient:
    """Get or create the rosbridge client singleton."""
    global _rosbridge_client
    if _rosbridge_client is None:
        _rosbridge_client = RosbridgeClient(host, port)
    return _rosbridge_client
