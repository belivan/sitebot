#!/usr/bin/env python3
"""
MuJoCo MCP Server for stdio transport
Production-ready MCP server that works with Claude Desktop and other MCP clients
MCP Protocol Version: 2024-11-05

Based on https://github.com/robotlearning123/mujoco-mcp
Customized for SiteBot project
"""

import asyncio
import json
import logging
import os
import sys
import time
from dataclasses import dataclass
from typing import Dict, Any, List, Optional

from mcp.server import Server, NotificationOptions
from mcp.server.models import InitializationOptions
import mcp.server.stdio
import mcp.types as types

from .version import __version__
from .viewer_client import MuJoCoViewerClient as ViewerClient
from .rosbridge_client import get_rosbridge_client, ROSLIBPY_AVAILABLE

# MCP Protocol constants
MCP_PROTOCOL_VERSION = "2024-11-05"

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("mujoco-mcp")

# Create server instance
server = Server("mujoco-mcp")


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


@dataclass
class _ResourcePayload:
    content: str
    mime_type: str | None = None


viewer_client: ViewerClient | None = None
rosbridge_client = None  # Will be initialized on first ROS2 tool call


def _json_content(payload: Dict[str, Any]) -> List[types.TextContent]:
    """Serialize payload as indented JSON for MCP text responses."""
    return [
        types.TextContent(
            type="text",
            text=json.dumps(payload, indent=2, ensure_ascii=False)
        )
    ]


def _success(message: str, data: Dict[str, Any] | None = None) -> List[types.TextContent]:
    """Create a standard success payload."""
    payload: Dict[str, Any] = {"status": "ok", "message": message}
    if data is not None:
        payload["data"] = data
    return _json_content(payload)


def _error(
    code: str,
    message: str,
    remediation: str | None = None,
    details: Dict[str, Any] | None = None,
) -> List[types.TextContent]:
    """Create a standard error payload following MCP guidance."""
    error_body: Dict[str, Any] = {
        "status": "error",
        "error": {
            "code": code,
            "message": message,
        },
    }

    if remediation:
        error_body["error"]["remediation"] = remediation
    if details:
        error_body["error"]["details"] = details

    return _json_content(error_body)


def _redact_arguments(arguments: Dict[str, Any]) -> Dict[str, Any]:
    """Redact large or sensitive payloads before logging tool calls."""
    return {
        key: "<redacted>" if isinstance(value, str) and len(value) > 256 else value
        for key, value in arguments.items()
    }

@server.list_tools()
async def handle_list_tools() -> List[types.Tool]:
    """Return list of available MuJoCo MCP tools."""

    return [
        types.Tool(
            name="get_server_info",
            description="Get information about the MuJoCo MCP server",
            inputSchema={
                "$schema": "http://json-schema.org/draft-07/schema#",
                "type": "object",
                "properties": {},
                "required": [],
                "additionalProperties": False,
                "examples": [{}],
            },
        ),
        types.Tool(
            name="create_scene",
            description="Create a physics simulation scene",
            inputSchema={
                "$schema": "http://json-schema.org/draft-07/schema#",
                "type": "object",
                "properties": {
                    "scene_type": {
                        "type": "string",
                        "description": "Type of scene to create",
                        "enum": ["pendulum", "double_pendulum", "cart_pole", "arm"],
                    }
                },
                "required": ["scene_type"],
                "additionalProperties": False,
                "examples": [
                    {"scene_type": "pendulum"},
                    {"scene_type": "double_pendulum"},
                ],
            },
        ),
        types.Tool(
            name="step_simulation",
            description="Step the physics simulation forward",
            inputSchema={
                "$schema": "http://json-schema.org/draft-07/schema#",
                "type": "object",
                "properties": {
                    "model_id": {
                        "type": "string",
                        "description": "ID of the model to step",
                    },
                    "steps": {
                        "type": "integer",
                        "description": "Number of simulation steps",
                        "default": 1,
                        "minimum": 1,
                    },
                },
                "required": ["model_id"],
                "additionalProperties": False,
                "examples": [
                    {"model_id": "pendulum", "steps": 5},
                ],
            },
        ),
        types.Tool(
            name="get_state",
            description="Get current state of the simulation",
            inputSchema={
                "$schema": "http://json-schema.org/draft-07/schema#",
                "type": "object",
                "properties": {
                    "model_id": {
                        "type": "string",
                        "description": "ID of the model to get state from",
                    }
                },
                "required": ["model_id"],
                "additionalProperties": False,
                "examples": [{"model_id": "pendulum"}],
            },
        ),
        types.Tool(
            name="reset_simulation",
            description="Reset simulation to initial state",
            inputSchema={
                "$schema": "http://json-schema.org/draft-07/schema#",
                "type": "object",
                "properties": {
                    "model_id": {
                        "type": "string",
                        "description": "ID of the model to reset",
                    }
                },
                "required": ["model_id"],
                "additionalProperties": False,
                "examples": [{"model_id": "pendulum"}],
            },
        ),
        types.Tool(
            name="close_viewer",
            description="Close the MuJoCo viewer window",
            inputSchema={
                "$schema": "http://json-schema.org/draft-07/schema#",
                "type": "object",
                "properties": {
                    "model_id": {
                        "type": "string",
                        "description": "ID of the model viewer to close",
                    }
                },
                "required": ["model_id"],
                "additionalProperties": False,
                "examples": [{"model_id": "pendulum"}],
            },
        ),
        types.Tool(
            name="load_model",
            description="Load a custom MuJoCo MJCF model from a file path",
            inputSchema={
                "$schema": "http://json-schema.org/draft-07/schema#",
                "type": "object",
                "properties": {
                    "file_path": {
                        "type": "string",
                        "description": "Absolute path to the MJCF XML file",
                    },
                    "model_id": {
                        "type": "string",
                        "description": "Optional ID for the model (defaults to filename)",
                    },
                },
                "required": ["file_path"],
                "additionalProperties": False,
                "examples": [{"file_path": "/path/to/robot.xml", "model_id": "my_robot"}],
            },
        ),
        types.Tool(
            name="set_control",
            description="Apply control inputs (motor torques/velocities) to actuators",
            inputSchema={
                "$schema": "http://json-schema.org/draft-07/schema#",
                "type": "object",
                "properties": {
                    "model_id": {
                        "type": "string",
                        "description": "ID of the model",
                    },
                    "control": {
                        "type": "array",
                        "items": {"type": "number"},
                        "description": "Control values for each actuator",
                    },
                },
                "required": ["model_id", "control"],
                "additionalProperties": False,
                "examples": [{"model_id": "my_robot", "control": [1.0, -1.0]}],
            },
        ),
        types.Tool(
            name="set_joint_positions",
            description="Set joint positions directly (teleport)",
            inputSchema={
                "$schema": "http://json-schema.org/draft-07/schema#",
                "type": "object",
                "properties": {
                    "model_id": {
                        "type": "string",
                        "description": "ID of the model",
                    },
                    "positions": {
                        "type": "array",
                        "items": {"type": "number"},
                        "description": "Joint position values",
                    },
                },
                "required": ["model_id", "positions"],
                "additionalProperties": False,
                "examples": [{"model_id": "my_robot", "positions": [0.0, 0.5, 1.0]}],
            },
        ),
        types.Tool(
            name="get_sensor_data",
            description="Read all sensor values from the simulation",
            inputSchema={
                "$schema": "http://json-schema.org/draft-07/schema#",
                "type": "object",
                "properties": {
                    "model_id": {
                        "type": "string",
                        "description": "ID of the model",
                    },
                },
                "required": ["model_id"],
                "additionalProperties": False,
                "examples": [{"model_id": "my_robot"}],
            },
        ),
        types.Tool(
            name="capture_frame",
            description="Capture rendered image from simulation as base64 PNG",
            inputSchema={
                "$schema": "http://json-schema.org/draft-07/schema#",
                "type": "object",
                "properties": {
                    "model_id": {
                        "type": "string",
                        "description": "ID of the model",
                    },
                    "width": {
                        "type": "integer",
                        "description": "Image width in pixels",
                        "default": 640,
                    },
                    "height": {
                        "type": "integer",
                        "description": "Image height in pixels",
                        "default": 480,
                    },
                },
                "required": ["model_id"],
                "additionalProperties": False,
                "examples": [{"model_id": "my_robot", "width": 800, "height": 600}],
            },
        ),
        # ROS2 tools via rosbridge
        types.Tool(
            name="ros2_connect",
            description="Connect to ROS2 via rosbridge WebSocket. Required before using other ros2_* tools.",
            inputSchema={
                "$schema": "http://json-schema.org/draft-07/schema#",
                "type": "object",
                "properties": {
                    "host": {
                        "type": "string",
                        "description": "rosbridge WebSocket host",
                        "default": "localhost",
                    },
                    "port": {
                        "type": "integer",
                        "description": "rosbridge WebSocket port",
                        "default": 9090,
                    },
                },
                "required": [],
                "additionalProperties": False,
                "examples": [{"host": "localhost", "port": 9090}],
            },
        ),
        types.Tool(
            name="ros2_status",
            description="Get ROS2 rosbridge connection status",
            inputSchema={
                "$schema": "http://json-schema.org/draft-07/schema#",
                "type": "object",
                "properties": {},
                "required": [],
                "additionalProperties": False,
                "examples": [{}],
            },
        ),
        types.Tool(
            name="ros2_drive",
            description="Send velocity command to the robot via ROS2 /cmd_vel topic",
            inputSchema={
                "$schema": "http://json-schema.org/draft-07/schema#",
                "type": "object",
                "properties": {
                    "linear_x": {
                        "type": "number",
                        "description": "Forward velocity in m/s (positive=forward, negative=backward)",
                    },
                    "angular_z": {
                        "type": "number",
                        "description": "Angular velocity in rad/s (positive=left, negative=right)",
                    },
                },
                "required": ["linear_x", "angular_z"],
                "additionalProperties": False,
                "examples": [
                    {"linear_x": 0.5, "angular_z": 0.0},
                    {"linear_x": 0.0, "angular_z": 0.5},
                ],
            },
        ),
        types.Tool(
            name="ros2_stop",
            description="Stop the robot by sending zero velocity",
            inputSchema={
                "$schema": "http://json-schema.org/draft-07/schema#",
                "type": "object",
                "properties": {},
                "required": [],
                "additionalProperties": False,
                "examples": [{}],
            },
        ),
        types.Tool(
            name="ros2_get_pose",
            description="Get robot pose from ROS2 odometry",
            inputSchema={
                "$schema": "http://json-schema.org/draft-07/schema#",
                "type": "object",
                "properties": {},
                "required": [],
                "additionalProperties": False,
                "examples": [{}],
            },
        ),
        types.Tool(
            name="ros2_navigate_to",
            description="Send navigation goal to Nav2 (requires Nav2 stack running)",
            inputSchema={
                "$schema": "http://json-schema.org/draft-07/schema#",
                "type": "object",
                "properties": {
                    "x": {
                        "type": "number",
                        "description": "Target X position in meters",
                    },
                    "y": {
                        "type": "number",
                        "description": "Target Y position in meters",
                    },
                    "theta": {
                        "type": "number",
                        "description": "Target heading in radians",
                        "default": 0.0,
                    },
                    "frame_id": {
                        "type": "string",
                        "description": "Reference frame",
                        "default": "map",
                    },
                },
                "required": ["x", "y"],
                "additionalProperties": False,
                "examples": [
                    {"x": 5.0, "y": 3.0},
                    {"x": 10.0, "y": -2.0, "theta": 1.57},
                ],
            },
        ),
    ]


def _state_snapshot() -> Dict[str, Any]:
    """Return a lightweight snapshot of the first active simulation."""

    if not viewer_client or not viewer_client.connected:
        return {
            "status": "ok",
            "data": {
                "active": False,
                "message": "Viewer connection inactive",
            },
        }

    try:
        response = viewer_client.send_command({"type": "get_state"})
        if not response.get("success"):
            return {
                "status": "error",
                "error": {
                    "code": "viewer_error",
                    "message": response.get("error", "Unable to fetch state."),
                },
            }

        state = response.get("state")
        if state is None:
            state_keys = ["time", "qpos", "qvel", "ctrl", "xpos"]
            state = {key: response[key] for key in state_keys if key in response}

        return {
            "status": "ok",
            "data": {
                "active": True,
                "state": state,
            },
        }
    except Exception as exc:  # pragma: no cover - defensive path
        logger.exception("Failed to build state snapshot")
        return {
            "status": "error",
            "error": {
                "code": "internal_error",
                "message": "Failed to gather simulation state.",
                "details": {"exception": str(exc)},
            },
        }


@server.list_resources()
async def handle_list_resources() -> List[types.Resource]:
    """Advertise available resources following MCP guidelines."""

    return [
        types.Resource(
            name="simulation_state",
            title="Simulation State Snapshot",
            uri="simulation://state",
            description="Latest state snapshot of the first active simulation (if any).",
            mimeType="application/json",
        ),
        types.Resource(
            name="server_config",
            title="Server Configuration",
            uri="simulation://config",
            description="Server capabilities, version, and protocol metadata.",
            mimeType="application/json",
        ),
    ]


@server.read_resource()
async def handle_read_resource(uri: str):
    """Provide resource contents for the advertised URIs."""

    if uri == "simulation://state":
        payload = _state_snapshot()
    elif uri == "simulation://config":
        tools = await handle_list_tools()
        payload = {
            "status": "ok",
            "data": {
                "version": __version__,
                "protocol_version": MCP_PROTOCOL_VERSION,
                "tools": [tool.name for tool in tools],
            },
        }
    else:
        payload = {
            "status": "error",
            "error": {
                "code": "unknown_resource",
                "message": f"Resource '{uri}' is not available.",
            },
        }

    return [_ResourcePayload(content=json.dumps(payload), mime_type="application/json")]

async def _handle_ros2_tool(name: str, arguments: Dict[str, Any]) -> List[types.TextContent]:
    """Handle ROS2 tools via rosbridge."""
    global rosbridge_client

    # Check if roslibpy is available
    if not ROSLIBPY_AVAILABLE:
        return _error(
            code="ros2_unavailable",
            message="ROS2 integration is not available. roslibpy package not installed.",
            remediation="Install roslibpy: pip install roslibpy",
        )

    if name == "ros2_connect":
        host = arguments.get("host", "localhost")
        port = arguments.get("port", 9090)

        rosbridge_client = get_rosbridge_client(host, port)
        if rosbridge_client.connected:
            return _success("Already connected to rosbridge", {
                "host": host,
                "port": port,
                "connected": True,
            })

        if rosbridge_client.connect():
            return _success("Connected to rosbridge", {
                "host": host,
                "port": port,
                "connected": True,
            })
        else:
            return _error(
                code="ros2_connection_failed",
                message=f"Failed to connect to rosbridge at {host}:{port}",
                remediation="Ensure rosbridge_websocket is running in the Docker container.",
            )

    if name == "ros2_status":
        if not rosbridge_client:
            return _success("ROS2 status", {
                "connected": False,
                "message": "rosbridge client not initialized. Call ros2_connect first.",
                "roslibpy_available": ROSLIBPY_AVAILABLE,
            })
        return _success("ROS2 status", rosbridge_client.get_connection_status())

    # All other ros2_* tools require connection
    if not rosbridge_client or not rosbridge_client.connected:
        return _error(
            code="ros2_not_connected",
            message="Not connected to ROS2. Call ros2_connect first.",
            remediation="Use ros2_connect tool to establish connection to rosbridge.",
        )

    if name == "ros2_drive":
        linear_x = arguments.get("linear_x", 0.0)
        angular_z = arguments.get("angular_z", 0.0)
        result = rosbridge_client.publish_cmd_vel(linear_x, angular_z)
        if result.get("success"):
            return _success("Velocity command sent", result)
        else:
            return _error(
                code="ros2_cmd_vel_failed",
                message=result.get("error", "Failed to publish cmd_vel"),
            )

    if name == "ros2_stop":
        result = rosbridge_client.stop_robot()
        if result.get("success"):
            return _success("Robot stopped", result)
        else:
            return _error(
                code="ros2_stop_failed",
                message=result.get("error", "Failed to stop robot"),
            )

    if name == "ros2_get_pose":
        result = rosbridge_client.get_odom()
        if result.get("success"):
            return _success("Robot pose from odometry", result)
        else:
            return _error(
                code="ros2_odom_failed",
                message=result.get("error", "Failed to get odometry"),
                remediation="Ensure diff_drive_controller is publishing odometry.",
            )

    if name == "ros2_navigate_to":
        x = arguments.get("x")
        y = arguments.get("y")
        theta = arguments.get("theta", 0.0)
        frame_id = arguments.get("frame_id", "map")

        if x is None or y is None:
            return _error(
                code="missing_argument",
                message="Both 'x' and 'y' arguments are required.",
            )

        result = rosbridge_client.navigate_to_pose(x, y, theta, frame_id)
        if result.get("success"):
            return _success("Navigation goal sent", result)
        else:
            return _error(
                code="ros2_nav_failed",
                message=result.get("error", "Failed to send navigation goal"),
                remediation="Ensure Nav2 stack is running.",
            )

    return _error(
        code="unknown_ros2_tool",
        message=f"Unknown ROS2 tool: {name}",
    )


@server.call_tool()
async def handle_call_tool(name: str, arguments: Dict[str, Any]) -> List[types.TextContent]:
    """Handle tool calls with MCP-compliant responses."""

    global viewer_client
    start = time.perf_counter()
    redacted_args = _redact_arguments(arguments)

    try:
        if name == "get_server_info":
            logger.info("get_server_info invoked")
            return _success(
                "Server information",
                {
                    "name": "MuJoCo MCP Server",
                    "version": __version__,
                    "description": "Control MuJoCo physics simulations through MCP",
                    "protocol_version": MCP_PROTOCOL_VERSION,
                    "capabilities": [
                        "create_scene",
                        "step_simulation",
                        "get_state",
                        "reset_simulation",
                        "close_viewer",
                        "ros2_connect",
                        "ros2_drive",
                        "ros2_get_pose",
                        "ros2_navigate_to",
                        "ros2_stop",
                        "ros2_status",
                    ],
                    "ros2_available": ROSLIBPY_AVAILABLE,
                },
            )

        # Handle ROS2 tools
        if name.startswith("ros2_"):
            return await _handle_ros2_tool(name, arguments)

        if name not in {
            "create_scene", "step_simulation", "get_state", "reset_simulation", "close_viewer",
            "load_model", "set_control", "set_joint_positions", "get_sensor_data", "capture_frame"
        }:
            logger.warning("Unknown tool requested", extra={"tool": name})
            return _error(
                code="unknown_tool",
                message=f"Tool '{name}' is not available.",
                remediation="Call list_tools to discover supported tools.",
            )

        if not viewer_client:
            viewer_client = ViewerClient()

        if not viewer_client.connected and not viewer_client.connect():
            return _error(
                code="viewer_unavailable",
                message="Failed to connect to the MuJoCo viewer server.",
                remediation="Start 'mujoco-mcp-viewer' and retry the tool call.",
            )

        if name == "create_scene":
            scene_type = arguments.get("scene_type", "pendulum")
            scene_models = {
                "pendulum": """
                <mujoco>
                    <worldbody>
                        <body name="pole" pos="0 0 1">
                            <joint name="hinge" type="hinge" axis="1 0 0"/>
                            <geom name="pole" type="capsule" size="0.02 0.6" rgba="0.8 0.2 0.2 1"/>
                            <body name="mass" pos="0 0 -0.6">
                                <geom name="mass" type="sphere" size="0.05" rgba="0.2 0.8 0.2 1"/>
                            </body>
                        </body>
                    </worldbody>
                </mujoco>
                """,
                "double_pendulum": """
                <mujoco>
                    <worldbody>
                        <body name="pole1" pos="0 0 1">
                            <joint name="hinge1" type="hinge" axis="1 0 0"/>
                            <geom name="pole1" type="capsule" size="0.02 0.4" rgba="0.8 0.2 0.2 1"/>
                            <body name="pole2" pos="0 0 -0.4">
                                <joint name="hinge2" type="hinge" axis="1 0 0"/>
                                <geom name="pole2" type="capsule" size="0.02 0.4" rgba="0.2 0.8 0.2 1"/>
                                <body name="mass" pos="0 0 -0.4">
                                    <geom name="mass" type="sphere" size="0.05" rgba="0.2 0.2 0.8 1"/>
                                </body>
                            </body>
                        </body>
                    </worldbody>
                </mujoco>
                """,
                "cart_pole": """
                <mujoco>
                    <worldbody>
                        <body name="cart" pos="0 0 0.1">
                            <joint name="slider" type="slide" axis="1 0 0"/>
                            <geom name="cart" type="box" size="0.1 0.1 0.1" rgba="0.8 0.2 0.2 1"/>
                            <body name="pole" pos="0 0 0.1">
                                <joint name="hinge" type="hinge" axis="0 1 0"/>
                                <geom name="pole" type="capsule" size="0.02 0.5" rgba="0.2 0.8 0.2 1"/>
                            </body>
                        </body>
                    </worldbody>
                </mujoco>
                """,
                "arm": """
                <mujoco>
                    <worldbody>
                        <body name="base">
                            <joint name="hinge" type="hinge" axis="0 0 1"/>
                            <geom name="link" type="capsule" size="0.02 0.4" rgba="0.8 0.2 0.2 1"/>
                        </body>
                    </worldbody>
                </mujoco>
                """,
            }

            if scene_type not in scene_models:
                return _error(
                    code="invalid_scene",
                    message=f"Scene type '{scene_type}' is not supported.",
                    remediation=f"Use one of: {', '.join(scene_models)}.",
                )

            response = viewer_client.send_command(
                {
                    "type": "load_model",
                    "model_id": scene_type,
                    "model_xml": scene_models[scene_type],
                }
            )

            if not response.get("success"):
                return _error(
                    code="viewer_error",
                    message=response.get("error", "Unknown viewer error"),
                )

            return _success(
                "Scene created",
                {
                    "model_id": scene_type,
                    "viewer_response": response,
                },
            )

        model_id = arguments.get("model_id")
        if not model_id:
            return _error(
                code="missing_argument",
                message="The 'model_id' argument is required.",
                remediation="Pass the target model identifier in the tool arguments.",
            )

        if name == "step_simulation":
            steps = max(1, int(arguments.get("steps", 1)))
            # Simulation runs continuously; acknowledge the request.
            return _success(
                "Simulation step acknowledged",
                {"model_id": model_id, "steps": steps},
            )

        if name == "get_state":
            response = viewer_client.send_command({"type": "get_state", "model_id": model_id})
            if not response.get("success"):
                return _error(
                    code="viewer_error",
                    message=response.get("error", "Failed to retrieve state."),
                )

            state = response.get("state")
            if state is None:
                state_keys = ["time", "qpos", "qvel", "qacc", "ctrl", "xpos"]
                state = {key: response[key] for key in state_keys if key in response}

            return _success("Simulation state", {"model_id": model_id, "state": state})

        if name == "reset_simulation":
            response = viewer_client.send_command({"type": "reset", "model_id": model_id})
            if not response.get("success"):
                return _error(
                    code="viewer_error",
                    message=response.get("error", "Reset failed."),
                )
            return _success("Simulation reset", {"model_id": model_id})

        if name == "close_viewer":
            response = viewer_client.send_command({"type": "close_model", "model_id": model_id})
            if viewer_client:
                viewer_client.disconnect()
                viewer_client = None

            if not response.get("success"):
                return _error(
                    code="viewer_error",
                    message=response.get("error", "Failed to close viewer."),
                )

            return _success("Viewer closed", {"model_id": model_id})

        if name == "load_model":
            file_path = arguments.get("file_path")
            model_id = arguments.get("model_id") or os.path.basename(file_path).replace(".xml", "")

            if not os.path.exists(file_path):
                return _error(
                    code="file_not_found",
                    message=f"File not found: {file_path}",
                    remediation="Provide an absolute path to an existing MJCF XML file.",
                )

            response = viewer_client.send_command({
                "type": "load_model",
                "model_id": model_id,
                "model_xml": file_path,
            })

            if not response.get("success"):
                return _error(
                    code="viewer_error",
                    message=response.get("error", "Failed to load model."),
                )

            return _success("Model loaded", {
                "model_id": model_id,
                "file_path": file_path,
                "model_info": response.get("model_info", {}),
            })

        if name == "set_control":
            model_id = arguments.get("model_id")
            control = arguments.get("control", [])

            response = viewer_client.send_command({
                "type": "set_control",
                "model_id": model_id,
                "control": control,
            })

            if not response.get("success"):
                return _error(
                    code="viewer_error",
                    message=response.get("error", "Failed to set control."),
                )

            return _success("Control applied", {"model_id": model_id, "control": control})

        if name == "set_joint_positions":
            model_id = arguments.get("model_id")
            positions = arguments.get("positions", [])

            response = viewer_client.send_command({
                "type": "set_joint_positions",
                "model_id": model_id,
                "positions": positions,
            })

            if not response.get("success"):
                return _error(
                    code="viewer_error",
                    message=response.get("error", "Failed to set joint positions."),
                )

            return _success("Joint positions set", {"model_id": model_id, "positions": positions})

        if name == "get_sensor_data":
            model_id = arguments.get("model_id")

            response = viewer_client.send_command({
                "type": "get_sensor_data",
                "model_id": model_id,
            })

            if not response.get("success"):
                return _error(
                    code="viewer_error",
                    message=response.get("error", "Failed to get sensor data."),
                )

            return _success("Sensor data", {"model_id": model_id, "sensors": response.get("sensors", {})})

        if name == "capture_frame":
            model_id = arguments.get("model_id")
            width = arguments.get("width", 640)
            height = arguments.get("height", 480)

            response = viewer_client.send_command({
                "type": "capture_render",
                "model_id": model_id,
                "width": width,
                "height": height,
            })

            if not response.get("success"):
                return _error(
                    code="viewer_error",
                    message=response.get("error", "Failed to capture frame."),
                )

            return _success("Frame captured", {
                "model_id": model_id,
                "image_data": response.get("image_data"),
                "format": "png",
                "encoding": "base64",
                "width": width,
                "height": height,
            })

        return _error(
            code="unknown_tool",
            message=f"Tool '{name}' is not available.",
        )

    except Exception as exc:
        logger.exception("Error in tool handler", extra={"tool": name, "arguments": redacted_args})
        return _error(
            code="internal_error",
            message="Unexpected server error.",
            details={"exception": str(exc)},
        )
    finally:
        duration_ms = (time.perf_counter() - start) * 1000
        logger.info(
            "Tool handled",
            extra={"tool": name, "duration_ms": round(duration_ms, 2), "arguments": redacted_args},
        )

async def main():
    """Main entry point for MCP server"""
    logger.info(f"Starting MuJoCo MCP Server v{__version__}")
    logger.info(f"MCP Protocol Version: {MCP_PROTOCOL_VERSION}")

    # Initialize server capabilities with enhanced configuration
    capabilities = server.get_capabilities(
        notification_options=NotificationOptions(),
        experimental_capabilities={}
    )

    server_options = InitializationOptions(
        server_name="mujoco-mcp",
        server_version=__version__,
        capabilities=capabilities,
        protocol_versions=[MCP_PROTOCOL_VERSION],
        instructions="MuJoCo physics simulation server with viewer support. "
                    f"Implements MCP Protocol {MCP_PROTOCOL_VERSION}. "
                    "Provides tools for creating scenes, controlling simulation, and managing state."
    )

    logger.info(f"Server capabilities: {capabilities}")
    logger.info("MCP server initialization complete")

    # Run server with stdio transport
    try:
        async with mcp.server.stdio.stdio_server() as (read_stream, write_stream):
            logger.info("Starting MCP server stdio transport")
            await server.run(
                read_stream,
                write_stream,
                server_options
            )
    except Exception as exc:
        logger.exception("MCP server error", extra={"exception": str(exc)})
        raise

if __name__ == "__main__":
    asyncio.run(main())
