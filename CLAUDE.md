# CLAUDE.md - Claude Code Context

## Project Overview

SiteBot is a construction robotics simulation platform combining MuJoCo physics simulation with ROS2/Nav2 for autonomous navigation. It provides MCP (Model Context Protocol) integration for AI-assisted robot control.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Docker Container (ROS2 Humble)               │
│                                                                 │
│  ┌──────────────┐    ┌───────────────────┐    ┌─────────────┐  │
│  │   MuJoCo     │───▶│ mujoco_ros2_control│───▶│   Nav2      │  │
│  │  (physics)   │    │   (ros2_control)   │    │   Stack     │  │
│  └──────────────┘    └───────────────────┘    └─────────────┘  │
│                              │                                  │
│                    ┌─────────┴─────────┐                       │
│                    │ rosbridge_websocket│ (port 9090)          │
│                    └─────────┬─────────┘                       │
└──────────────────────────────┼──────────────────────────────────┘
                               │ WebSocket
┌──────────────────────────────┼──────────────────────────────────┐
│                         Mac Host                                 │
│                    ┌─────────┴─────────┐                        │
│                    │    MCP Server     │                        │
│                    │ (rosbridge client)│                        │
│                    └───────────────────┘                        │
│                              │                                   │
│                        Claude Code                               │
└──────────────────────────────────────────────────────────────────┘
```

## Quick Commands

### Start ROS2 Simulation
```bash
cd docker && docker compose up -d
```

### View Live Simulation
Open http://localhost:6080/vnc.html in browser

### Test MCP Connection
```bash
source .sitebot_env/bin/activate
python -c "from mujoco_mcp.rosbridge_client import get_rosbridge_client; c = get_rosbridge_client(); print(c.connect())"
```

### Run Standalone MuJoCo Test
```bash
source .sitebot_env/bin/activate
python scripts/test_robot.py
```

## Key Files

| File | Purpose |
|------|---------|
| `models/sitebot.xml` | MuJoCo robot model (differential drive) |
| `worlds/construction_site.xml` | Simulation environment |
| `docker/docker-compose.yml` | ROS2 container orchestration |
| `docker/Dockerfile.ros2` | ROS2 Humble + mujoco_ros2_control |
| `ros2_ws/src/sitebot_ros/` | ROS2 package (launch, URDF, controllers) |
| `mcp_server/src/mujoco_mcp/` | MCP server with ROS2 tools |
| `mcp_server/src/mujoco_mcp/rosbridge_client.py` | rosbridge WebSocket client |

## MCP Tools Available

### Simulation Tools (mujoco-mcp)
- `create_scene` - Create physics scenes (pendulum, cart_pole, arm)
- `step_simulation` - Step physics forward
- `get_state` - Get simulation state
- `set_control` - Apply motor torques
- `capture_frame` - Render simulation frame

### ROS2 Tools (via rosbridge)
- `ros2_connect` - Connect to rosbridge WebSocket
- `ros2_drive` - Publish velocity commands (/cmd_vel)
- `ros2_get_pose` - Get odometry (/odom)
- `ros2_stop` - Stop robot motion
- `ros2_navigate_to` - Send Nav2 goal

## Robot Specifications

- **Type**: Differential drive (2 drive wheels + 2 casters)
- **Wheel separation**: 0.6m
- **Wheel radius**: 0.1m
- **Max velocity**: ~2 m/s (linear), ~4 rad/s (angular)
- **Sensors**: Position, orientation, velocity (via framepos/framequat sensors)

## Development Notes

### Virtual Environment
```bash
source .sitebot_env/bin/activate
```

### Rebuild Docker Container
```bash
cd docker && docker compose build --no-cache
```

### Check ROS2 Topics (inside container)
```bash
docker exec -it sitebot_ros2 bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /diff_drive_controller/odom
```

### Known Issues
- Robot velocity at ~25% of commanded (tuning needed in motor gains)
- Nav2 requires SLAM/localization setup for full autonomous navigation

## Testing

```bash
cd mcp_server
pytest tests/ -v
```

## Dependencies

- Python 3.12+
- MuJoCo 3.0+
- ROS2 Humble (in Docker)
- mujoco_ros2_control (submodule)
- roslibpy (rosbridge client)
