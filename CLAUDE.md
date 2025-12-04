# CLAUDE.md - Claude Code Context

## Project Overview

SiteBot is a construction robotics simulation platform using Gazebo Harmonic for simulation with ROS2 Jazzy/Nav2 for autonomous navigation. MuJoCo models are kept for future RL training. It provides MCP (Model Context Protocol) integration for AI-assisted robot control.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│              Docker Container (ROS2 Jazzy + Gazebo Harmonic)    │
│                                                                 │
│  ┌──────────────┐    ┌───────────────────┐    ┌─────────────┐  │
│  │   Gazebo     │───▶│   ros_gz_bridge   │───▶│   Nav2      │  │
│  │  Harmonic    │    │  (topic bridge)   │    │   Stack     │  │
│  └──────────────┘    └───────────────────┘    └─────────────┘  │
│         │                                                       │
│    GPU Lidar ─────────────────────────────▶ /scan              │
│    diff_drive ────────────────────────────▶ /odom, /cmd_vel    │
│                                                                 │
│                    ┌─────────────────────┐                     │
│                    │ rosbridge_websocket │ (port 9090)         │
│                    └─────────┬───────────┘                     │
└──────────────────────────────┼──────────────────────────────────┘
                               │ WebSocket
┌──────────────────────────────┼──────────────────────────────────┐
│                         Mac Host                                 │
│                    ┌─────────┴─────────┐                        │
│                    │    MCP Server     │                        │
│                    │ (rosbridge client)│                        │
│                    └───────────────────┘                        │
│                              │                                   │
│   MuJoCo (standalone) ◀──── │ ────▶ Claude Code                 │
│   (for RL training)         │                                   │
└──────────────────────────────────────────────────────────────────┘
```

## Quick Commands

### Start ROS2 Simulation
```bash
cd docker && docker compose up -d
```

### View Live Simulation
Open http://localhost:6080/vnc_lite.html in browser
- Gazebo (left): 3D simulation with robot and lidar rays
- RViz (right): Sensor visualization (lidar, TF, odometry)

### Test MCP Connection
```bash
source .sitebot_env/bin/activate
python -c "from mujoco_mcp.rosbridge_client import get_rosbridge_client; c = get_rosbridge_client(); print(c.connect())"
```

### Run Standalone MuJoCo Test (for RL)
```bash
source .sitebot_env/bin/activate
python scripts/test_robot.py
```

### Launch with SLAM
```bash
docker exec -it sitebot_ros2 bash
ros2 launch sitebot_ros gazebo.launch.py use_slam:=true
```

### Launch with Nav2
```bash
docker exec -it sitebot_ros2 bash
ros2 launch sitebot_ros gazebo.launch.py use_nav2:=true use_slam:=true
```

## Key Files

| File | Purpose |
|------|---------|
| `gazebo/models/sitebot/model.sdf` | Gazebo robot model (diff_drive + GPU lidar) |
| `gazebo/worlds/construction_site.sdf` | Gazebo simulation world |
| `models/sitebot.xml` | MuJoCo robot model (for RL training) |
| `worlds/construction_site.xml` | MuJoCo world (for RL training) |
| `docker/docker-compose.yml` | ROS2 container orchestration |
| `docker/Dockerfile.ros2` | ROS2 Jazzy + Gazebo Harmonic |
| `ros2_ws/src/sitebot_ros/` | ROS2 package (launch, config) |
| `ros2_ws/src/sitebot_ros/config/gz_bridge.yaml` | ros_gz_bridge topic config |
| `ros2_ws/src/sitebot_ros/config/sitebot.rviz` | RViz visualization config |
| `mcp_server/src/mujoco_mcp/` | MCP server with ROS2 tools |

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
- **Sensors**: 360-degree GPU lidar (30m range), IMU

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
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

### Launch Gazebo GUI and RViz (inside container)
```bash
docker exec sitebot_ros2 bash -c "DISPLAY=:1 gz sim -g &"
docker exec sitebot_ros2 bash -c "DISPLAY=:1 rviz2 -d /ros2_ws/src/sitebot_ros/config/sitebot.rviz &"
```

### Organize Windows Side-by-Side
```bash
docker exec sitebot_ros2 bash -c "DISPLAY=:1 wmctrl -r 'Gazebo' -e 0,0,0,960,1080"
docker exec sitebot_ros2 bash -c "DISPLAY=:1 wmctrl -r 'RViz' -e 0,960,0,960,1080"
```

## Testing

```bash
cd mcp_server
pytest tests/ -v
```

## Dependencies

- Python 3.12+
- MuJoCo 3.0+ (for standalone RL)
- ROS2 Jazzy (in Docker)
- Gazebo Harmonic (in Docker)
- roslibpy (rosbridge client)
