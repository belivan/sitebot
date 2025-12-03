# SiteBot Setup Guide

## Prerequisites

- Docker Desktop (for ROS2 simulation)
- Python 3.12+
- macOS (tested on M2 Apple Silicon)

## Quick Start

### Option 1: ROS2 Simulation (Recommended)

```bash
# Start the full simulation stack
cd docker
docker compose up -d

# View simulation in browser
open http://localhost:6080/vnc.html
```

### Option 2: Standalone MuJoCo (No ROS2)

```bash
# Activate environment
source .sitebot_env/bin/activate

# Test robot model
python scripts/test_robot.py
```

## Detailed Setup

### 1. Clone Repository

```bash
git clone --recursive https://github.com/belivan/sitebot.git
cd sitebot
```

If you forgot `--recursive`, initialize submodules:

```bash
git submodule update --init --recursive
```

### 2. Python Environment

```bash
# Create virtual environment
python3 -m venv .sitebot_env
source .sitebot_env/bin/activate

# Install dependencies
pip install -r requirements.txt

# Install MCP server (editable)
pip install -e mcp_server/
```

### 3. Docker Container (ROS2)

```bash
cd docker

# Build container (first time ~10 min)
docker compose build

# Start simulation
docker compose up -d

# View logs
docker compose logs -f

# Stop
docker compose down
```

## MCP Integration for Claude Code

### Configure Claude Code

Create or update `.mcp.json` in project root:

```json
{
  "mcpServers": {
    "mujoco-mcp": {
      "command": "/path/to/sitebot/.sitebot_env/bin/python",
      "args": ["-m", "mujoco_mcp"]
    }
  }
}
```

Replace `/path/to/sitebot` with your actual path.

### Available MCP Commands

**Simulation Control:**
- `create_scene` - Create physics scenes
- `step_simulation` - Step simulation forward
- `get_state` - Get current state
- `set_control` - Apply motor controls

**ROS2 Control (requires Docker running):**
- `ros2_connect` - Connect to rosbridge
- `ros2_drive` - Send velocity commands
- `ros2_get_pose` - Get robot odometry
- `ros2_stop` - Emergency stop
- `ros2_navigate_to` - Nav2 goal (requires Nav2)

### Natural Language Examples

```
"Connect to the ROS2 simulation"
"Drive forward at 0.5 m/s"
"Where is the robot?"
"Stop the robot"
"Navigate to position (5, 3)"
```

## Accessing Services

| Service | URL/Port | Description |
|---------|----------|-------------|
| noVNC Viewer | http://localhost:6080/vnc.html | Live simulation view |
| rosbridge | ws://localhost:9090 | WebSocket for MCP |

## Troubleshooting

### Container Won't Start

```bash
# Check logs
docker compose logs ros2

# Rebuild from scratch
docker compose build --no-cache
```

### MCP Connection Failed

```bash
# Verify rosbridge is running
curl -I http://localhost:9090

# Check container status
docker ps
```

### Simulation Not Visible

1. Open http://localhost:6080/vnc.html
2. Click "Connect"
3. If black screen, simulation may still be loading

### Robot Moves Slowly

Known issue: Robot achieves ~25% of commanded velocity. Motor tuning in progress.

## Development Workflow

### Modify Robot Model

1. Edit `models/sitebot.xml`
2. Changes apply on next container restart:
   ```bash
   docker compose restart
   ```

### Modify ROS2 Code

1. Edit files in `ros2_ws/src/sitebot_ros/`
2. Rebuild inside container:
   ```bash
   docker exec -it sitebot_ros2 bash
   source /opt/ros/humble/setup.bash
   cd /ros2_ws && colcon build
   ```

### Modify MCP Server

1. Edit files in `mcp_server/src/mujoco_mcp/`
2. Reinstall:
   ```bash
   pip install -e mcp_server/
   ```

## Testing

```bash
# Run MCP server tests
cd mcp_server
pytest tests/ -v

# Test robot model loading
python -c "
import mujoco
model = mujoco.MjModel.from_xml_path('models/sitebot.xml')
print(f'Robot loaded: {model.nq} DOF, {model.nu} actuators')
"
```

## Project Structure

```
sitebot/
├── .sitebot_env/          # Python virtual environment
├── docker/
│   ├── Dockerfile.ros2    # ROS2 Humble container
│   └── docker-compose.yml # Container config
├── models/
│   └── sitebot.xml        # MuJoCo robot model
├── worlds/
│   └── construction_site.xml
├── ros2_ws/src/
│   ├── sitebot_ros/       # ROS2 package
│   │   ├── config/        # Controller YAML
│   │   ├── launch/        # Launch files
│   │   └── urdf/          # Robot URDF
│   └── mujoco_ros2_control/  # Git submodule
├── mcp_server/
│   └── src/mujoco_mcp/    # MCP server
├── src/                   # Python modules
│   ├── control/           # Controllers
│   ├── navigation/        # Navigation
│   └── planning/          # Path planning
└── scripts/               # Utility scripts
```

## Key Features Demonstrated

1. **Physics Simulation**: MuJoCo for accurate dynamics
2. **ROS2 Integration**: mujoco_ros2_control + ros2_control
3. **Navigation**: Nav2 stack (optional)
4. **AI Control**: MCP server for LLM interaction
5. **Visualization**: Web-based viewer via noVNC
6. **Modular Architecture**: Clean separation of concerns

## Similar to CIV Robotics CivDot

This project demonstrates skills relevant to autonomous construction site surveying robots.
