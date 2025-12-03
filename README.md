# SiteBot - Construction Robotics Simulation

A MuJoCo-based autonomous robot simulation for construction site surveying, featuring ROS2/Nav2 integration and MCP (Model Context Protocol) for AI-assisted control.

## Features

- **Physics Simulation**: MuJoCo for accurate robot dynamics
- **ROS2 Integration**: ros2_control with diff_drive_controller
- **Navigation**: Nav2 stack for autonomous navigation
- **MCP Server**: Control robot via Claude natural language
- **Live Viewer**: Web-based simulation viewer via noVNC

## Quick Start

### Prerequisites

- Docker Desktop
- Python 3.12+
- macOS (tested on M2)

### 1. Clone Repository

```bash
git clone --recursive https://github.com/belivan/sitebot.git
cd sitebot
```

### 2. Start ROS2 Simulation

```bash
cd docker
docker compose up -d
```

### 3. View Simulation

Open http://localhost:6080/vnc.html in your browser.

### 4. Control via MCP

Add to Claude Code MCP settings (`.mcp.json`):

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

## Architecture

```
┌────────────────────────────────────────┐
│        Docker (ROS2 Humble)            │
│                                        │
│  MuJoCo ─▶ mujoco_ros2_control ─▶ Nav2 │
│                    │                   │
│            rosbridge:9090              │
└────────────────────┼───────────────────┘
                     │
              ┌──────┴──────┐
              │ MCP Server  │
              │ (roslibpy)  │
              └──────┬──────┘
                     │
              Claude Code
```

## Project Structure

```
sitebot/
├── docker/
│   ├── Dockerfile.ros2          # ROS2 Humble container
│   └── docker-compose.yml       # Container orchestration
├── models/
│   └── sitebot.xml              # MuJoCo robot model
├── worlds/
│   └── construction_site.xml    # Simulation environment
├── ros2_ws/src/
│   ├── sitebot_ros/             # ROS2 package
│   │   ├── config/              # Controller configs
│   │   ├── launch/              # Launch files
│   │   └── urdf/                # Robot description
│   └── mujoco_ros2_control/     # MoveIt submodule
├── mcp_server/
│   └── src/mujoco_mcp/          # MCP server
│       ├── server.py            # FastMCP server
│       └── rosbridge_client.py  # ROS2 bridge
├── src/                         # Python modules
│   ├── control/                 # Controllers
│   ├── navigation/              # Waypoint navigation
│   └── planning/                # Path planning
└── scripts/                     # Utility scripts
```

## Robot Specifications

| Parameter | Value |
|-----------|-------|
| Type | Differential drive |
| Wheel separation | 0.6 m |
| Wheel radius | 0.1 m |
| Max linear velocity | ~2 m/s |
| Max angular velocity | ~4 rad/s |

## Usage

### Drive Robot (via MCP)

```
"Drive forward at 0.5 m/s"
"Turn left at 1 rad/s"
"Stop the robot"
```

### Navigate (via MCP)

```
"Navigate to position (5, 3)"
"Where is the robot?"
```

### Manual Control (via ROS2)

```bash
# Inside container
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

## Ports

| Port | Service |
|------|---------|
| 9090 | rosbridge WebSocket |
| 6080 | noVNC web viewer |

## Development

### Setup Local Environment

```bash
python3 -m venv .sitebot_env
source .sitebot_env/bin/activate
pip install -r requirements.txt
pip install -e mcp_server/
```

### Rebuild Container

```bash
cd docker
docker compose build --no-cache
docker compose up -d
```

### Run Tests

```bash
cd mcp_server
pytest tests/ -v
```

## License

MIT

## Acknowledgments

- [MuJoCo](https://mujoco.org/) - Physics simulation
- [mujoco_ros2_control](https://github.com/moveit/mujoco_ros2_control) - ROS2 integration
- [Nav2](https://nav2.org/) - Navigation stack
- [rosbridge](http://wiki.ros.org/rosbridge_suite) - WebSocket bridge
