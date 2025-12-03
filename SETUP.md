# SiteBot Setup Guide

## Quick Start

### 1. Activate Environment

```bash
cd /Users/antonyanovich/Projects/Robotics/sitebot
source .sitebot_env/bin/activate
```

### 2. Run Simulation with Viewer

```bash
python scripts/run_mission.py
```

This will:
- Load the construction site world
- Run a grid coverage mission
- Show the MuJoCo viewer

### 3. Test Robot (Headless)

```bash
python -c "
import mujoco
model = mujoco.MjModel.from_xml_path('models/sitebot.xml')
data = mujoco.MjData(model)
print('Robot loaded successfully!')
"
```

## MCP Integration for Claude

### Configure Claude Code

Add this to your Claude Code MCP settings:

```json
{
  "mcpServers": {
    "sitebot": {
      "command": "/Users/antonyanovich/Projects/Robotics/sitebot/.sitebot_env/bin/python",
      "args": ["/Users/antonyanovich/Projects/Robotics/sitebot/scripts/sitebot_mcp_server.py"]
    }
  }
}
```

### Available MCP Commands

Once configured, you can use natural language:

- **load_world**: "Load the construction site"
- **get_robot_pose**: "Where is the robot?"
- **move_robot**: "Move forward at 0.5 m/s"
- **navigate_to**: "Go to position (5, 3)"
- **run_coverage**: "Cover the area from -5,-5 to 5,5"

## Project Structure

```
sitebot/
├── .sitebot_env/          # Python virtual environment
├── models/
│   └── sitebot.xml        # Robot model
├── worlds/
│   └── construction_site.xml  # Environment
├── src/
│   ├── control/           # Differential drive controller
│   ├── navigation/        # Waypoint navigation
│   └── planning/          # Coverage path planning
├── scripts/
│   ├── run_mission.py     # Main simulation runner
│   ├── test_robot.py      # Robot test script
│   └── sitebot_mcp_server.py  # MCP server
└── mcp_server/            # MuJoCo MCP server (cloned)
```

## Key Features

- **Differential Drive Robot**: 4-wheeled robot with sensor mast
- **Construction Site Environment**: With obstacles, markers, boundaries
- **Coverage Path Planning**: Grid, boustrophedon, spiral patterns
- **Waypoint Navigation**: PID-based go-to-goal controller
- **MCP Integration**: Control via Claude natural language

## Interview Talking Points

This project demonstrates:

1. **Physics Simulation**: MuJoCo for accurate robot dynamics
2. **Motion Planning**: Coverage path planning algorithms
3. **Control Systems**: Differential drive kinematics, PID control
4. **Navigation**: Waypoint following, heading control
5. **AI Integration**: MCP server for LLM control
6. **Software Architecture**: Modular Python packages

Similar to CIV Robotics CivDot autonomous surveying robot.
