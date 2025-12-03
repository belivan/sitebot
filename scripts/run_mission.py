#!/usr/bin/env python3
"""
SiteBot Mission Runner.

Runs a complete coverage mission in the construction site environment.
Demonstrates autonomous navigation and path planning.
"""

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import mujoco
import mujoco.viewer
import numpy as np
import time
from dataclasses import dataclass

from control.diff_drive import DiffDriveController, DiffDriveParams
from navigation.waypoint_nav import WaypointNavigator, NavigationParams, Pose2D
from planning.coverage import CoveragePathPlanner, CoverageArea


@dataclass
class SimulationConfig:
    """Simulation configuration."""
    model_path: str = "worlds/construction_site.xml"
    realtime: bool = True
    max_time: float = 300.0  # 5 minutes max


class SiteBotSimulation:
    """
    Main simulation class for SiteBot.

    Integrates MuJoCo physics with navigation and planning.
    """

    def __init__(self, config: SimulationConfig = None):
        self.config = config or SimulationConfig()

        # Find model path
        project_root = Path(__file__).parent.parent
        model_path = project_root / self.config.model_path
        print(f"Loading model: {model_path}")

        # Load MuJoCo model
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)

        # Controllers
        self.drive_controller = DiffDriveController(DiffDriveParams(
            wheel_radius=0.1,
            wheel_separation=0.6
        ))

        # Navigation
        self.navigator = WaypointNavigator(NavigationParams(
            goal_tolerance=0.2,
            angle_tolerance=0.15,
            linear_speed=0.5,
            angular_speed=1.5
        ))

        # Path planner
        self.planner = CoveragePathPlanner(line_spacing=2.0)

        # State
        self.mission_active = False
        self.visited_points = []

    def get_robot_pose(self) -> Pose2D:
        """Get current robot pose from simulation."""
        # Position from sensor
        pos = self.data.sensordata[0:3]

        # Quaternion from sensor
        quat = self.data.sensordata[3:7]

        # Extract yaw from quaternion (z-rotation)
        # quat is [w, x, y, z]
        w, x, y, z = quat
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

        return Pose2D(x=pos[0], y=pos[1], theta=yaw)

    def set_wheel_velocities(self, left_vel: float, right_vel: float):
        """Set wheel motor commands."""
        self.data.ctrl[0] = left_vel
        self.data.ctrl[1] = right_vel

    def run_coverage_mission(self, area: CoverageArea, pattern: str = "grid"):
        """
        Run a coverage mission over the specified area.

        Args:
            area: Area to cover
            pattern: Pattern type ("grid", "boustrophedon", "perimeter")
        """
        # Generate waypoints
        if pattern == "grid":
            waypoints = self.planner.generate_grid(area)
        elif pattern == "boustrophedon":
            waypoints = self.planner.generate_boustrophedon(area)
        elif pattern == "perimeter":
            waypoints = self.planner.generate_perimeter(area)
        else:
            raise ValueError(f"Unknown pattern: {pattern}")

        print(f"\nMission: {pattern.upper()} coverage")
        print(f"Area: ({area.x_min}, {area.y_min}) to ({area.x_max}, {area.y_max})")
        print(f"Waypoints: {len(waypoints)}")

        # Set waypoints
        self.navigator.set_waypoints(waypoints)
        self.mission_active = True
        self.visited_points = []

        return waypoints

    def step(self, dt: float):
        """
        Execute one simulation step.

        Args:
            dt: Time step
        """
        if not self.mission_active:
            self.set_wheel_velocities(0, 0)
            return

        # Get current pose
        pose = self.get_robot_pose()

        # Compute velocity command
        linear, angular = self.navigator.compute_velocity(pose, dt)

        # Convert to wheel velocities
        left_vel, right_vel = self.drive_controller.cmd_vel_to_wheel_vel(linear, angular)

        # Apply to simulation
        self.set_wheel_velocities(left_vel, right_vel)

        # Track visited points
        if self.navigator.current_waypoint:
            dist = pose.distance_to(self.navigator.current_waypoint)
            if dist < 0.3:
                wp = (self.navigator.current_waypoint.x, self.navigator.current_waypoint.y)
                if wp not in self.visited_points:
                    self.visited_points.append(wp)
                    progress = self.navigator.get_progress()
                    print(f"  Reached waypoint {progress[0]}/{progress[1]}: ({wp[0]:.1f}, {wp[1]:.1f})")

        # Check if mission complete
        if self.navigator.is_navigation_complete():
            print("\n✓ Mission complete!")
            print(f"  Visited {len(self.visited_points)} points")
            self.mission_active = False

    def run_with_viewer(self):
        """Run simulation with interactive viewer."""
        # Define coverage area (survey markers area)
        area = CoverageArea(x_min=-6, x_max=6, y_min=-6, y_max=6)

        # Start mission
        waypoints = self.run_coverage_mission(area, pattern="grid")

        print("\nStarting simulation...")
        print("  - Robot will navigate through grid points")
        print("  - Close viewer to exit\n")

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            start_time = time.time()
            last_print = 0

            while viewer.is_running():
                elapsed = time.time() - start_time

                # Timeout
                if elapsed > self.config.max_time:
                    print("Mission timeout!")
                    break

                # Run simulation step
                self.step(self.model.opt.timestep)
                mujoco.mj_step(self.model, self.data)

                # Status update every 5 seconds
                if elapsed - last_print > 5:
                    pose = self.get_robot_pose()
                    progress = self.navigator.get_progress()
                    print(f"  [{elapsed:.0f}s] Position: ({pose.x:.2f}, {pose.y:.2f}) | "
                          f"Progress: {progress[0]}/{progress[1]}")
                    last_print = elapsed

                # Sync viewer
                viewer.sync()

                # Realtime
                if self.config.realtime:
                    time.sleep(self.model.opt.timestep)

                # Exit if mission complete
                if not self.mission_active:
                    time.sleep(2)  # Show final position
                    break


def main():
    """Main entry point."""
    print("=" * 50)
    print("SiteBot - Construction Site Robot Simulation")
    print("=" * 50)

    sim = SiteBotSimulation()
    sim.run_with_viewer()

    print("\nSimulation ended.")


if __name__ == "__main__":
    main()
