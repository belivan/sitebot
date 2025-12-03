"""
Coverage Path Planning for SiteBot.

Generates paths to cover a rectangular area - similar to lawn mower patterns
used in construction surveying/marking robots.
"""

import numpy as np
from dataclasses import dataclass
from enum import Enum
from typing import Generator


class PatternType(Enum):
    """Coverage pattern types."""
    BOUSTROPHEDON = "boustrophedon"  # Back-and-forth (ox-turning)
    SPIRAL = "spiral"                 # Inward spiral
    GRID = "grid"                     # Grid pattern for marking


@dataclass
class CoverageArea:
    """Rectangular coverage area."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        return self.y_max - self.y_min

    @property
    def center(self) -> tuple[float, float]:
        return (self.x_min + self.x_max) / 2, (self.y_min + self.y_max) / 2


class CoveragePathPlanner:
    """
    Generates coverage paths for construction site operations.

    Supports multiple patterns commonly used in:
    - Surveying/layout (grid marking)
    - Surface preparation
    - Inspection
    """

    def __init__(self, line_spacing: float = 1.0):
        """
        Initialize planner.

        Args:
            line_spacing: Distance between parallel lines (m)
        """
        self.line_spacing = line_spacing

    def generate_boustrophedon(self, area: CoverageArea,
                                start_corner: str = "sw") -> list[tuple[float, float]]:
        """
        Generate boustrophedon (lawn mower) path.

        Back-and-forth pattern, efficient for continuous coverage.

        Args:
            area: Coverage area
            start_corner: Starting corner ("sw", "se", "nw", "ne")

        Returns:
            List of waypoints
        """
        waypoints = []

        # Determine number of lines
        num_lines = int(np.ceil(area.width / self.line_spacing)) + 1

        # Generate x positions for lines
        x_positions = np.linspace(area.x_min, area.x_max, num_lines)

        # Adjust starting corner
        if start_corner in ("se", "ne"):
            x_positions = x_positions[::-1]

        y_start = area.y_min if start_corner in ("sw", "se") else area.y_max
        y_end = area.y_max if start_corner in ("sw", "se") else area.y_min

        for i, x in enumerate(x_positions):
            if i % 2 == 0:
                waypoints.append((x, y_start))
                waypoints.append((x, y_end))
            else:
                waypoints.append((x, y_end))
                waypoints.append((x, y_start))

        return waypoints

    def generate_grid(self, area: CoverageArea,
                      spacing: float = None) -> list[tuple[float, float]]:
        """
        Generate grid marking pattern.

        Visits discrete points in a grid - used for marking/surveying.

        Args:
            area: Coverage area
            spacing: Grid spacing (uses line_spacing if None)

        Returns:
            List of waypoints (grid intersection points)
        """
        spacing = spacing or self.line_spacing
        waypoints = []

        # Generate grid points
        x_points = np.arange(area.x_min, area.x_max + spacing/2, spacing)
        y_points = np.arange(area.y_min, area.y_max + spacing/2, spacing)

        # Traverse in boustrophedon order for efficiency
        for i, x in enumerate(x_points):
            if i % 2 == 0:
                for y in y_points:
                    waypoints.append((x, y))
            else:
                for y in reversed(y_points):
                    waypoints.append((x, y))

        return waypoints

    def generate_spiral(self, area: CoverageArea,
                        inward: bool = True) -> list[tuple[float, float]]:
        """
        Generate spiral path.

        Used for inspection patterns or progressive area coverage.

        Args:
            area: Coverage area
            inward: If True, spiral inward; else spiral outward

        Returns:
            List of waypoints
        """
        waypoints = []

        # Current bounds
        x_min, x_max = area.x_min, area.x_max
        y_min, y_max = area.y_min, area.y_max

        spacing = self.line_spacing

        while x_max - x_min > spacing and y_max - y_min > spacing:
            # Bottom edge (left to right)
            waypoints.append((x_min, y_min))
            waypoints.append((x_max, y_min))

            # Right edge (bottom to top)
            waypoints.append((x_max, y_max))

            # Top edge (right to left)
            waypoints.append((x_min, y_max))

            # Shrink bounds for next loop
            x_min += spacing
            x_max -= spacing
            y_min += spacing
            y_max -= spacing

            # Close the loop back to next layer
            if x_max - x_min > spacing:
                waypoints.append((x_min, y_min + spacing))

        # Add center point
        center = area.center
        waypoints.append(center)

        if not inward:
            waypoints = waypoints[::-1]

        return waypoints

    def generate_perimeter(self, area: CoverageArea,
                           offset: float = 0.0) -> list[tuple[float, float]]:
        """
        Generate perimeter path.

        Traces the boundary of the area.

        Args:
            area: Coverage area
            offset: Inward offset from boundary

        Returns:
            List of waypoints
        """
        x_min = area.x_min + offset
        x_max = area.x_max - offset
        y_min = area.y_min + offset
        y_max = area.y_max - offset

        return [
            (x_min, y_min),
            (x_max, y_min),
            (x_max, y_max),
            (x_min, y_max),
            (x_min, y_min),  # Close the loop
        ]


def demo_patterns():
    """Demonstrate coverage patterns."""
    area = CoverageArea(x_min=-5, x_max=5, y_min=-5, y_max=5)
    planner = CoveragePathPlanner(line_spacing=2.0)

    print("Boustrophedon pattern:")
    for wp in planner.generate_boustrophedon(area)[:10]:
        print(f"  {wp}")

    print("\nGrid pattern:")
    for wp in planner.generate_grid(area)[:10]:
        print(f"  {wp}")


if __name__ == "__main__":
    demo_patterns()
