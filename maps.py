'''
AER1516 - Project
File that contains the map definitions for the project. Each map is represented as a grid with obstacles, a start point, and a goal point. 
The maps can be easily modified or extended by changing the grid layout and obstacle placements
'''

from matplotlib.pyplot import grid
import numpy as np
from dynamics import RobotDynamics
from dynamics import State

# =========================
# Map class
# =========================
class Map:
    def __init__(self, grid, start, goals, name, static_obstacles=None, dynamic_obstacles=None): # goals is a list of (x, y) tuples
        self.grid = grid
        self.cell_size = cell_size
        self.dimensions = grid.shape # outputs (row dim, col dim) --> (height, width)
        self.start = start
        self.goals = goals
        self.name = name
        self.dynamic_obstacles = dynamic_obstacles if dynamic_obstacles is not None else []
    
    @property
    def width(self) -> float:
        """Width of the map in meters (columns)."""
        return self.dimensions[1] * self.cell_size

    @property
    def height(self) -> float:
        """Height of the map in meters (rows)."""
        return self.dimensions[0] * self.cell_size
    
    def get_obstacles_grid_position(self) -> list[tuple[int, int]]:
        """Return list of (row, col) of all obstacle cells."""
        rows, cols = np.nonzero(self.grid == 1)
        return list(zip(rows, cols))
    
    def meters_to_grid(self, x_m: float, y_m: float) -> tuple[int, int]:
        """
        Convert a position in meters to a grid cell (row, col).
        Returns:
            (row, col) grid cell indices
        """
        col = int(x_m / self.cell_size)
        row = int(y_m / self.cell_size)
        return (row, col)
    
    def grid_to_meters(self, row: int, col: int) -> tuple[float, float]:
        """
        Convert a grid cell (row, col) to its center position in meters.
        Returns:
            (x_m, y_m) position of the cell center
        """
        x_m = (col + 0.5) * self.cell_size
        y_m = (row + 0.5) * self.cell_size
        return (x_m, y_m)
    
    
    def is_in_bounds(self, row: int, col: int) -> bool:
        """Check if a grid cell (row, col) is within the map boundaries."""
        return (0 <= row < self.dimensions[0]) and (0 <= col < self.dimensions[1])

    
    def get_obstacle_clusters(self) -> list[dict]:
        """
        Detect connected clusters of obstacle cells using flood fill,
        and return their bounding boxes in both cells and meters.

        Returns list of dicts with keys:
            cells                               - list of (row, col) in the cluster
            min_row, max_row, min_col, max_col  - bounding box in cells
            x_min, x_max, y_min, y_max          - bounding box in meters
            width_m, height_m                   - size in meters
        """

        labeled, num_clusters = label(self.grid == 1)
        clusters = []

        for i in range(1, num_clusters + 1):
            rows, cols = np.where(labeled == i)
            min_row, max_row = int(rows.min()), int(rows.max())
            min_col, max_col = int(cols.min()), int(cols.max())

            clusters.append({
                "cells":    list(zip(rows.tolist(), cols.tolist())),
                # cell-space bounding box
                "min_row":  min_row,
                "max_row":  max_row,
                "min_col":  min_col,
                "max_col":  max_col,
                # meter-space bounding box
                "x_min":    min_col * self.cell_size,
                "x_max":    (max_col + 1) * self.cell_size,
                "y_min":    min_row * self.cell_size,
                "y_max":    (max_row + 1) * self.cell_size,
            })

        return clusters
    

        self.static_obstacles = static_obstacles if static_obstacles is not None else []
        self.dynamic_obstacles = dynamic_obstacles if dynamic_obstacles is not None else []


class Obstacle:
    """
    Base obstacle class (geometry only).
    """

    def __init__(self, x, y, w, h):
        self.x = x  # column
        self.y = y  # row
        self.w = w
        self.h = h

    # =========================
    # Geometry
    # =========================
    def get_rect(self):
        return (self.x, self.y, self.w, self.h)

    def collides_with_point(self, px, py, radius=0.0):
        """
        Collision check with circular robot via rectangle inflation.
        """
        return (
            self.x - radius <= px <= self.x + self.w + radius and
            self.y - radius <= py <= self.y + self.h + radius
        )

    def __repr__(self):
        return f"Obstacle(x={self.x}, y={self.y}, w={self.w}, h={self.h})"
    
class StaticObstacle(Obstacle):
    """
    Static rectangular obstacle.
    """

    def __init__(self, x, y, w, h):
        super().__init__(x, y, w, h)

class DynamicObstacle(Obstacle):
    """
    Dynamic obstacle (square only) with velocity.
    """

    def __init__(self, x, y, size, vel=(0, 0)):
        super().__init__(x, y, size, size)

        self.size = size
        self.vel = list(vel)

        # store initial position for prediction
        self.initial_pos = (x, y)

    # =========================
    # Motion
    # =========================
    def update(self, grid):
        """
        Move obstacle with bounce logic.
        """
        vx, vy = self.vel

        new_x = self.x + vx
        new_y = self.y + vy

        if self._valid_position(grid, new_x, new_y):
            self.x = new_x
            self.y = new_y
        else:
            # bounce
            self.vel[0] *= -1
            self.vel[1] *= -1

    def _valid_position(self, grid, x, y):
        for i in range(int(self.h)):
            for j in range(int(self.w)):
                r = int(y + i)
                c = int(x + j)

                if r < 0 or r >= grid.shape[0] or c < 0 or c >= grid.shape[1]:
                    return False

                if grid[r, c] == 1:
                    return False

        return True

    def get_position_at_time(self, t):
        """
        Predict position at time t (for planning).
        NOTE: simple linear model (no bounce prediction yet)
        """
        x = self.initial_pos[0] + self.vel[0] * t
        y = self.initial_pos[1] + self.vel[1] * t

        return DynamicObstacle(x, y, self.size, self.vel)

    def __repr__(self):
        return f"DynamicObstacle(x={self.x}, y={self.y}, size={self.size}, vel={self.vel})"

# =========================
# Helper: add walls
# =========================
def create_boundary_obstacles(rows, cols):
    return [
        StaticObstacle(0, 0, cols, 1),
        StaticObstacle(0, rows - 1, cols, 1),
        StaticObstacle(0, 0, 1, rows),
        StaticObstacle(cols - 1, 0, 1, rows),
    ]

'''
Note: the map is 20x20 but the interior is 18x18 becasue of the boundaries. A lot of Activate maps also have boundaries so you're not too close to the walls
'''

# =========================
# Map 1: Static Map (simple)
# =========================
def simple():
    grid = np.zeros((20, 20))

    # simple obstacles - (x, y, w, h)
    static_obstacles = create_boundary_obstacles(20, 20) + [
        StaticObstacle(5, 6, 3, 4),
        StaticObstacle(2, 12, 2, 3),
        StaticObstacle(11, 13, 3, 3),
        StaticObstacle(12, 2, 3, 3),
        StaticObstacle(15, 7, 2, 4),
    ]

    dynamic_obstacles = []

    start = (2, 2)
    goals = [(18, 18), (16, 5)]  # list of goals, can add more later

    return Map(grid, start, goals, "Simple Map", static_obstacles, dynamic_obstacles)

# =========================
# Map 2: Static Map (narrow passage)
# =========================
def narrow_passage():
    grid = np.zeros((20, 20))

    # simple obstacles - (x, y, w, h)
    static_obstacles = create_boundary_obstacles(20, 20) + [
        # wall 1 (left + right of gap)
        StaticObstacle(0, 4, 15, 2),
        StaticObstacle(16, 4, 4, 2),

        # wall 2 (full)
        StaticObstacle(0, 9, 20, 1),

        # wall 3 (left + right of gap)
        StaticObstacle(0, 13, 6, 2),
        StaticObstacle(7, 13, 13, 2),
    ]

    dynamic_obstacles = []

    start = (2, 2)
    goals = [(17, 17)]

    return Map(grid, start, goals, "Narrow Passage Map w/ Jump", static_obstacles, dynamic_obstacles)

# =========================
# Map 3: Static Map (multi-passage)
# =========================
def multi_passage():
    grid = np.zeros((20, 20))

    # simple obstacles - (x, y, w, h)
    static_obstacles = create_boundary_obstacles(20, 20) + [
        # y = 4
        StaticObstacle(0, 4, 2, 1),
        StaticObstacle(3, 4, 9, 1),
        StaticObstacle(13, 4, 7, 1),

        # y = 8
        StaticObstacle(0, 8, 6, 1),
        StaticObstacle(7, 8, 9, 1),
        StaticObstacle(17, 8, 3, 1),

        # y = 12
        StaticObstacle(0, 12, 3, 1),
        Obstacle(4, 12, 3, 1),
        StaticObstacle(8, 12, 4, 1),
        StaticObstacle(13, 12, 7, 1),

        # y = 16
        StaticObstacle(0, 16, 1, 1),
        StaticObstacle(2, 16, 5, 1),
        StaticObstacle(8, 16, 7, 1),
        StaticObstacle(16, 16, 4, 1),
    ]

    dynamic_obstacles = []

    start = (2, 9)
    goals = [(18, 8)]

    return Map(grid, start, goals, "Multi Route Map", static_obstacles, dynamic_obstacles)

# =========================
# Map 4: Simple Dynamic Map (two moving obstacles)
# =========================
def simple_dynamic():
    grid = np.zeros((20, 20))

    # simple obstacles - (x, y, w, h)
    static_obstacles = create_boundary_obstacles(20, 20) + []

    dynamic_obstacles = [
        DynamicObstacle(2, 3, 2, vel=(1, 0)),
    ]

    start = (2, 7)
    goals = [(18, 18)]

    return Map(grid, start, goals, "Simple Dynamic Map", static_obstacles, dynamic_obstacles)

# =========================
# Map 5: Hard Dynamic Map (multiple moving obstacles)
# =========================
def hard_dynamic():
    grid = np.zeros((20, 20))

    # simple obstacles - (x, y, w, h)
    static_obstacles = create_boundary_obstacles(20, 20) + [
        # central wall (split for gaps)
        StaticObstacle(9, 4, 2, 3),
        StaticObstacle(9, 9, 2, 2),

        # clutter
        StaticObstacle(3, 2, 3, 2),
        StaticObstacle(14, 5, 3, 2),
        StaticObstacle(3, 10, 2, 2),
        StaticObstacle(5, 14, 3, 3),
        StaticObstacle(12, 15, 3, 3),
        StaticObstacle(16, 12, 3, 2),
    ]

    dynamic_obstacles = [
        DynamicObstacle(1, 5, 2, vel=(0, 1)),
        DynamicObstacle(9, 15, 2, vel=(0, -1)),
        DynamicObstacle(11, 10, 2, vel=(1, 0)),
        DynamicObstacle(14, 7, 2, vel=(-1, 0)),
        DynamicObstacle(12, 12, 2, vel=(0, 1)),
    ]

    start = (1, 5)
    goals = [(18, 18)]

    return Map(grid, start, goals, "Hard Dynamic Map", static_obstacles, dynamic_obstacles)

# =========================
# Map loader
# =========================
MAPS = {
    "map1": simple,
    "map2": narrow_passage,
    "map3": multi_passage,
    "map4": simple_dynamic,
    "map5": hard_dynamic,
}


def get_map(name):
    if name not in MAPS:
        raise ValueError(f"Map '{name}' not found")
    return MAPS[name]()


def grid_to_obstacles(grid):
    """Convert a numpy grid to a list of (x, y, w, h) obstacle rectangles (one per cell)."""
    obstacles = []
    rows, cols = grid.shape
    for r in range(rows):
        for c in range(cols):
            if grid[r, c] == 1:
                obstacles.append((c, r, 1, 1))  # (x, y, w, h)
    return obstacles