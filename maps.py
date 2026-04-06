'''
AER1516 - Project
File that contains the map definitions for the project. Each map is represented as a grid with obstacles, a start point, and a goal point. 
The maps can be easily modified or extended by changing the grid layout and obstacle placements
'''

from matplotlib.pyplot import grid
import numpy as np
from typing import List, Tuple, Optional
from config import CELL_SIZE
from scipy.ndimage import label


# =========================
# Map class
# =========================
class Map:
    def __init__(self, grid, start, goals, name, cell_size=CELL_SIZE): # goals is a list of (x, y) tuples
        self.grid = grid
        self.cell_size = cell_size
        self.dimensions = grid.shape # outputs (row dim, col dim) --> (height, width)
        self.start = start
        self.goals = goals
        self.name = name
    
    @property
    def width(self) -> float:
        """Width of the map in meters (columns)."""
        return self.dimensions[1] * self.cell_size

    @property
    def height(self) -> float:
        """Height of the map in meters (rows)."""
        return self.dimensions[0] * self.cell_size

    def get_obstacles_grid_position(self) -> list[tuple[int, int]]:
        """Return (row, col) of all obstacle cells."""
        rows, cols = np.nonzero(self.grid == 1)
        return list(zip(rows, cols))
    
    def meters_to_grid(self, x_m: float, y_m: float) -> tuple[int, int]:
        """
        Convert a position in meters to a grid cell (row, col).
        
        Args:
            x_m: x position in meters (column direction)
            y_m: y position in meters (row direction)
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
    

# =========================
# Helper: add walls
# =========================
def add_boundaries(grid):
    grid[0, :] = 1 # first row all columns
    grid[-1, :] = 1 # last row all columns
    grid[:, 0] = 1 # first column all rows
    grid[:, -1] = 1 # last column all rows
    return grid

'''
Note: the map is 20x20 but the interior is 18x18 becasue of the boundaries. A lot of Activate maps also have boundaries so you're not too close to the walls
'''

# =========================
# Map 1: Static Map (simple)
# =========================
def simple():
    grid = np.zeros((20, 20))
    grid = add_boundaries(grid)

    # simple obstacles
    grid[6:10, 5:8] = 1
    grid[12:15, 2:4] = 1
    grid[13:16, 11:14] = 1
    grid[2:5, 12:15] = 1
    grid[7:11, 15:17] = 1

    start = (2, 2)
    goals = [(18, 18), (16, 5)]  # list of goals, can add more later

    return Map(grid, start, goals, "Simple Map")

# =========================
# Map 2: Static Map (narrow passage)
# =========================
def narrow_passage():
    grid = np.zeros((20, 20))
    grid = add_boundaries(grid)

    # horizontal wall 1
    grid[4:6, :] = 1
    
    # narrow gap in wall 1
    grid[4:6, 15:16] = 0
    
    # horizontal wall 2
    grid[9:10, :] = 1
    
    # horizontal wall 3
    grid[13:15, :] = 1
    
    # narrow gap in wall 3
    grid[13:15, 6:7] = 0

    start = (2, 2)
    goals = [(17, 17)]

    return Map(grid, start, goals, "Narrow Passage Map w/ Jump")

# =========================
# Map 3: Static Map (multi-passage)
# =========================
def multi_passage():
    grid = np.zeros((20, 20))
    grid = add_boundaries(grid)

    # horizontal barriers
    grid[4:5, :] = 1
    grid[8:9, :] = 1
    grid[12:13, :] = 1
    grid[16:17, :] = 1

    # openings (different routes)
    grid[4:5, 2:3] = 0
    grid[4:5, 12:13] = 0
    
    grid[8:9, 6:7] = 0
    grid[8:9, 16:17] = 0
      
    grid[12:13, 3:4] = 0
    grid[12:13, 7:8] = 0
    grid[12:13, 12:13] = 0
    
    grid[16:17, 1:2] = 0
    grid[16:17, 7:8] = 0
    grid[16:17, 15:16] = 0

    start = (1, 9)
    goals = [(18, 8)]

    return Map(grid, start, goals, "Multi Route Map")

# =========================
# Map 4: Simple Dynamic Map (one moving obstacle)
# =========================
def simple_dynamic():
    import numpy as np

    grid = np.zeros((20, 20))

    # boundaries
    grid[0, :] = 1
    grid[-1, :] = 1
    grid[:, 0] = 1
    grid[:, -1] = 1

    start = (1, 7)
    goals = [(18, 18)]

    return Map(grid, start, goals, "Simple Dynamic Map")

# =========================
# Map 5: Hard Dynamic Map (multiple moving obstacles)
# =========================
def hard_dynamic():
    import numpy as np

    grid = np.zeros((20, 20))

    # boundaries
    grid[0, :] = 1
    grid[-1, :] = 1
    grid[:, 0] = 1
    grid[:, -1] = 1

    # central vertical wall
    grid[4:16, 9:11] = 1

    # gaps
    grid[7:9, 9:11] = 0
    grid[11:16, 9:11] = 0

    # more clutter blocks
    grid[2:4, 3:6] = 1
    grid[5:7, 14:17] = 1
    grid[10:12, 3:5] = 1
    grid[14:17, 5:8] = 1
    grid[15:18, 12:15] = 1
    grid[12:14, 16:19] = 1

    start = (1, 5)
    goals = [(18, 18)]

    return Map(grid, start, goals, "Hard Dynamic Map")

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