'''
AER1516 - Project
File that contains the map definitions for the project. Each map is represented as a grid with obstacles, a start point, and a goal point. 
The maps can be easily modified or extended by changing the grid layout and obstacle placements
'''

from matplotlib.pyplot import grid
import numpy as np

# =========================
# Map class
# =========================
class Map:
    def __init__(self, grid, start, goals, name): # goals is a list of (x, y) tuples
        self.grid = grid
        self.dimensions = grid.shape # outputs (rows, cols) --> (height, width)
        self.start = start
        self.goals = goals
        self.name = name
        self.dynamic_obstacles = []

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

    start = (2, 9)
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

    start = (2, 7)
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