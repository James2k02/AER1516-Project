'''
AER1516 - Project
File that contains the map definitions for the project. Each map is represented as a grid with obstacles, a start point, and a goal point. 
The maps can be easily modified or extended by changing the grid layout and obstacle placements
'''

import numpy as np

# =========================
# Helper: add walls
# =========================
def add_boundaries(grid):
    grid[0, :] = 1 # first row all columns
    grid[-1, :] = 1 # last row all columns
    grid[:, 0] = 1 # first column all rows
    grid[:, -1] = 1 # last column all rows
    return grid

# =========================
# Map 1: Static Map (simple)
# =========================
def simple():
    grid = np.zeros((20, 10))
    grid = add_boundaries(grid)

    # simple obstacles
    grid[6:10, 5:8] = 1
    grid[12:15, 2:4] = 1

    start = (1, 1)
    goal = (18, 8)

    return grid, start, goal, {"name": "Simple Map"}

# =========================
# Map 2: Static Map (narrow passage)
# =========================
def narrow_passage():
    grid = np.zeros((20, 10))
    grid = add_boundaries(grid)

    # horizontal wall 1
    grid[4:6, :] = 1
    
    # narrow gap in wall 1
    grid[4:6, 4:5] = 0
    
    # horizontal wall 2
    grid[9:10, :] = 1
    
    # horizontal wall 3
    grid[13:15, :] = 1
    
    # narrow gap in wall 3
    grid[13:15, 6:7] = 0

    start = (2, 2)
    goal = (17, 7)

    return grid, start, goal, {"name": "Narrow Passage Map w/ Jump"}

# =========================
# Map 3: Static Map (multi-passage)
# =========================
def multi_passage():
    grid = np.zeros((20, 10))
    grid = add_boundaries(grid)

    # horizontal barriers
    grid[4:5, 1:9] = 1
    grid[8:9, 1:9] = 1
    grid[12:13, 1:9] = 1
    grid[16:17, 1:9] = 1

    # openings (different routes)
    grid[4:5, 2:3] = 0
    grid[8:9, 6:7] = 0
    grid[12:13, 3:4] = 0
    grid[16:17, 7:8] = 0

    start = (1, 1)
    goal = (18, 8)

    return grid, start, goal, {"name": "Multi Route Map"}

# =========================
# Map 4: Simple Dynamic Map (one moving obstacle)
# =========================
def simple_dynamic():
    import numpy as np

    grid = np.zeros((20, 10))

    # boundaries
    grid[0, :] = 1
    grid[-1, :] = 1
    grid[:, 0] = 1
    grid[:, -1] = 1

    start = (2, 1)
    goal = (17, 8)

    return grid, start, goal, {"name": "Simple Dynamic Map"}

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
    goal = (18, 18)

    return grid, start, goal, {"name": "Hard Dynamic Map"}

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