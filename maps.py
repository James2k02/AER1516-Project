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
        self.dimensions = grid.shape # outputs (rows, cols) --> (height, width)
        self.start = start
        self.goals = goals
        self.name = name

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
            self.x - radius - 0.05 <= px <= self.x + self.w + radius + 0.05 and
            self.y - radius - 0.05 <= py <= self.y + self.h + radius + 0.05
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
        self.initial_vel = list(vel)

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
        # Check full extent of obstacle against grid bounds
        if x < 0 or y < 0 or x + self.w > grid.shape[1] or y + self.h > grid.shape[0]:
            return False

        for i in range(int(self.h)):
            for j in range(int(self.w)):
                r = int(y + i)
                c = int(x + j)

                if grid[r, c] == 1:
                    return False

        return True

    def get_position_at_time(self, t, grid, dt = 0.1):
        """
        Predict position at time t using SAME bounce dynamics as update().

        Returns:
            A NEW DynamicObstacle at predicted position
        """
        from copy import deepcopy

        obs_copy = deepcopy(self)
        obs_copy.reset_dynamic_obstacle()

        time_elapsed = 0.0

        while time_elapsed < t:
            obs_copy.update(grid)
            time_elapsed += dt

        return obs_copy
    
    def reset_dynamic_obstacle(self):
        """
        Reset obstacle to its initial position and velocity.
        """
        self.x = self.initial_pos[0]
        self.y = self.initial_pos[1]
        self.vel = list(self.initial_vel)

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

    start = (2.5, 2.5, np.pi/4) # (x, y, theta)
    goals = [(18.5, 18.5,0.0), (16.5, 5.5,0.0)]  # list of goals, can add more later

    return Map(grid, start, goals, "Simple Map", static_obstacles, dynamic_obstacles)

# =========================
# Map 2: Static Map (narrow passage)
# =========================
def narrow_passage():
    grid = np.zeros((20, 20))

    # simple obstacles - (x, y, w, h)
    static_obstacles = create_boundary_obstacles(20, 20) + [
        # wall 1 
        StaticObstacle(0, 4, 14, 2),
        StaticObstacle(16, 4, 4, 2),

        # wall 2 
        # StaticObstacle(0, 9, 20, 1), --> add back if you want to try jumping (remove the two lines below)
        StaticObstacle(0, 9, 5, 1),
        StaticObstacle(7, 9, 13, 1),

        # wall 3 
        StaticObstacle(0, 13, 5, 2),
        StaticObstacle(7, 13, 13, 2),
    ]

    dynamic_obstacles = []

    start = (18.5, 18.5, np.pi)
    goals = [(2.5, 9.5,np.pi), (11.5, 15.5, 0.0)]

    return Map(grid, start, goals, "Narrow Passage", static_obstacles, dynamic_obstacles)

# =========================
# Map 3: Static Map (multi-passage)
# =========================
def multi_passage():
    grid = np.zeros((20, 20))

    # simple obstacles - (x, y, w, h)
    static_obstacles = create_boundary_obstacles(20, 20) + [
        # y = 4
        StaticObstacle(0, 4, 2, 1),
        StaticObstacle(4, 4, 9, 1),
        StaticObstacle(15, 4, 7, 1),

        # y = 8
        StaticObstacle(0, 8, 5, 1),
        StaticObstacle(7, 8, 8, 1),
        StaticObstacle(17, 8, 3, 1),

        # y = 12
        StaticObstacle(0, 12, 2, 1),
        StaticObstacle(4, 12, 2, 1),
        StaticObstacle(8, 12, 3, 1),
        StaticObstacle(13, 12, 6, 1),

        # y = 16
        StaticObstacle(0, 16, 1, 1),
        StaticObstacle(3, 16, 5, 1),
        StaticObstacle(8, 16, 6, 1),
        StaticObstacle(16, 16, 3, 1),
    ]

    dynamic_obstacles = []

    start = (2.5, 9.5, 0.0)
    goals = [(18.5, 8.5,0.0)]

    return Map(grid, start, goals, "Multi Route Map", static_obstacles, dynamic_obstacles)

# =========================
# Map 4: Simple Dynamic Map (two moving obstacles)
# =========================
def simple_dynamic():
    grid = np.zeros((20, 20))

    # simple obstacles - (x, y, w, h)
    static_obstacles = create_boundary_obstacles(20, 20) + []

    dynamic_obstacles = [
        DynamicObstacle(2, 2, 2, vel=(0.25, 0)), # (x, y, theta, velocity)
        #DynamicObstacle(10, 2, 2, vel=(-0.5, 0)),
        DynamicObstacle(10, 6, 2, vel=(-0.25, 0)),
        #DynamicObstacle(14, 6, 2, vel=(-0.5, 0)),
        DynamicObstacle(2, 10, 2, vel=(0.25, 0)),
        #DynamicObstacle(10, 10, 2, vel=(-0.5, 0)),
        DynamicObstacle(10, 14, 2, vel=(-0.25, 0)),
        #DynamicObstacle(14, 14, 2, vel=(-0.5, 0))
    ]

    start = (18.5, 12.5, (-np.pi/2))
    goals = [(1.5, 12.5,-np.pi/4), (9.5, 5.5,0.0)] # (5.5, 16.5)

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
        StaticObstacle(12, 15, 2, 2),
        StaticObstacle(16, 12, 2, 2),
    ]

    dynamic_obstacles = [
        DynamicObstacle(1, 5, 2, vel=(0, 0.1)),
        # DynamicObstacle(9, 15, 2, vel=(0, -0.1)),
        DynamicObstacle(11, 10, 2, vel=(0.1, 0)),
        DynamicObstacle(14, 7, 2, vel=(-0.1, 0)),
        # DynamicObstacle(12, 12, 2, vel=(0, 0.1)),
    ]

    start = (2.5, 8.5, np.pi/4)
    goals = [(18.5, 18.5,0.0)]

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