'''
AER1516 - Project
Main file for the project. This file will contain the main loop and logic for running the pathfinding algorithms on the defined maps. 
It will also handle the visualization of the maps and the paths found by the algorithms
'''

import matplotlib.pyplot as plt
from maps import get_map
from dynamics import RobotDynamics
from dynamics import State
from path_planner import plan_rrt

# =========================
# Checking if a position is valid for the obstacle to move into (no walls, within bounds)
# =========================
def is_valid_position(grid, r, c, size):
    for i in range(size):
        for j in range(size):
            rr = r + i
            cc = c + j

            # out of bounds
            if rr < 0 or rr >= grid.shape[0] or cc < 0 or cc >= grid.shape[1]:
                return False

            # hit wall
            if grid[rr, cc] == 1:
                return False

    return True

# =========================
# Update dynamic obstacles
# =========================
def update_obstacles(obstacles, grid):
    for obs in obstacles:
        r, c = obs["pos"]
        vr, vc = obs["vel"]
        size = obs["size"]

        # try move
        r_new = r + vr
        c_new = c + vc

        # check if move is valid
        if is_valid_position(grid, r_new, c_new, size):
            obs["pos"] = [r_new, c_new]
        else:
            # bounce if blocked
            obs["vel"][0] *= -1
            obs["vel"][1] *= -1
            
            
# =========================
# MAIN FUNCTIONS
# =========================

def RRT_tester(map_name):
    # =========================
    # Load map
    # =========================
    m = get_map(map_name)
    grid = m.grid

    # =========================
    # Setup dynamics
    # =========================
    dynamics_model = RobotDynamics()
    dynamics_model.grid = grid

    dynamics_model.static_obstacles = m.static_obstacles
    dynamics_model.dynamic_obstacles = m.dynamic_obstacles
    print("STATIC OBSTACLES:", dynamics_model.static_obstacles)
    print("COUNT:", len(dynamics_model.static_obstacles))

    # =========================
    # Convert start/goal to State
    # =========================
    # NOTE: (row, col) → (x, y)
    start = State(m.start[1], m.start[0], 0)
    goal = State(m.goals[0][1], m.goals[0][0], 0)

    print(f"Start: {start}")
    print(f"Goal: {goal}")

    # =========================
    # Visualization setup
    # =========================
    plt.ion()
    fig, ax = plt.subplots()

    # =========================
    # Run RRT
    # =========================
    path = plan_rrt(
        start=start,
        goal=goal,
        map_info=m,
        dynamics_model=dynamics_model,
        ax=ax,
        step_size = 0.5,
        max_iterations=3000,
        goal_threshold=1.0
    )

    # =========================
    # Final display
    # =========================
    plt.ioff()

    if path is None:
        print("No path found.")
    else:
        print(f"Path length: {len(path)}")

    plt.show()

# =========================
# RUN HERE
# =========================
if __name__ == "__main__":
    # choose map: "map4" or "map5"
    # run_simulation("map1")
    RRT_tester("map4")