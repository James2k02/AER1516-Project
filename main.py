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
from utils import update_obstacles            
            
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