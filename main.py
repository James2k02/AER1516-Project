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
from path_planner import grid_to_obstacles
from path_planner import rrt_step
from path_planner import visualize_rrt
from path_planner import extract_path
from path_planner import RRTTree

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

def RRT_dynamic_test(map_name):
    m = get_map(map_name)
    grid = m.grid

    dynamics_model = RobotDynamics()
    dynamics_model.grid = grid
    dynamics_model.static_obstacles = grid_to_obstacles(grid)

    # Start/goal
    start = State(m.start[1], m.start[0], 0)
    goal = State(m.goals[0][1], m.goals[0][0], 0)

    # Dynamic obstacles (reuse your existing ones)
    dynamic_obstacles = [
        {"pos": [3, 2], "size": 2, "vel": [0, 1]},
        {"pos": [7, 12], "size": 2, "vel": [0, 1]},
        {"pos": [11, 8], "size": 2, "vel": [0, -1]},
        {"pos": [15, 5], "size": 2, "vel": [0, -1]}
    ]

    dynamics_model.dynamic_obstacles = dynamic_obstacles

    # RRT tree
    tree = RRTTree(start)

    plt.ion()
    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    goal_node = None

    for iteration in range(3000):

        # =========================
        # 1. Move obstacles
        # =========================
        update_obstacles(dynamic_obstacles, grid)

        # =========================
        # 2. Grow RRT ONE STEP
        # =========================
        new_node, reached = rrt_step(
            tree, goal, m, dynamics_model,
            step_size=1.0,
            goal_threshold=1.0,
            p_goal_bias=0.05
        )

        if reached:
            goal_node = new_node
            print("Goal reached!")
            break

        # =========================
        # 3. Draw everything
        # =========================
        ax.clear()

        temp_grid = grid.copy()

        # draw dynamic obstacles into grid
        for obs in dynamic_obstacles:
            r, c = obs["pos"]
            size = obs["size"]

            for i in range(size):
                for j in range(size):
                    temp_grid[r+i, c+j] = 1

        ax.imshow(1 - temp_grid, cmap='gray', origin='upper')

        # draw RRT tree
        visualize_rrt(ax, tree)

        # start/goal
        ax.scatter(start.x, start.y, c='green', s=100)
        ax.scatter(goal.x, goal.y, c='red', s=100)

        ax.set_title(f"Dynamic RRT (iter={iteration})")
        ax.set_xlim(0, grid.shape[1])
        ax.set_ylim(grid.shape[0], 0)

        plt.pause(0.01)

    # =========================
    # Final path
    # =========================
    if goal_node:
        path = extract_path(goal_node)

        ax.clear()
        ax.imshow(1 - grid, cmap='gray', origin='upper')
        visualize_rrt(ax, tree, path)
        ax.scatter(start.x, start.y, c='green', s=100)
        ax.scatter(goal.x, goal.y, c='red', s=100)
        plt.pause(1)

    plt.ioff()
    plt.show()

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

    # Convert grid → static obstacles (CRITICAL)
    dynamics_model.static_obstacles = grid_to_obstacles(grid)

    # No dynamic obstacles for map1
    dynamics_model.dynamic_obstacles = []

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
        step_size = 1.0,
        max_iterations = 50000,
        goal_threshold = 0.1
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


def run_simulation(map_name):
    # load map
    m = get_map(map_name)
    grid = m.grid

    dynamics_model = RobotDynamics()
    dynamics_model.grid = grid
    dynamics_model.static_obstacles = grid_to_obstacles(grid)

    start = State(m.start[1], m.start[0], 0)
    goal = State(m.goals[0][1], m.goals[0][0], 0)
    name = m.name

    plt.ion()
    fig, ax = plt.subplots()

    # =========================
    # Define dynamic obstacles
    # =========================
    if map_name == "map4":
        dynamic_obstacles = [
            {"initial_pos": [3, 2], "pos": [3, 2], "size": 2, "vel": [0, 1]},   # horizontal (right)
            {"initial_pos": [7, 12], "pos": [7, 12], "size": 2, "vel": [0, 1]},   # horizontal (right)
            {"initial_pos": [11, 8], "pos": [11, 8], "size": 2, "vel": [0, -1]},  # horizontal (left)
            {"initial_pos": [15, 5], "pos": [15, 5], "size": 2, "vel": [0, -1]}  # horizontal (left)
        ]

    elif map_name == "map5":
        dynamic_obstacles = [
            {"initial_pos": [5, 1], "pos": [5, 1], "size": 2, "vel": [1, 0]},    # vertical (down)
            {"initial_pos": [15, 9], "pos": [15, 9], "size": 2, "vel": [-1, 0]},  # vertical (up)
            {"initial_pos": [10, 11], "pos": [10, 11], "size": 2, "vel": [0, 1]},  # horizontal (right)
            {"initial_pos": [7, 14], "pos": [7, 14], "size": 2, "vel": [0, -1]},  # horizontal (left)
            {"initial_pos": [12, 12], "pos": [12, 12], "size": 2, "vel": [1, 0]},  # vertical (down)
        ]

    else:
        dynamic_obstacles = []

    dynamics_model.dynamic_obstacles = dynamic_obstacles

    # =========================
    # Visualization setup
    # =========================
    plt.ion()
    fig, ax = plt.subplots()

    while True:
        ax.clear()

        # =========================
        # 1. update dynamic obstacles
        # =========================
        update_obstacles(dynamic_obstacles, grid)

        # =========================
        # 2. create temp grid (for static + dynamic obstacles; does not modify original grid)
        # =========================
        temp_grid = grid.copy()

        # =========================
        # 3. stamp dynamic obstacles onto grid
        # =========================
        for obs in dynamic_obstacles:
            r, c = obs["pos"]
            size = obs["size"]

            for i in range(size):
                for j in range(size):
                    temp_grid[r + i, c + j] = 1

        # =========================
        # 4. draw map (ALL obstacles = black)
        # =========================
        ax.imshow(1 - temp_grid, cmap='gray', origin='upper')

        # =========================
        # 5. draw start + goals
        # =========================
        ax.scatter(start[1], start[0], c='green', s=120, label='Start')
        for goal in goals:
            ax.scatter(goal[1], goal[0], c='red', s=120, label='Goal')

        # =========================
        # 6. formatting
        # =========================
        ax.set_title(name)
        ax.set_xticks([])
        ax.set_yticks([])

        # clean legend
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())

        plt.pause(0.2)


# =========================
# RUN HERE
# =========================
if __name__ == "__main__":
    # choose map: "map4" or "map5"
    # run_simulation("map1")
    # RRT_tester("map4")
    RRT_dynamic_test("map4")