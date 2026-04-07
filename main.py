'''
AER1516 - Project
Main file for the project. This file will contain the main loop and logic for running the pathfinding algorithms on the defined maps. 
It will also handle the visualization of the maps and the paths found by the algorithms
'''

import matplotlib.pyplot as plt
from maps import get_map
import numpy as np
from matplotlib.patches import Circle
from dynamics import RobotDynamics
from dynamics import State
from path_planner import plan_rrt
from utils import update_obstacles

def reset_all_dynamic_obstacles(dynamic_obstacles):
    for obs in dynamic_obstacles:
        obs.reset_dynamic_obstacle()

# =========================
# MAIN FUNCTIONS
# =========================

def animate_execution(full_traj, dynamics_model, map_info, start, goal):
    """
    Animate robot following trajectory with moving obstacles.
    """

    grid = map_info.grid

    plt.ion()
    fig, ax = plt.subplots()

    for i in range(len(full_traj)):

        ax.clear()

        x, y, theta = full_traj[i]

        # =========================
        # UPDATE DYNAMIC OBSTACLES
        # =========================
        for obs in dynamics_model.dynamic_obstacles:
            obs.update(grid)

        # =========================
        # DRAW STATIC OBSTACLES
        # =========================
        for obs in dynamics_model.static_obstacles:
            rect = plt.Rectangle(
                (obs.x, obs.y),
                obs.w,
                obs.h,
                edgecolor='black',
                facecolor='black'
            )
            ax.add_patch(rect)

        # =========================
        # DRAW DYNAMIC OBSTACLES
        # =========================
        for obs in dynamics_model.dynamic_obstacles:
            if obs.collides_with_point(x, y, dynamics_model.robot_radius):
                print("COLLISION DETECTED")
            rect = plt.Rectangle(
                (obs.x, obs.y),
                obs.w,
                obs.h,
                edgecolor='red',
                facecolor='red'
            )
            ax.add_patch(rect)

        # =========================
        # DRAW ROBOT (circle + heading)
        # =========================
        radius = dynamics_model.robot_radius

        robot = Circle((x, y), radius=radius, color='blue')
        ax.add_patch(robot)

        # heading line
        hx = x + radius * np.cos(theta)
        hy = y + radius * np.sin(theta)
        ax.plot([x, hx], [y, hy], color='white', linewidth=2)

        # =========================
        # DRAW START / GOAL
        # =========================
        ax.scatter(start.x, start.y, c='green', s=100)
        ax.scatter(goal.x, goal.y, c='yellow', s=100)

        # =========================
        # SETTINGS
        # =========================
        ax.set_xlim(0, grid.shape[1])
        ax.set_ylim(grid.shape[0], 0)
        ax.set_title(f"Execution (step {i})")

        plt.pause(0.01)

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
    path = plan_rrt(start = start, goal = goal, map_info = m, dynamics_model = dynamics_model, ax = ax, step_size = 0.5, max_iterations = 3000, goal_threshold = 1.0)

    if path is None:
        print("No path found.")
    else:
        print(f"Path length: {len(path)}")

        full_traj = dynamics_model.simulate_trajectory(path)

        print(f"Simulated trajectory length: {len(full_traj)}")
        reset_all_dynamic_obstacles(dynamics_model.dynamic_obstacles)
        animate_execution(full_traj, dynamics_model, m, start, goal)

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