'''
AER1516 - Project
Main file for the project. This file will contain the main loop and logic for running the pathfinding algorithms on the defined maps. 
It will also handle the visualization of the maps and the paths found by the algorithms
'''

import matplotlib.pyplot as plt
from maps import get_map
from dynamics import RobotDynamics, State
from path_planner import plan_rrt, plan_rrt_star
from visualization import plot_final_path, render_planning_step, animate_execution
from config import RRT_STAR_VIZ_INTERVAL

def reset_all_dynamic_obstacles(dynamic_obstacles):
    for obs in dynamic_obstacles:
        obs.reset_dynamic_obstacle()

def RRT_tester(map_name):
    m = get_map(map_name)
    dynamics_model = RobotDynamics()
    dynamics_model.grid = m.grid
    dynamics_model.static_obstacles = m.static_obstacles
    dynamics_model.dynamic_obstacles = m.dynamic_obstacles

    start = State(m.start[1], m.start[0], 0)
    goal = State(m.goals[0][1], m.goals[0][0], 0)
    print(f"Start: {start}  Goal: {goal}")

    path = plan_rrt(start=start, goal=goal, map_info=m, dynamics_model=dynamics_model,
                    step_size=0.5, max_iterations=3000, goal_threshold=1.0)

    if path is None:
        print("No path found.")
        return

    print(f"Path length: {len(path)}")

    fig, ax = plt.subplots()
    plot_final_path(ax, path, m, dynamics_model, start, goal, title="RRT Final Path")
    plt.pause(0.1)

    full_traj = dynamics_model.simulate_trajectory(path)
    reset_all_dynamic_obstacles(dynamics_model.dynamic_obstacles)
    animate_execution(full_traj, dynamics_model, m, start, goal)


def RRT_star_tester(map_name):
    m = get_map(map_name)
    dynamics_model = RobotDynamics()
    dynamics_model.grid = m.grid
    dynamics_model.static_obstacles = m.static_obstacles
    dynamics_model.dynamic_obstacles = m.dynamic_obstacles

    start = State(m.start[1], m.start[0], 0)
    goal = State(m.goals[0][1], m.goals[0][0], 0)
    print(f"Start: {start}  Goal: {goal}")

    plt.ion()
    fig, ax = plt.subplots()

    def on_viz(payload):
        render_planning_step(ax, payload, m, dynamics_model, start, goal)
        plt.pause(0.01)

    path = plan_rrt_star(start=start, goal=goal, map_info=m, dynamics_model=dynamics_model,
                         step_size=0.5, max_iterations=3000, goal_threshold=1.0,
                         viz_callback=on_viz, viz_interval=RRT_STAR_VIZ_INTERVAL)

    plt.ioff()

    if path is None:
        print("No path found.")
        return

    print(f"Path length: {len(path)}")

    fig, ax = plt.subplots()
    plot_final_path(ax, path, m, dynamics_model, start, goal, title="RRT* Final Path")
    plt.pause(0.1)

    full_traj = dynamics_model.simulate_trajectory(path)
    reset_all_dynamic_obstacles(dynamics_model.dynamic_obstacles)
    animate_execution(full_traj, dynamics_model, m, start, goal)

# =========================
# RUN HERE
# =========================
if __name__ == "__main__":
    # choose map: "map4" or "map5"
    # run_simulation("map1")
    # RRT_tester("map2")
    RRT_star_tester("map1")