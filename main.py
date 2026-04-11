'''
AER1516 - Project
Entry point for running path planners on defined maps.

Usage:
  python main.py --planner rrt_star --map map1
  python main.py --planner rrt     --map map3 --iters 5000 --step 0.3
  python main.py --help
'''

import argparse
import matplotlib.pyplot as plt
from maps import get_map
from dynamics import RobotDynamics, State
from path_planner import plan_multi_goal
from visualization import plot_final_path, render_planning_step, animate_path_execution
from config import RRT_VIZ_INTERVAL, GOAL_SUCCESS_THRESH


def _setup(map_name):
    """Load map and wire up the dynamics model. Returns (map, dynamics_model, start, goals)."""
    m = get_map(map_name)
    dynamics_model = RobotDynamics()
    dynamics_model.grid = m.grid
    dynamics_model.static_obstacles = m.static_obstacles
    dynamics_model.dynamic_obstacles = m.dynamic_obstacles
    start = State(m.start[1], m.start[0], 0)
    goals = [State(g[1], g[0], 0) for g in m.goals]
    return m, dynamics_model, start, goals


def _execute(path, dynamics_model, m, start, goals):
    """Simulate and animate the robot following the planned path."""
    for obs in dynamics_model.dynamic_obstacles:
        obs.reset_dynamic_obstacle()
    animate_path_execution(path, dynamics_model, m, start, goals[-1])


# ---------------------------------------------------------
# RRT tester
# ---------------------------------------------------------
def RRT_tester(map_name, max_iterations=3000, step_size=0.5, goal_threshold=GOAL_SUCCESS_THRESH):
    m, dynamics_model, start, goals = _setup(map_name)
    print(f"[RRT] map={map_name}  start={start}  goals={goals}")

    # Live tree visualization during planning
    plt.ion()
    _, ax = plt.subplots()

    def on_viz(payload):
        render_planning_step(ax, payload, m, dynamics_model, start, goals[-1])
        plt.pause(0.01)

    path = plan_multi_goal(
        start=start, goals=goals, map_info=m, dynamics_model=dynamics_model,
        planner="rrt",
        step_size=step_size, max_iterations=max_iterations, goal_threshold=goal_threshold,
        viz_callback=on_viz, viz_interval=RRT_VIZ_INTERVAL,
    )
    plt.ioff()

    if path is None:
        print("No path found.")
        return

    print(f"Path found — {len(path)} waypoints")
    plot_final_path(ax, path, m, dynamics_model, start, goals[-1], title="RRT Final Path")
    plt.pause(0.1)
    _execute(path, dynamics_model, m, start, goals)


# ---------------------------------------------------------
# RRT* tester
# ---------------------------------------------------------
def RRT_star_tester(map_name, max_iterations=3000, step_size=0.5, goal_threshold=GOAL_SUCCESS_THRESH):
    m, dynamics_model, start, goals = _setup(map_name)
    print(f"[RRT*] map={map_name}  start={start}  goals={goals}")

    # Live tree visualization during planning
    plt.ion()
    _, ax = plt.subplots()

    def on_viz(payload):
        render_planning_step(ax, payload, m, dynamics_model, start, goals[-1])
        plt.pause(0.01)

    path = plan_multi_goal(
        start=start, goals=goals, map_info=m, dynamics_model=dynamics_model,
        planner="rrt_star",
        step_size=step_size, max_iterations=max_iterations, goal_threshold=goal_threshold,
        viz_callback=on_viz, viz_interval=RRT_VIZ_INTERVAL,
    )
    plt.ioff()

    if path is None:
        print("No path found.")
        return

    print(f"Path found — {len(path)} waypoints")
    plot_final_path(ax, path, m, dynamics_model, start, goals[-1], title="RRT* Final Path")
    plt.pause(0.1)
    _execute(path, dynamics_model, m, start, goals)


# ---------------------------------------------------------
# RRT*-FND tester — scaffold ready, waiting on plan_rrt_star_fnd implementation
# ---------------------------------------------------------
def RRT_fnd_tester(map_name, max_iterations=3000, step_size=0.5, goal_threshold=GOAL_SUCCESS_THRESH):
    m, dynamics_model, start, goals = _setup(map_name)
    print(f"[RRT*-FND] map={map_name}  start={start}  goals={goals}")

    plt.ion()
    _, ax = plt.subplots()
    def on_viz(payload):
        render_planning_step(ax, payload, m, dynamics_model, start, goals[-1])
        plt.pause(0.01)

    path = plan_multi_goal(
        start=start, goals=goals, map_info=m, dynamics_model=dynamics_model,
        planner="rrt_fnd",
        step_size=step_size, max_iterations=max_iterations, goal_threshold=goal_threshold,
        viz_callback=on_viz, viz_interval=RRT_VIZ_INTERVAL,
    )

    _, ax = plt.subplots()

    if path is None:
        print("No path found.")
        return

    print(f"Path found — {len(path)} waypoints")
    plot_final_path(ax, path, m, dynamics_model, start, goals[-1], title="RRT*-FND Final Path")
    plt.pause(0.1)
    _execute(path, dynamics_model, m, start, goals)


# ---------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AER1516 Path Planner")
    parser.add_argument("--planner",   choices=["rrt", "rrt_star", "rrt_fnd"], default="rrt_star")
    parser.add_argument("--map",       choices=["map1", "map2", "map3", "map4", "map5"], default="map1")
    parser.add_argument("--iters",     type=int,   default=3000, help="Max iterations")
    parser.add_argument("--step",      type=float, default=0.5,  help="Step size")
    parser.add_argument("--threshold", type=float, default=GOAL_SUCCESS_THRESH,  help="Goal threshold")
    args = parser.parse_args()

    kwargs = dict(max_iterations=args.iters, step_size=args.step, goal_threshold=args.threshold)
    if args.planner == "rrt":
        RRT_tester(args.map, **kwargs)
    elif args.planner == "rrt_star":
        RRT_star_tester(args.map, **kwargs)
    elif args.planner == "rrt_fnd":
        RRT_fnd_tester(args.map, **kwargs)
