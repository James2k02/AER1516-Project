"""
AER1516 - Project
Visualization module. All matplotlib drawing lives here — planners and
dynamics code should remain free of any visualization logic.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


# ============================================================================
# PRIMITIVE DRAW HELPERS
# ============================================================================

def draw_obstacles(ax, dynamics_model):
    """Draw static (black) and dynamic (red) obstacles."""
    for obs in dynamics_model.static_obstacles:
        ax.add_patch(plt.Rectangle(
            (obs.x, obs.y), obs.w, obs.h,
            edgecolor='black', facecolor='black'
        ))
    for obs in dynamics_model.dynamic_obstacles:
        ax.add_patch(plt.Rectangle(
            (obs.x, obs.y), obs.w, obs.h,
            edgecolor='red', facecolor='red', alpha=0.6
        ))


def draw_start_goal(ax, start, goal):
    """Draw start (green) and goal (yellow) markers."""
    ax.scatter(start.x, start.y, c='green', s=100, zorder=5, label='Start')
    ax.scatter(goal.x, goal.y, c='yellow', s=100, zorder=5, label='Goal')


def draw_robot(ax, x, y, theta, radius):
    """Draw robot as a filled circle with a heading line."""
    robot = Circle((x, y), radius=radius, color='blue', zorder=6)
    ax.add_patch(robot)
    hx = x + radius * np.cos(theta)
    hy = y + radius * np.sin(theta)
    ax.plot([x, hx], [y, hy], color='white', linewidth=2, zorder=7)


def _set_axes(ax, grid):
    ax.set_xlim(0, grid.shape[1])
    ax.set_ylim(grid.shape[0], 0)


# ============================================================================
# TREE / PATH DRAWING
# ============================================================================

def draw_rrt_tree(ax, tree, path=None):
    """Draw RRT tree edges (blue) and optional final path (yellow)."""
    for node in tree.nodes:
        if node.parent is not None:
            ax.plot(
                [node.state.x, node.parent.state.x],
                [node.state.y, node.parent.state.y],
                color='blue', linewidth=0.5, alpha=0.6
            )
    if path is not None:
        xs = [s.x for s in path]
        ys = [s.y for s in path]
        ax.plot(xs, ys, color='yellow', linewidth=2, label='Path')


# ============================================================================
# FULL-FRAME PLOTS
# ============================================================================

def render_planning_step(ax, payload, map_info, dynamics_model, start, goal):
    """
    Draw a planning frame from a payload dict.
    Works for any tree-based planner (RRT, RRT*, RRT*-FND, BIT*, ...).

    Supported payload keys:
        planner      (str)         : planner name used in the title - RRT, RRT*, RRT*-FND, BIT*
        iteration    (int)         : current iteration number
        phase        (str)         : "planning" | "executing" | "repairing"
        tree                       : RRTTree — draws all edges if present
        current_path (list[State]) : highlighted best path found so far
        current_node (State)       : robot's current position (execution phase)
    """
    ax.clear()
    ax.set_facecolor('white')
    draw_obstacles(ax, dynamics_model)

    tree = payload.get("tree")
    if tree is not None:
        draw_rrt_tree(ax, tree)

    current_path = payload.get("current_path")
    if current_path is not None:
        xs = [s.x for s in current_path]
        ys = [s.y for s in current_path]
        ax.plot(xs, ys, color='orange', linewidth=2, zorder=4, label='Current Path')

    current_node = payload.get("current_node")
    if current_node is not None:
        ax.scatter(current_node.x, current_node.y, c='cyan', s=80, zorder=5, label='Robot')

    draw_start_goal(ax, start, goal)
    _set_axes(ax, map_info.grid)

    planner  = payload.get("planner", "Planner")
    phase    = payload.get("phase", "planning")
    iteration = payload.get("iteration", "?")
    ax.set_title(f"{planner} | {phase} | iter={iteration}")


def plot_final_path(ax, path, map_info, dynamics_model, start, goal, title="Final Path"):
    """Show the final planned path on a clean map."""
    ax.clear()
    draw_obstacles(ax, dynamics_model)

    if path:
        xs = [s.x for s in path]
        ys = [s.y for s in path]
        ax.plot(xs, ys, color='blue', linewidth=2, label='Path')
        ax.scatter(xs, ys, c='blue', s=10)

    draw_start_goal(ax, start, goal)
    _set_axes(ax, map_info.grid)
    ax.set_title(title)
    ax.legend()


# ============================================================================
# ANIMATION
# ============================================================================

def animate_execution(full_traj, dynamics_model, map_info, start, goal):
    """Animate robot following a simulated trajectory with moving obstacles."""
    grid = map_info.grid

    plt.ion()
    fig, ax = plt.subplots()

    for i, (x, y, theta) in enumerate(full_traj):

        ax.clear()

        # Update and draw dynamic obstacles
        for obs in dynamics_model.dynamic_obstacles:
            obs.update(grid)
            if obs.collides_with_point(x, y, dynamics_model.robot_radius):
                print(f"COLLISION at step {i}")

        draw_obstacles(ax, dynamics_model)
        draw_robot(ax, x, y, theta, dynamics_model.robot_radius)
        draw_start_goal(ax, start, goal)

        _set_axes(ax, grid)
        ax.set_title(f"Execution (step {i})")

        plt.pause(0.01)

    plt.ioff()
    plt.show()


# ============================================================================
# HOW TO USE THIS MODULE
# ============================================================================
#
# There are three places you need to hook into visualization:
#
#   1. LIVE PLANNING VIEW (inside the planner loop)
#   2. FINAL PATH VIEW    (after planning finishes)
#   3. EXECUTION ANIMATION (after simulating the trajectory)
#
# ─────────────────────────────────────────────────────────────────────────────
# 1. LIVE PLANNING VIEW
#
#    The planner calls viz_callback(payload) every viz_interval iterations.
#    You define the callback in main.py and pass it to the planner.
#    The payload is a plain dict — include only the keys that make sense for
#    your planner; everything else is optional and safely ignored.
#
#    Payload keys:
#        "planner"      (str)         name shown in the title, e.g. "RRT*"
#        "iteration"    (int)         current iteration count
#        "phase"        (str)         "planning" | "executing" | "repairing"
#        "tree"         (RRTTree)     draws the tree edges in blue
#        "current_path" (list[State]) draws the best path so far in orange
#        "current_node" (State)       draws the robot's current position in cyan
#
#    Example — minimal (RRT / RRT*):
#
#        plt.ion()
#        fig, ax = plt.subplots()
#
#        def on_viz(payload):
#            render_planning_step(ax, payload, map_info, dynamics_model, start, goal)
#            plt.pause(0.01)
#
#        path = plan_rrt_star(..., viz_callback=on_viz, viz_interval=20)
#
#    Example — extended (RRT*-FND, showing current path and robot position):
#
#        def on_viz(payload):
#            render_planning_step(ax, payload, map_info, dynamics_model, start, goal)
#            plt.pause(0.01)
#
#        # Inside plan_rrt_star_fnd, call:
#        viz_callback({
#            "planner":      "RRT*-FND",
#            "iteration":    iterations,
#            "phase":        "repairing",     # or "planning" / "executing"
#            "tree":         tree,
#            "current_path": sigma,           # current solution path
#            "current_node": p_current,       # robot's live position
#        })
#
# ─────────────────────────────────────────────────────────────────────────────
# 2. FINAL PATH VIEW
#
#    Call once after the planner returns, before animating.
#
#        fig, ax = plt.subplots()
#        plot_final_path(ax, path, map_info, dynamics_model, start, goal,
#                        title="RRT* Final Path")
#        plt.pause(0.1)
#
# ─────────────────────────────────────────────────────────────────────────────
# 3. EXECUTION ANIMATION
#
#    Call after simulating the trajectory with dynamics_model.simulate_trajectory().
#    Reset dynamic obstacles first so they start from their initial positions.
#
#        full_traj = dynamics_model.simulate_trajectory(path)
#        reset_all_dynamic_obstacles(dynamics_model.dynamic_obstacles)
#        animate_execution(full_traj, dynamics_model, map_info, start, goal)
#
