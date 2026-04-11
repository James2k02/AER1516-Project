"""
AER1516 - Project
Visualization module. All matplotlib drawing lives here — planners and
dynamics code should remain free of any visualization logic.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from utils import update_obstacles


# ============================================================================
# ACTIVATE-STYLE COLOR PALETTE
# ============================================================================

_BG_COLOR     = '#111111'   # dark floor between tiles
_TILE_FREE    = '#3a3a3a'   # grey tile (free space)
_TILE_GOAL    = '#00cc44'   # bright green (goal)
_TILE_STATIC  = '#cc2200'   # red (static obstacle)
_TILE_DYNAMIC = '#ff6600'   # orange (dynamic obstacle)
_TILE_START   = '#0088ff'   # blue (start position)
_TILE_PATH    = '#8800cc'   # purple (path cells)
_TILE_PAD     = 0.07        # fraction of cell used as gap between tiles


# ============================================================================
# PRIMITIVE DRAW HELPERS
# ============================================================================

def _draw_tile(ax, col, row, color, zorder=1):
    """Draw a single padded tile at grid cell (col, row)."""
    p = _TILE_PAD
    ax.add_patch(plt.Rectangle(
        (col + p, row + p), 1 - 2 * p, 1 - 2 * p,
        facecolor=color, edgecolor='none', zorder=zorder
    ))


def draw_grid_background(ax, grid):
    """Draw all cells as grey tiles on a dark background (Activate floor look)."""
    rows, cols = grid.shape
    ax.set_facecolor(_BG_COLOR)
    for r in range(rows):
        for c in range(cols):
            _draw_tile(ax, c, r, _TILE_FREE)


def draw_obstacles(ax, dynamics_model):
    """Draw static (red) and dynamic (orange) obstacle tiles."""
    for obs in dynamics_model.static_obstacles:
        for r in range(int(obs.y), int(obs.y + obs.h)):
            for c in range(int(obs.x), int(obs.x + obs.w)):
                _draw_tile(ax, c, r, _TILE_STATIC, zorder=2)
    for obs in dynamics_model.dynamic_obstacles:
        for r in range(int(obs.y), int(obs.y + obs.h)):
            for c in range(int(obs.x), int(obs.x + obs.w)):
                _draw_tile(ax, c, r, _TILE_DYNAMIC, zorder=2)


def draw_goal_tiles(ax, goals):
    """Draw goal cells as bright green tiles."""
    for (gx, gy) in goals:
        _draw_tile(ax, int(gx), int(gy), _TILE_GOAL, zorder=3)


def draw_path_tiles(ax, path):
    """Color every grid cell the path passes through in purple."""
    cells = set()
    for i, s in enumerate(path):
        cells.add((int(s.x), int(s.y)))
        if i < len(path) - 1:
            x0, y0 = s.x, s.y
            x1, y1 = path[i + 1].x, path[i + 1].y
            steps = max(int(np.hypot(x1 - x0, y1 - y0) * 4), 1)
            for t in range(1, steps):
                frac = t / steps
                cells.add((int(x0 + frac * (x1 - x0)), int(y0 + frac * (y1 - y0))))
    for (c, r) in cells:
        _draw_tile(ax, c, r, _TILE_PATH, zorder=4)


def draw_start_goal(ax, start, goal):
    """Draw start (blue tile + marker) and goal (green tile + marker)."""
    _draw_tile(ax, int(start.x), int(start.y), _TILE_START, zorder=5)
    ax.scatter(start.x, start.y, c='white', s=60, zorder=7, marker='o', label='Start')
    _draw_tile(ax, int(goal.x), int(goal.y), _TILE_GOAL, zorder=5)
    ax.scatter(goal.x, goal.y, c='white', s=60, zorder=7, marker='*', label='Goal')


def draw_robot(ax, x, y, theta, radius):
    """Draw robot as a filled circle with a heading line."""
    robot = Circle((x, y), radius=radius, color='cyan', zorder=6)
    ax.add_patch(robot)
    hx = x + radius * np.cos(theta)
    hy = y + radius * np.sin(theta)
    ax.plot([x, hx], [y, hy], color='white', linewidth=2, zorder=7)


def _set_axes(ax, grid):
    ax.set_xlim(0, grid.shape[1])
    ax.set_ylim(grid.shape[0], 0)
    ax.set_xticks([])
    ax.set_yticks([])


# ============================================================================
# TREE / PATH DRAWING
# ============================================================================

def draw_rrt_tree(ax, tree, path=None):
    """Draw RRT tree edges (cyan) and optional final path (yellow)."""
    for node in tree.nodes:
        if node.parent is not None:
            ax.plot(
                [node.state.x, node.parent.state.x],
                [node.state.y, node.parent.state.y],
                color='cyan', linewidth=0.5, alpha=0.4, zorder=4
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
    update_obstacles(dynamics_model.dynamic_obstacles, map_info.grid)
    draw_grid_background(ax, map_info.grid)
    draw_goal_tiles(ax, map_info.goals)
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
        ax.scatter(current_node.x, current_node.y, c='cyan', s=80, zorder=5, label='Robot', edgecolors='white', linewidths=0.5)

    draw_start_goal(ax, start, goal)
    _set_axes(ax, map_info.grid)
    ax.figure.patch.set_facecolor('#222222')

    planner   = payload.get("planner", "Planner")
    phase     = payload.get("phase", "planning")
    iteration = payload.get("iteration", "?")
    ax.set_title(f"{planner} | {phase} | iter={iteration}", color='white')


def plot_final_path(ax, path, map_info, dynamics_model, start, goal, title="Final Path"):
    """Show the final planned path on a clean map."""
    ax.clear()
    draw_grid_background(ax, map_info.grid)
    draw_goal_tiles(ax, map_info.goals)
    draw_obstacles(ax, dynamics_model)

    if path:
        draw_path_tiles(ax, path)
        xs = [s.x for s in path]
        ys = [s.y for s in path]
        ax.plot(xs, ys, color='yellow', linewidth=2, label='Path', zorder=6)
        ax.scatter(xs, ys, c='yellow', s=15, zorder=6)

    draw_start_goal(ax, start, goal)
    _set_axes(ax, map_info.grid)
    ax.set_title(title, color='white')
    ax.set_facecolor(_BG_COLOR)
    ax.figure.patch.set_facecolor('#222222')
    ax.legend(facecolor='#333333', labelcolor='white')


# ============================================================================
# ANIMATION
# ============================================================================

def animate_execution(full_traj, dynamics_model, map_info, start, goal):
    """Animate robot following a simulated trajectory with moving obstacles."""
    grid = map_info.grid

    plt.ion()
    fig, ax = plt.subplots()
    fig.patch.set_facecolor('#222222')

    for i, (x, y, theta) in enumerate(full_traj):

        ax.clear()

        # Update and draw dynamic obstacles
        for obs in dynamics_model.dynamic_obstacles:
            obs.update(grid)
            if obs.collides_with_point(x, y, dynamics_model.robot_radius):
                print(f"COLLISION at step {i}")

        draw_grid_background(ax, grid)
        draw_goal_tiles(ax, map_info.goals)
        draw_obstacles(ax, dynamics_model)
        draw_robot(ax, x, y, theta, dynamics_model.robot_radius)
        draw_start_goal(ax, start, goal)

        _set_axes(ax, grid)
        ax.set_title(f"Execution (step {i})", color='white')

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
