"""
AER1516 — Benchmark Runner
Runs each planner on each applicable map for N_RUNS trials (headless, no GUI).
Outputs a timestamped CSV to results/ and prints a summary table.

Usage:
  python benchmark.py              # full run (150 trials)
  python benchmark.py --dry-run    # 1 trial per combination (quick smoke-test)
  python benchmark.py --runs 3     # custom number of runs
"""

import matplotlib
matplotlib.use('Agg')   # headless — must be set before any other matplotlib import

import argparse
import csv
import math
import os
import random
import time
from datetime import datetime

import numpy as np

from maps import get_map
from dynamics import RobotDynamics, State
from path_planner import plan_multi_goal
from utils import update_obstacles

# ---------------------------------------------------------------------------
# Trial matrix  — all algorithms run all 5 maps
# Static maps (1-3): no dynamic obstacles, execution trivially safe
# Dynamic maps (4-5):
#   rrt / rrt_star: plan with obstacles frozen, then execute live → collision = FAIL
#   rrt_fnd:        execution is built into the planner (handles moving obstacles)
# ---------------------------------------------------------------------------
TRIAL_MATRIX = {
    'rrt':      ['map1', 'map2', 'map3', 'map4', 'map5'],
    'rrt_star': ['map1', 'map2', 'map3', 'map4', 'map5'],
    'rrt_fnd':  ['map1', 'map2', 'map3', 'map4', 'map5'],
}

N_RUNS = 10
# One fixed seed per trial — different so variance is real, fixed so results are reproducible.
TRIAL_SEEDS = [42, 123, 456, 789, 1011, 1314, 2048, 3141, 4096, 9999]
RESULTS_DIR = os.path.join(os.path.dirname(__file__), 'results')

CSV_COLUMNS = [
    'algorithm', 'map', 'trial', 'seed',
    'success', 'execution_collision',
    'planning_time_s', 'total_time_s', 'fnd_overhead_s',
    'iterations', 'path_length_m', 'tree_nodes',
    'goals_reached', 'goals_total',
    'fnd_repairs', 'fnd_reconnects', 'fnd_regrows',
]


# ---------------------------------------------------------------------------
# Setup helper (mirrors main.py _setup, but never touches matplotlib)
# ---------------------------------------------------------------------------
def _setup(map_name):
    m = get_map(map_name)
    dynamics_model = RobotDynamics()
    dynamics_model.grid = m.grid
    dynamics_model.static_obstacles = m.static_obstacles
    dynamics_model.dynamic_obstacles = m.dynamic_obstacles

    start = State(m.start[1], m.start[0], m.start[2])
    goals = [State(g[1], g[0], g[2]) for g in m.goals]
    return m, dynamics_model, start, goals


def _reset_dynamic_obstacles(map_info):
    for obs in map_info.dynamic_obstacles:
        obs.reset_dynamic_obstacle()


# ---------------------------------------------------------------------------
# Execution simulation for RRT / RRT* on dynamic maps
# ---------------------------------------------------------------------------
def simulate_execution(path, map_info, dynamics_model) -> bool:
    """
    Walk a planned path while advancing dynamic obstacles one step per waypoint.
    Returns True (no collision) or False (collision detected).

    One update_obstacles() call per waypoint = 0.1 s of obstacle time, matching
    the rate used inside plan_rrt_star_fnd's execution loop.
    Obstacles must already be at their initial positions when this is called
    (ensured by _reset_dynamic_obstacles at the top of run_trial).
    """
    radius = dynamics_model.robot_radius
    for waypoint in path:
        update_obstacles(map_info.dynamic_obstacles, map_info.grid)
        for obs in map_info.dynamic_obstacles:
            if obs.collides_with_point(waypoint.x, waypoint.y, radius):
                return False
    return True


# ---------------------------------------------------------------------------
# Single trial
# ---------------------------------------------------------------------------
def run_trial(algorithm: str, map_name: str, trial_idx: int) -> dict:
    seed = TRIAL_SEEDS[trial_idx - 1]
    random.seed(seed)
    np.random.seed(seed)

    m, dynamics_model, start, goals = _setup(map_name)
    _reset_dynamic_obstacles(m)

    metrics: dict = {}
    wall_start = time.time()

    path = plan_multi_goal(
        start, goals, m, dynamics_model,
        planner=algorithm,
        metrics_out=metrics,
        freeze_obstacles=False,
        viz_callback=None,      # no rendering
    )

    wall_total = time.time() - wall_start
    success = path is not None
    execution_collision = False

    # For RRT / RRT* on dynamic maps: execute the planned path with live obstacles.
    # Planning is always done with frozen obstacles (no update_obstacles calls inside
    # plan_rrt / plan_rrt_star). Execution here reveals whether the static plan
    # survives a dynamic environment.
    if success and algorithm in ('rrt', 'rrt_star') and m.dynamic_obstacles:
        if not simulate_execution(path, m, dynamics_model):
            execution_collision = True
            success = False

    # fnd_overhead_s: extra time FND spent on execution + repairs beyond initial planning
    planning_time_s = metrics.get('planning_time_s', 0.0)
    total_time_s = metrics.get('total_time_s', wall_total)
    fnd_overhead_s = total_time_s - planning_time_s if algorithm == 'rrt_fnd' else 0.0

    row = {
        'algorithm':          algorithm,
        'map':                map_name,
        'trial':              trial_idx,
        'seed':               seed,
        'success':            int(success),
        'execution_collision': int(execution_collision),
        'planning_time_s':    round(planning_time_s, 4),
        'total_time_s':       round(total_time_s, 4),
        'fnd_overhead_s':     round(fnd_overhead_s, 4),
        'iterations':         metrics.get('iterations', 0),
        'path_length_m':      round(metrics.get('path_length_m', 0.0), 4),
        'tree_nodes':         metrics.get('tree_nodes', 0),
        'goals_reached':      metrics.get('goals_reached', 0),
        'goals_total':        metrics.get('goals_total', len(goals)),
        'fnd_repairs':        metrics.get('fnd_repairs', 0),
        'fnd_reconnects':     metrics.get('fnd_reconnects', 0),
        'fnd_regrows':        metrics.get('fnd_regrows', 0),
    }
    return row


# ---------------------------------------------------------------------------
# Summary table
# ---------------------------------------------------------------------------
def _mean_std(values):
    # Filter out non-numeric entries (e.g. '' from error rows)
    numeric = []
    for v in values:
        try:
            f = float(v)
            if not math.isnan(f):
                numeric.append(f)
        except (TypeError, ValueError):
            pass
    if not numeric:
        return float('nan'), float('nan')
    n = len(numeric)
    mu = sum(numeric) / n
    var = sum((v - mu) ** 2 for v in numeric) / n
    return mu, math.sqrt(var)


def print_summary(rows):
    # Group by (algorithm, map)
    groups: dict = {}
    for r in rows:
        key = (r['algorithm'], r['map'])
        groups.setdefault(key, []).append(r)

    header = (
        f"{'Algorithm':<12} {'Map':<6} "
        f"{'Success':>9} {'Plan(s) μ±σ':>16} {'Overhead(s) μ':>14} "
        f"{'PathLen(m) μ':>13} {'Iters μ':>9} {'Repairs μ':>10}"
    )
    print('\n' + '=' * len(header))
    print(header)
    print('=' * len(header))

    for key in sorted(groups.keys()):
        algo, map_name = key
        grp = groups[key]
        n = len(grp)

        successes = sum(int(r['success']) for r in grp)
        plan_mu, plan_sd = _mean_std([r['planning_time_s'] for r in grp])
        overhead_mu, _ = _mean_std([r['fnd_overhead_s'] for r in grp])
        pathlen_mu, _ = _mean_std([r['path_length_m'] for r in grp])
        iters_mu, _ = _mean_std([r['iterations'] for r in grp])
        repairs_mu, _ = _mean_std([r['fnd_repairs'] for r in grp])

        print(
            f"{algo:<12} {map_name:<6} "
            f"{successes:>4}/{n:<4} "
            f"{plan_mu:>7.2f}±{plan_sd:<6.2f} "
            f"{overhead_mu:>13.2f} "
            f"{pathlen_mu:>12.2f} "
            f"{iters_mu:>8.0f} "
            f"{repairs_mu:>9.1f}"
        )
    print('=' * len(header) + '\n')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='AER1516 benchmark runner')
    parser.add_argument('--runs', type=int, default=N_RUNS,
                        help=f'Trials per combination (default {N_RUNS})')
    parser.add_argument('--dry-run', action='store_true',
                        help='Run 1 trial per combination as a smoke-test')
    args = parser.parse_args()

    n_runs = 1 if args.dry_run else args.runs

    os.makedirs(RESULTS_DIR, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(RESULTS_DIR, f'benchmark_{timestamp}.csv')

    total = sum(len(maps) for maps in TRIAL_MATRIX.values()) * n_runs
    done = 0
    all_rows = []

    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        writer.writeheader()

        for algorithm, map_list in TRIAL_MATRIX.items():
            for map_name in map_list:
                for trial in range(1, n_runs + 1):
                    done += 1
                    print(f'[{done}/{total}] {algorithm} / {map_name} / trial {trial}')

                    try:
                        row = run_trial(algorithm, map_name, trial)
                    except Exception as e:
                        print(f'  ERROR: {e}')
                        row = {c: '' for c in CSV_COLUMNS}
                        row.update({'algorithm': algorithm, 'map': map_name,
                                    'trial': trial, 'seed': TRIAL_SEEDS[trial - 1],
                                    'success': 0, 'execution_collision': 0})

                    status = 'OK' if row.get('success') else 'FAIL'
                    ec = ' [exec collision]' if row.get('execution_collision') else ''
                    print(f'  {status}{ec}  plan={row.get("planning_time_s","?")}s  '
                          f'path={row.get("path_length_m","?")}m  '
                          f'iters={row.get("iterations","?")}')

                    all_rows.append(row)
                    writer.writerow(row)
                    f.flush()

    print(f'\nResults saved to {csv_path}')
    print_summary(all_rows)


if __name__ == '__main__':
    main()
