"""
AER1516 — Benchmark Runner
Runs each planner on each applicable map for N_RUNS trials (headless, no GUI).
Outputs a timestamped CSV to results/ and prints a summary table.

Usage:
  python benchmark.py              # full run (78 trials)
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

# ---------------------------------------------------------------------------
# Trial matrix
# Static maps: RRT, RRT*, RRT*-FND
# Dynamic maps: RRT* (frozen obstacles = baseline) and RRT*-FND (live obstacles)
# ---------------------------------------------------------------------------
TRIAL_MATRIX = {
    'rrt':      [('map1', False), ('map2', False), ('map3', False)],
    'rrt_star': [('map1', False), ('map2', False), ('map3', False),
                 ('map4', True),  ('map5', True)],   # True = freeze_obstacles
    'rrt_fnd':  [('map1', False), ('map2', False), ('map3', False),
                 ('map4', False), ('map5', False)],
}

N_RUNS = 6
# One fixed seed per trial — different so variance is real, fixed so results are reproducible.
TRIAL_SEEDS = [42, 110, 456, 789, 1011, 1314]
RESULTS_DIR = os.path.join(os.path.dirname(__file__), 'results')

CSV_COLUMNS = [
    'algorithm', 'map', 'freeze_obstacles', 'trial', 'seed',
    'success',
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
# Single trial
# ---------------------------------------------------------------------------
def run_trial(algorithm: str, map_name: str, freeze_obstacles: bool, trial_idx: int) -> dict:
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
        freeze_obstacles=freeze_obstacles,
        viz_callback=None,      # no rendering
    )

    wall_total = time.time() - wall_start
    success = path is not None

    # fnd_overhead_s only meaningful for FND; for RRT/RRT* it is always 0
    planning_time_s = metrics.get('planning_time_s', 0.0)
    total_time_s = metrics.get('total_time_s', wall_total)
    fnd_overhead_s = total_time_s - planning_time_s if algorithm == 'rrt_fnd' else 0.0

    row = {
        'algorithm':        algorithm,
        'map':              map_name,
        'freeze_obstacles': freeze_obstacles,
        'trial':            trial_idx,
        'seed':             seed,
        'success':          int(success),
        'planning_time_s':  round(planning_time_s, 4),
        'total_time_s':     round(total_time_s, 4),
        'fnd_overhead_s':   round(fnd_overhead_s, 4),
        'iterations':       metrics.get('iterations', 0),
        'path_length_m':    round(metrics.get('path_length_m', 0.0), 4),
        'tree_nodes':       metrics.get('tree_nodes', 0),
        'goals_reached':    metrics.get('goals_reached', 0),
        'goals_total':      metrics.get('goals_total', len(goals)),
        'fnd_repairs':      metrics.get('fnd_repairs', 0),
        'fnd_reconnects':   metrics.get('fnd_reconnects', 0),
        'fnd_regrows':      metrics.get('fnd_regrows', 0),
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
    # Group by (algorithm, map, freeze_obstacles)
    groups: dict = {}
    for r in rows:
        key = (r['algorithm'], r['map'], r['freeze_obstacles'])
        groups.setdefault(key, []).append(r)

    header = (
        f"{'Algorithm':<12} {'Map':<6} {'Frozen':<7} "
        f"{'Success':>8} {'Plan(s) μ±σ':>16} {'Overhead(s) μ':>14} "
        f"{'PathLen(m) μ':>13} {'Iters μ':>9} {'Repairs μ':>10}"
    )
    print('\n' + '=' * len(header))
    print(header)
    print('=' * len(header))

    for key in sorted(groups.keys()):
        algo, map_name, frozen = key
        grp = groups[key]
        n = len(grp)

        success_rate = sum(r['success'] for r in grp) / n
        plan_mu, plan_sd = _mean_std([r['planning_time_s'] for r in grp])
        overhead_mu, _ = _mean_std([r['fnd_overhead_s'] for r in grp])
        pathlen_mu, _ = _mean_std([r['path_length_m'] for r in grp])
        iters_mu, _ = _mean_std([r['iterations'] for r in grp])
        repairs_mu, _ = _mean_std([r['fnd_repairs'] for r in grp])

        print(
            f"{algo:<12} {map_name:<6} {str(frozen):<7} "
            f"{success_rate:>7.0%}  "
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
    parser = argparse.ArgumentParser(description='Benchmark runner')
    parser.add_argument('--runs', type=int, default=N_RUNS,
                        help=f'Trials per combination (default {N_RUNS})')
    parser.add_argument('--dry-run', action='store_true',
                        help='Run 1 trial per combination as a smoke-test')
    args = parser.parse_args()

    n_runs = 1 if args.dry_run else args.runs

    os.makedirs(RESULTS_DIR, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(RESULTS_DIR, f'benchmark_{timestamp}.csv')

    # Count total trials
    total = sum(len(combos) for combos in TRIAL_MATRIX.values()) * n_runs
    done = 0

    all_rows = []

    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        writer.writeheader()

        for algorithm, combos in TRIAL_MATRIX.items():
            for map_name, freeze in combos:
                for trial in range(1, n_runs + 1):
                    done += 1
                    label = f"{algorithm} / {map_name} / frozen={freeze} / trial {trial}"
                    print(f'[{done}/{total}] {label}')

                    try:
                        row = run_trial(algorithm, map_name, freeze, trial)
                    except Exception as e:
                        print(f'  ERROR: {e}')
                        row = {c: '' for c in CSV_COLUMNS}
                        row.update({'algorithm': algorithm, 'map': map_name,
                                    'freeze_obstacles': freeze, 'trial': trial,
                                    'success': 0})

                    status = 'OK' if row.get('success') else 'FAIL'
                    print(f'  {status}  plan={row.get("planning_time_s","?")}s  '
                          f'path={row.get("path_length_m","?")}m  '
                          f'iters={row.get("iterations","?")}')

                    all_rows.append(row)
                    writer.writerow(row)
                    f.flush()

    print(f'\nResults saved to {csv_path}')
    print_summary(all_rows)


if __name__ == '__main__':
    main()
