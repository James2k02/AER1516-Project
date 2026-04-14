# AER1516 — Motion Planning Project

This project implements and compares three sampling-based motion planning algorithms — **RRT**, **RRT\***, and **RRT\*-FND** — for a 2D robot navigating through static and dynamic obstacle environments. The planner supports multi-goal missions across five hand-crafted maps ranging from simple open spaces to narrow corridors and environments with moving obstacles. RRT\*-FND extends RRT\* with online failure detection and repair, allowing it to replan in real time as dynamic obstacles shift.

---

## Project Structure

```
AER1516-Project/
├── main.py            # Interactive entry point — runs planners with live visualization
├── benchmark.py       # Headless batch runner — evaluates planners across all maps
├── path_planner.py    # RRT, RRT*, and RRT*-FND implementations
├── maps.py            # Map definitions (map1–map5) and obstacle classes
├── dynamics.py        # Robot dynamics and state representation
├── visualization.py   # Plotting and animation utilities
├── config.py          # Tunable constants (step size, iteration limits, etc.)
├── utils.py           # Shared helpers
├── maps_img/          # Pre-rendered map images
├── plots/             # Output plots from main.py runs
└── results/           # Timestamped CSV files from benchmark runs
```

---

## Maps

| Map   | Name              | Type    | Description                                      |
|-------|-------------------|---------|--------------------------------------------------|
| map1  | Simple            | Static  | Open space with scattered rectangular obstacles  |
| map2  | Narrow Passage    | Static  | Horizontal walls with small gaps to thread       |
| map3  | Multi-Passage     | Static  | Four layers of walls, each with multiple gaps    |
| map4  | Simple Dynamic    | Dynamic | Open space with four slow-moving obstacles       |
| map5  | Hard Dynamic      | Dynamic | Static clutter plus three fast-moving obstacles  |

---

## Algorithms

| Planner    | Flag         | Description                                                      |
|------------|--------------|------------------------------------------------------------------|
| RRT        | `rrt`        | Rapidly-exploring Random Tree (baseline)                         |
| RRT\*      | `rrt_star`   | Asymptotically optimal RRT with rewiring                         |
| RRT\*-FND  | `rrt_fnd`    | RRT\* with online failure detection, reconnect, and regrow       |

---

## Installation

```bash
pip install -r requirements.txt
```

Python 3.10+ is recommended.

---

## Running a Planner — `main.py`

`main.py` is the interactive entry point. It runs the selected planner on a single map (or all maps) with live matplotlib visualization.

### Basic usage

```bash
python main.py --planner <planner> --map <map>
```

### Arguments

| Argument      | Choices / Type                          | Default    | Description                        |
|---------------|-----------------------------------------|------------|------------------------------------|
| `--planner`   | `rrt`, `rrt_star`, `rrt_fnd`            | `rrt_star` | Planning algorithm to use          |
| `--map`       | `map1`, `map2`, `map3`, `map4`, `map5` | `map1`     | Map to run on                      |
| `--iters`     | int                                     | `3000`     | Maximum planning iterations        |
| `--step`      | float                                   | `0.5`      | Step size for tree expansion       |
| `--threshold` | float                                   | `0.25`     | Goal-success distance threshold    |
| `--all_maps`  | flag                                    | off        | Run the planner on all five maps   |

### Examples

```bash
# RRT* on map1 (defaults)
python main.py

# RRT on map3 with custom iterations and step size
python main.py --planner rrt --map map3 --iters 5000 --step 0.3

# RRT*-FND on the hard dynamic map
python main.py --planner rrt_fnd --map map5

# Run RRT* across all five maps in sequence
python main.py --planner rrt_star --all_maps
```

Output plots are saved to `plots/` as `<planner>_<map>.png`.

---

## Running Benchmarks — `benchmark.py`

`benchmark.py` runs all three planners on all five maps for multiple randomised trials in headless mode (no GUI). Results are written to a timestamped CSV in `results/` and a summary table is printed to the console.

### Basic usage

```bash
python benchmark.py
```

### Arguments

| Argument    | Type  | Default | Description                                          |
|-------------|-------|---------|------------------------------------------------------|
| `--runs`    | int   | `10`    | Number of trials per planner/map combination         |
| `--dry-run` | flag  | off     | Run 1 trial per combination as a quick smoke-test    |

### Examples

```bash
# Full benchmark (10 trials × 3 planners × 5 maps = 150 trials)
python benchmark.py

# Quick smoke-test (1 trial per combination)
python benchmark.py --dry-run

# Custom number of trials
python benchmark.py --runs 5
```

### Output

Each run appends a timestamped CSV to `results/` (e.g. `results/benchmark_20260413_120000.csv`) with one row per trial and the following columns:

| Column                | Description                                              |
|-----------------------|----------------------------------------------------------|
| `algorithm`           | Planner name                                             |
| `map`                 | Map identifier                                           |
| `trial` / `seed`      | Trial index and random seed used                         |
| `success`             | 1 if a valid path was found and executed without collision|
| `execution_collision` | 1 if RRT/RRT* path collided with a dynamic obstacle      |
| `planning_time_s`     | Wall-clock time spent in the planner                     |
| `total_time_s`        | Total time including execution (FND only)                |
| `fnd_overhead_s`      | Extra time FND spent on repairs beyond initial planning  |
| `iterations`          | Number of tree-expansion iterations                      |
| `path_length_m`       | Total path length in metres                              |
| `tree_nodes`          | Number of nodes in the final tree                        |
| `goals_reached`       | Number of goals reached                                  |
| `fnd_repairs`         | FND repair events triggered                              |
| `fnd_reconnects`      | FND reconnect attempts                                   |
| `fnd_regrows`         | FND full-regrow attempts                                 |

A summary table is also printed to stdout showing mean/std planning time, path length, iterations, and repairs grouped by algorithm and map.

---

## Configuration

Key parameters can be adjusted in [config.py](config.py):

| Parameter             | Default  | Description                                      |
|-----------------------|----------|--------------------------------------------------|
| `GOAL_SUCCESS_THRESH` | `0.25`   | Distance (m) to consider a goal reached          |
| `STEP_SIZE`           | `2.0`    | Default tree-expansion step size                 |
| `MAX_RRT_ITERATION`   | `10000`  | Maximum iterations before giving up              |
| `MAX_RRT_TIME`        | `100.0`  | Time limit per planning call (seconds)           |
| `GOAL_SAMPLE_RATE`    | `0.10`   | Fraction of samples biased toward the goal       |
| `RRT_VIZ_INTERVAL`    | `20`     | Iterations between visualization refreshes       |
| `FND_EXEC_PAUSE`      | `0.1`    | Seconds per execution step in FND animation      |
| `FND_REGROW_TIMEOUT`  | `5.0`    | Seconds allowed per regrow attempt               |
| `FND_REGROW_RETRIES`  | `3`      | Max regrow attempts before abandoning a repair   |
