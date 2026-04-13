'''
AER1516 - Project
Configuration file for the project. Contains constants and settings that can be easily modified
'''
# Adding time limits, similar to Activate
TIME_LIMITS = {
    'simple': 60,
    'narrow_passage': 60,
    'multi': 60,
    'simple_dynamic': 60,
    'hard_dynamic': 60,
}

CELL_SIZE = 0.50 # Meters
GOAL_SAMPLE_RATE = 0.1  # 10%
GOAL_SUCCESS_THRESH = 0.25 # Meters
STEP_SIZE = 2.0 # Meters

MAX_RRT_ITERATION = 10000 # samples
MAX_RRT_TIME = 100.0 # seconds

## VISUALS
RRT_VIZ_INTERVAL = 20 # iterations
FND_EXEC_PAUSE  = 0.1  # seconds per execution step (increase to slow down, 0.01 for fast)

## RRT*-FND TUNING
FND_REGROW_TIMEOUT   = 5.0  # wall-clock seconds allowed per regrow attempt
FND_REGROW_RETRIES   = 3    # max regrow attempts before giving up on this repair
FND_RECONNECT_RADIUS = 3.0  # neighbour-search radius used in reconnect() KDTree fallback