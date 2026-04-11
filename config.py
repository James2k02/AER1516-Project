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
STEP_SIZE = 3.0 # Meters

MAX_RRT_ITERATION = 20000 # samples
MAX_RRT_TIME = 60.0 # seconds

## VISUALS
RRT_VIZ_INTERVAL = 20 # iterations