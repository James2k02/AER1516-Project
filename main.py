'''
AER1516 - Project
Main file for the project. This file will contain the main loop and logic for running the pathfinding algorithms on the defined maps. 
It will also handle the visualization of the maps and the paths found by the algorithms
'''

import matplotlib.pyplot as plt
from maps import get_map

# So far, this will just be the animation for maps 4 and 5 (dynamic maps) and later will add the whole pipeline

# =========================
# Update dynamic obstacles
# =========================
# def update_obstacles(obstacles, grid):
#     for obs in obstacles:
#         r, c = obs["pos"]
#         vr, vc = obs["vel"]
#         size = obs["size"]

#         # move
#         r_new = r + vr
#         c_new = c + vc

#         # =========================
#         # VERTICAL (UP/DOWN)
#         # =========================
#         if r_new < 1:
#             r_new = 1
#             vr *= -1

#         elif r_new > grid.shape[0] - size - 1:
#             r_new = grid.shape[0] - size - 1
#             vr *= -1

#         # =========================
#         # HORIZONTAL (LEFT/RIGHT)
#         # =========================
#         if c_new < 1:
#             c_new = 1
#             vc *= -1

#         elif c_new > grid.shape[1] - size - 1:
#             c_new = grid.shape[1] - size - 1
#             vc *= -1

#         # update
#         obs["pos"] = [r_new, c_new]
#         obs["vel"] = [vr, vc]

def is_valid_position(grid, r, c, size):
    for i in range(size):
        for j in range(size):
            rr = r + i
            cc = c + j

            # out of bounds
            if rr < 0 or rr >= grid.shape[0] or cc < 0 or cc >= grid.shape[1]:
                return False

            # hit wall
            if grid[rr, cc] == 1:
                return False

    return True

def update_obstacles(obstacles, grid):
    for obs in obstacles:
        r, c = obs["pos"]
        vr, vc = obs["vel"]
        size = obs["size"]

        # try move
        r_new = r + vr
        c_new = c + vc

        # check if move is valid
        if is_valid_position(grid, r_new, c_new, size):
            obs["pos"] = [r_new, c_new]
        else:
            # bounce if blocked
            obs["vel"][0] *= -1
            obs["vel"][1] *= -1
            
            
# =========================
# MAIN FUNCTION
# =========================
def run_simulation(map_name):
    # load map
    grid, start, goal, meta = get_map(map_name)

    # =========================
    # Define dynamic obstacles
    # =========================
    if map_name == "map4":
        dynamic_obstacles = [
            {"pos": [6, 2], "size": 2, "vel": [0, 1]},   # top block
            {"pos": [14, 5], "size": 2, "vel": [0, -1]}  # bottom block
        ]

    elif map_name == "map5":
        dynamic_obstacles = [
            {"pos": [5, 1], "size": 2, "vel": [1, 0]},    # vertical
            {"pos": [15, 9], "size": 2, "vel": [-1, 0]},  # vertical
            {"pos": [10, 11], "size": 2, "vel": [0, 1]},   # horizontal
            {"pos": [7, 14], "size": 2, "vel": [0, -1]},  # horizontal
            {"pos": [12, 12], "size": 2, "vel": [1, 0]},  # vertical
        ]

    else:
        dynamic_obstacles = []
               
    # =========================
    # Visualization loop
    # =========================
    plt.ion()
    fig, ax = plt.subplots()

    while True:
        ax.clear()

        # draw map (white = free, black = obstacles)
        ax.imshow(1 - grid, cmap='gray', origin='upper')

        # update dynamic obstacles
        update_obstacles(dynamic_obstacles, grid)

        # draw dynamic obstacles
        for obs in dynamic_obstacles:
            r, c = obs["pos"]
            size = obs["size"]

        # draw square block
            for i in range(size):
                for j in range(size):
                    ax.scatter(c + j, r + i, c='blue', s=120)

        # draw start and goal
        ax.scatter(start[1], start[0], c='green', s=120, label='Start')
        ax.scatter(goal[1], goal[0], c='red', s=120, label='Goal')

        # title
        ax.set_title(meta["name"])

        # remove axis ticks for cleaner look
        ax.set_xticks([])
        ax.set_yticks([])

        # avoid duplicate legend entries
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())

        plt.pause(0.2)
        
# =========================
# RUN HERE
# =========================
if __name__ == "__main__":
    # choose map: "map4" or "map5"
    run_simulation("map5")