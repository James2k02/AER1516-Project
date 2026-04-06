'''
AER1516 - Project
Temporary file for visualizing the maps defined in maps.py. This file can be used to quickly check the layout of the maps and ensure that the start and goal points are correctly placed. 
It uses matplotlib to display the grid, with obstacles shown in black, free space in white, the start point in green, and the goal point in red.
'''

import matplotlib.pyplot as plt
from maps import get_map

m = get_map("map1")
grid = m.grid
start = m.start
goals = m.goals
name = m.name

plt.imshow(1 - grid, cmap = 'gray', origin = 'upper')

# plot start (green)
plt.scatter(start[1], start[0], c = 'green', s = 100, label = 'Start')

# plot goals (red)
for goal in goals:
    plt.scatter(goal[1], goal[0], c = 'red', s = 100, label = 'Goal')

plt.title(name)
plt.legend()
plt.show()