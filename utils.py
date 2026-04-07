# =========================
# Update dynamic obstacles
# =========================
def update_obstacles(obstacles, grid):
    for obs in obstacles:
        obs.update(grid)