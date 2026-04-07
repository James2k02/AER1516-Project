from maps import get_map
import matplotlib.pyplot as plt

def debug_map(m):

    print("\n=========================")
    print(f"Map: {m.name}")
    print("=========================")

    print("\nStart:", m.start)
    print("Goals:", m.goals)

    print("\n--- Static Obstacles ---")
    for obs in m.static_obstacles:
        print(obs)

    print("\n--- Dynamic Obstacles ---")
    for obs in m.dynamic_obstacles:
        print(obs)

def visualize_map(m, steps=50, pause_time=0.2):
    plt.ion()
    fig, ax = plt.subplots()

    for _ in range(steps):
        ax.clear()

        # =========================
        # 1. Update dynamic obstacles
        # =========================
        for obs in m.dynamic_obstacles:
            obs.update(m.grid)

        # =========================
        # 2. Draw grid
        # =========================
        ax.imshow(1 - m.grid, cmap='gray', origin='upper')

        # =========================
        # 3. Draw static obstacles (blue)
        # =========================
        for obs in m.static_obstacles:
            rect = plt.Rectangle(
                (obs.x, obs.y),
                obs.w,
                obs.h,
                edgecolor='blue',
                facecolor='none',
                linewidth=2
            )
            ax.add_patch(rect)

        # =========================
        # 4. Draw dynamic obstacles (red)
        # =========================
        for obs in m.dynamic_obstacles:
            rect = plt.Rectangle(
                (obs.x, obs.y),
                obs.w,
                obs.h,
                edgecolor='red',
                facecolor='none',
                linewidth=2
            )
            ax.add_patch(rect)

        # =========================
        # 5. Draw start + goals
        # =========================
        ax.scatter(m.start[1], m.start[0], c='green', s=100, label='Start')
        for g in m.goals:
            ax.scatter(g[1], g[0], c='red', s=100, label='Goal')

        # =========================
        # 6. Formatting
        # =========================
        ax.set_title(m.name)
        ax.set_xlim(0, m.grid.shape[1])
        ax.set_ylim(m.grid.shape[0], 0)
        ax.set_xticks([])
        ax.set_yticks([])

        # clean legend
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())

        plt.pause(pause_time)

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    m = get_map("map5")

    debug_map(m)
    visualize_map(m)