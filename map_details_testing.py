from maps import get_map

def debug_map(map_name):
    m = get_map(map_name)

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


if __name__ == "__main__":
    debug_map("map5")