from map_generator import GridMap
from value_iteration import BackwardValueIteration


def main():

    # Generate map
    grid_map = GridMap()

    print("Map created")
    print("Start:", grid_map.start)
    print("Goal:", grid_map.goal)

    # Create BVI solver
    bvi = BackwardValueIteration(grid_map)

    # Run BVI
    values, policy = bvi.solve()

    # Start coordinates
    sx, sy = grid_map.start

    print()
    print("Cost-to-go from START:")

    print(values[sy, sx])

    print()
    print("Optimal first action from START:")

    print(policy[sy, sx])


if __name__ == "__main__":
    main()
