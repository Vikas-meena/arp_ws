import matplotlib.pyplot as plt
import numpy as np

from map_generator import GridMap
from value_iteration import BackwardValueIteration
from path_planner import PathPlanner


def main():

    # -----------------------------
    # Create map
    # -----------------------------

    grid_map = GridMap()

    # -----------------------------
    # Run BVI
    # -----------------------------

    bvi = BackwardValueIteration(grid_map)

    values, policy = bvi.solve()

    # -----------------------------
    # Extract optimal path
    # -----------------------------

    planner = PathPlanner(
        grid_map,
        policy
    )

    path = planner.get_path()

    if not path:

        print("ERROR: No path found!")

        return

    print()
    print("Optimal path found!")
    print("Number of movements:", len(path) - 1)

    print()
    print("Path:")

    print(path)

    # -----------------------------
    # Plot map
    # -----------------------------

    fig, ax = plt.subplots(
        figsize=(10, 10)
    )

    # Cost-to-go values
    display_values = values.copy()

    # Hide obstacles
    obstacle_mask = grid_map.grid == 1

    display_values[
        obstacle_mask
    ] = np.nan

    image = ax.imshow(
        display_values,
        origin="lower",
        cmap="viridis"
    )

    # -----------------------------
    # Draw obstacles
    # -----------------------------

    obstacle_display = np.zeros(
        (grid_map.height,
         grid_map.width,
         4)
    )

    obstacle_display[
        obstacle_mask
    ] = [0, 0, 0, 1]

    ax.imshow(
        obstacle_display,
        origin="lower"
    )

    # -----------------------------
    # Draw optimal path
    # -----------------------------

    path_x = [
        state[0]
        for state in path
    ]

    path_y = [
        state[1]
        for state in path
    ]

    ax.plot(
        path_x,
        path_y,
        linewidth=3,
        label="Optimal Path"
    )

    # -----------------------------
    # Start
    # -----------------------------

    sx, sy = grid_map.start

    ax.scatter(
        sx,
        sy,
        marker="o",
        s=200,
        label="Start"
    )

    # -----------------------------
    # Goal
    # -----------------------------

    gx, gy = grid_map.goal

    ax.scatter(
        gx,
        gy,
        marker="*",
        s=300,
        label="Goal"
    )

    # -----------------------------
    # Labels
    # -----------------------------

    ax.set_title(
        "Backward Value Iteration - Optimal Path"
    )

    ax.set_xlabel("X")

    ax.set_ylabel("Y")

    ax.set_xticks(range(0, 30, 5))

    ax.set_yticks(range(0, 30, 5))

    ax.grid(True, linewidth=0.3)

    plt.colorbar(
        image,
        ax=ax,
        label="Cost-to-Go"
    )

    ax.legend()

    plt.tight_layout()

    plt.show()


if __name__ == "__main__":
    main()
