import matplotlib.pyplot as plt
import numpy as np

from map_generator import GridMap
from value_iteration import BackwardValueIteration


def main():

    # Create map
    grid_map = GridMap()

    # Run BVI
    bvi = BackwardValueIteration(grid_map)
    values, policy = bvi.solve()

    # Create figure
    fig, ax = plt.subplots(figsize=(10, 10))

    # Copy values for visualization
    display_values = values.copy()

    # Obstacles will be shown separately
    obstacle_mask = grid_map.grid == 1

    # Hide obstacle values
    display_values[obstacle_mask] = np.nan

    # Show cost-to-go values
    image = ax.imshow(
        display_values,
        origin="lower",
        cmap="viridis"
    )

    # Show obstacles
    obstacle_display = np.zeros(
        (grid_map.height, grid_map.width, 4)
    )

    obstacle_display[obstacle_mask] = [0, 0, 0, 1]

    ax.imshow(
        obstacle_display,
        origin="lower"
    )

    # Start
    sx, sy = grid_map.start

    ax.scatter(
        sx,
        sy,
        marker="o",
        s=150,
        label="Start"
    )

    # Goal
    gx, gy = grid_map.goal

    ax.scatter(
        gx,
        gy,
        marker="*",
        s=250,
        label="Goal"
    )

    # Add cost values to cells
    for y in range(grid_map.height):

        for x in range(grid_map.width):

            if grid_map.grid[y, x] == 1:
                continue

            if np.isfinite(values[y, x]):

                ax.text(
                    x,
                    y,
                    str(int(values[y, x])),
                    ha="center",
                    va="center",
                    fontsize=5
                )

    ax.set_title("Backward Value Iteration - Cost-to-Go")

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
