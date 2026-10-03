import numpy as np


class GridMap:

    def __init__(
        self,
        width=30,
        height=30,
        obstacle_ratio=0.20,
        seed=42
    ):

        self.width = width
        self.height = height
        self.obstacle_ratio = obstacle_ratio
        self.seed = seed

        # 0 = free
        # 1 = obstacle
        self.grid = np.zeros(
            (height, width),
            dtype=int
        )

        # Coordinates are (x, y)
        self.start = (1, 1)
        self.goal = (28, 28)

        self.generate_obstacles()

    def generate_obstacles(self):

        rng = np.random.default_rng(self.seed)

        total_cells = self.width * self.height

        obstacle_count = int(
            total_cells * self.obstacle_ratio
        )

        cells = [
            (x, y)
            for y in range(self.height)
            for x in range(self.width)
        ]

        # Start and goal cannot be obstacles
        cells.remove(self.start)
        cells.remove(self.goal)

        selected = rng.choice(
            len(cells),
            size=obstacle_count,
            replace=False
        )

        for index in selected:

            x, y = cells[index]

            self.grid[y, x] = 1

        # Ensure start and goal are free
        sx, sy = self.start
        gx, gy = self.goal

        self.grid[sy, sx] = 0
        self.grid[gy, gx] = 0

    def is_valid(self, x, y):

        return (
            0 <= x < self.width
            and
            0 <= y < self.height
            and
            self.grid[y, x] == 0
        )

    def is_obstacle(self, cell):

        x, y = cell

        return self.grid[y, x] == 1


if __name__ == "__main__":

    grid_map = GridMap()

    print(
        "Map size:",
        grid_map.width,
        "x",
        grid_map.height
    )

    print(
        "Total cells:",
        grid_map.width * grid_map.height
    )

    print(
        "Obstacles:",
        np.sum(grid_map.grid == 1)
    )

    print(
        "Start:",
        grid_map.start
    )

    print(
        "Goal:",
        grid_map.goal
    )