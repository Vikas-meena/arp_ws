import numpy as np


class BackwardValueIteration:

    # Robot motion primitives
    ACTIONS = {
        "N": (0, 1),
        "S": (0, -1),
        "E": (1, 0),
        "W": (-1, 0),
    }

    def __init__(self, grid_map):

        self.map = grid_map

        self.width = grid_map.width
        self.height = grid_map.height

        # Initially all states have infinite cost
        self.values = np.full(
            (self.height, self.width),
            np.inf
        )

        # Optimal action/policy
        self.policy = np.full(
            (self.height, self.width),
            "",
            dtype="<U1"
        )

    def solve(self):

        # Goal state has zero cost
        gx, gy = self.map.goal

        self.values[gy, gx] = 0

        changed = True
        iterations = 0

        while changed:

            changed = False
            iterations += 1

            for y in range(self.height):

                for x in range(self.width):

                    # Ignore obstacles
                    if self.map.grid[y, x] == 1:
                        continue

                    # Goal is already optimal
                    if (x, y) == self.map.goal:
                        continue

                    best_value = np.inf
                    best_action = ""

                    # Try all four motion primitives
                    for action, (dx, dy) in self.ACTIONS.items():

                        nx = x + dx
                        ny = y + dy

                        # Ignore invalid states
                        if not self.map.is_valid(nx, ny):
                            continue

                        # Movement cost = 1
                        candidate_value = (
                            1 + self.values[ny, nx]
                        )

                        if candidate_value < best_value:

                            best_value = candidate_value
                            best_action = action

                    # Update state
                    if best_value < self.values[y, x]:

                        self.values[y, x] = best_value

                        self.policy[y, x] = best_action

                        changed = True

        print(
            "BVI iterations:",
            iterations
        )

        return self.values, self.policy