class PathPlanner:

    def __init__(self, grid_map, policy):

        self.map = grid_map
        self.policy = policy

    def get_path(self):

        current = self.map.start
        goal = self.map.goal

        path = [current]

        # Safety limit
        max_steps = (
            self.map.width *
            self.map.height
        )

        for _ in range(max_steps):

            # Goal reached
            if current == goal:

                return path

            x, y = current

            action = self.policy[y, x]

            # No valid policy
            if action == "":

                return []

            # Convert action to next state
            if action == "N":

                next_state = (
                    x,
                    y + 1
                )

            elif action == "S":

                next_state = (
                    x,
                    y - 1
                )

            elif action == "E":

                next_state = (
                    x + 1,
                    y
                )

            elif action == "W":

                next_state = (
                    x - 1,
                    y
                )

            else:

                return []

            # Check next state
            if not self.map.is_valid(
                *next_state
            ):

                return []

            path.append(next_state)

            current = next_state

        # If this happens, something went wrong
        return []