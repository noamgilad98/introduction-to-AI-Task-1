class grid_robot_state:
    def __init__(self, robot_location, map=None, lamp_height=-1, lamp_location=(-1, -1), stairs_held=0):
        """
        Initialize the grid_robot_state with the robot's location, map, lamp height/location, and stairs held.
        """
        assert isinstance(robot_location, tuple) and len(robot_location) == 2, "Invalid robot_location"
        assert map is not None and all(isinstance(row, list) for row in map), "Invalid map"
        self.robot_location = robot_location
        self.map = map
        self.lamp_height = lamp_height
        self.lamp_location = lamp_location
        self.stairs_held = stairs_held

    @staticmethod
    def is_goal_state(state):
        """
        Check if the robot is at the lamp's location and the lamp has the correct height.
        """
        x, y = state.robot_location
        return (
            (x, y) == state.lamp_location and
            state.map[x][y] == state.lamp_height
        )

    def get_neighbors(self):
        """
        Generate neighboring states with their respective costs.
        """
        neighbors = []
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        x, y = self.robot_location
        map_height, map_width = len(self.map), len(self.map[0])

        # תנועה לכל הכיוונים האפשריים
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < map_height and 0 <= ny < map_width and self.map[nx][ny] != -1:
                cost = 1 + self.stairs_held  # עלות = 1 + גובה המדרגות המוחזקות
                neighbors.append((
                    grid_robot_state(
                        robot_location=(nx, ny),
                        map=self.map,
                        lamp_height=self.lamp_height,
                        lamp_location=self.lamp_location,
                        stairs_held=self.stairs_held
                    ), cost))

        # הרמת מדרגות (אם הרובוט לא מחזיק מדרגות ויש מדרגות במשבצת הנוכחית)
        if self.stairs_held == 0 and self.map[x][y] > 0:
            new_map = [row[:] for row in self.map]
            new_map[x][y] = 0  # המדרגות נלקחו מהמשבצת
            neighbors.append((
                grid_robot_state(
                    robot_location=(x, y),
                    map=new_map,
                    lamp_height=self.lamp_height,
                    lamp_location=self.lamp_location,
                    stairs_held=self.map[x][y]
                ), 1))  # עלות הרמת מדרגות = 1

        # הנחת מדרגות
        if self.stairs_held > 0:
            if self.map[x][y] == 0:  # המשבצת ריקה
                new_map = [row[:] for row in self.map]
                new_map[x][y] = self.stairs_held
                neighbors.append((
                    grid_robot_state(
                        robot_location=(x, y),
                        map=new_map,
                        lamp_height=self.lamp_height,
                        lamp_location=self.lamp_location,
                        stairs_held=0
                    ), 1))  # עלות הנחת מדרגות = 1

            elif self.map[x][y] > 0 and self.stairs_held + self.map[x][y] <= self.lamp_height:  # המשבצת מכילה מדרגות
                # חיבור מדרגות ליחידה אחת
                new_map = [row[:] for row in self.map]
                combined_height = self.map[x][y] + self.stairs_held
                new_map[x][y] = combined_height
                neighbors.append((
                    grid_robot_state(
                        robot_location=(x, y),
                        map=new_map,
                        lamp_height=self.lamp_height,
                        lamp_location=self.lamp_location,
                        stairs_held=0
                    ), 1))  # עלות חיבור מדרגות = 1

        # חיבור מדרגות תוך כדי הרובוט מחזיק את התוצאה
        if self.stairs_held > 0 and self.map[x][y] > 0:
            combined_height = self.stairs_held + self.map[x][y]
            if combined_height <= self.lamp_height:
                new_map = [row[:] for row in self.map]
                new_map[x][y] = 0  # המדרגות מחוברות לרובוט
                neighbors.append((
                    grid_robot_state(
                        robot_location=(x, y),
                        map=new_map,
                        lamp_height=self.lamp_height,
                        lamp_location=self.lamp_location,
                        stairs_held=combined_height
                    ), 1))  # עלות חיבור מדרגות = 1

        return neighbors

    def get_state_str(self):
        """
        Return a string representation of the current state.
        """
        return f"Location: {self.robot_location}, Stairs Held: {self.stairs_held}, Lamp: {self.lamp_location}"

    def __hash__(self):
        """
        Generate a unique hash for the state.
        """
        return hash((self.robot_location, self.stairs_held, tuple(tuple(row) for row in self.map)))

    def __eq__(self, other):
        """
        Check if two states are equal.
        """
        return (
            self.robot_location == other.robot_location and
            self.stairs_held == other.stairs_held and
            self.map == other.map
        )
