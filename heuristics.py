def base_heuristic(state):
    """
    Compute the base heuristic: Manhattan distance to the lamp location.
    """
    robot_x, robot_y = state.robot_location
    lamp_x, lamp_y = state.lamp_location

    # Manhattan distance to the lamp
    distance = abs(robot_x - lamp_x) + abs(robot_y - lamp_y)

    # Ensure proper transitions near the goal
    return distance


def advanced_heuristic(state):
    """
    Compute the advanced heuristic:
    - Manhattan distance to the lamp location.
    - Penalty for stairs gathering and lamp height adjustments.
    """
    robot_x, robot_y = state.robot_location
    lamp_x, lamp_y = state.lamp_location
    lamp_height = state.lamp_height
    stairs_held = state.stairs_held

    # Manhattan distance to the lamp
    distance_to_lamp = abs(robot_x - lamp_x) + abs(robot_y - lamp_y)

    # Current stairs at the lamp
    current_stairs_at_lamp = state.map[lamp_x][lamp_y]

    # Stairs needed to match the lamp height
    stairs_needed = max(0, lamp_height - (current_stairs_at_lamp + stairs_held))

    # If stairs are needed and robot holds stairs, account for placement cost
    if stairs_needed > 0 and stairs_held > 0:
        stairs_penalty = stairs_needed + 1  # 1 for placing stairs
    else:
        stairs_penalty = stairs_needed

    # If robot is far from the nearest stairs, add penalty for gathering stairs
    if stairs_needed > 0:
        nearest_stairs_distance = float('inf')
        for i, row in enumerate(state.map):
            for j, cell in enumerate(row):
                if cell > 0:  # Stairs exist in this cell
                    nearest_stairs_distance = min(nearest_stairs_distance, abs(robot_x - i) + abs(robot_y - j))
        stairs_penalty += nearest_stairs_distance

    # Total heuristic combines distance to lamp and stair penalties
    return distance_to_lamp + stairs_penalty
