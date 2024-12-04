import heapq
from search_node import search_node
from grid_robot_state import grid_robot_state

class PriorityQueue:
    def __init__(self):
        self.elements = []
        self.entry_finder = {}  # Dictionary to keep track of states
        self.REMOVED = '<removed>'  # Placeholder for removed entries
        self.counter = 0  # To ensure stable priority queue behavior for equal priorities

    def add(self, node, priority):
        """
        Add a node to the priority queue or update its priority if it already exists.
        """
        if node.state in self.entry_finder:
            self.remove(node.state)
        count = self.counter
        self.counter += 1
        entry = [priority, count, node]
        self.entry_finder[node.state] = entry
        heapq.heappush(self.elements, entry)

    def remove(self, state):
        """
        Mark a state as removed.
        """
        entry = self.entry_finder.pop(state)
        entry[-1] = self.REMOVED

    def pop(self):
        """
        Pop and return the node with the lowest priority.
        """
        while self.elements:
            priority, count, node = heapq.heappop(self.elements)
            if node is not self.REMOVED:
                del self.entry_finder[node.state]
                return node
        raise KeyError("Pop from an empty priority queue")

    def is_empty(self):
        return not self.entry_finder

def create_open_set():
    """
    Create and return an empty priority queue for the open set.
    """
    return PriorityQueue()

def create_closed_set():
    """
    Create and return an empty set for the closed states.
    """
    return set()

def add_to_open(vn, open_set):
    """
    Add a node to the open set, using its f value as priority.
    """
    open_set.add(vn, vn.f)

def open_not_empty(open_set):
    """
    Check if the open set is not empty.
    """
    return not open_set.is_empty()

def get_best(open_set):
    """
    Pop the best node (with the lowest f value) from the open set.
    """
    return open_set.pop()

def add_to_closed(vn, closed_set):
    """
    Add the state of the current node to the closed set.
    """
    closed_set.add(vn.state)

def duplicate_in_open(vn, open_set):
    """
    Check if the state is in the open set with a lower or equal g value.
    """
    if vn.state in open_set.entry_finder:
        existing_entry = open_set.entry_finder[vn.state]
        _, _, existing_node = existing_entry
        if existing_node.g <= vn.g:
            return True
    return False

def duplicate_in_closed(vn, closed_set):
    """
    Check if the state is in the closed set.
    """
    return vn.state in closed_set

# Helps to debug sometimes.
def print_path(path):
    """
    Print the path from the start state to the goal state.
    """
    for i in range(len(path) - 1):
        print(f"[{path[i].state.get_state_str()}]", end=", ")
    print(path[-1].state.get_state_str())
    print(path[-1].g)

def search(start_state, heuristic):
    """
    A* search algorithm for finding the path from start_state to the goal.
    """
    open_set = create_open_set()
    closed_set = create_closed_set()

    # Start with the search node, not directly the state
    start_node = search_node(start_state, 0, heuristic(start_state))
    add_to_open(start_node, open_set)

    while open_not_empty(open_set):
        # Get the best node from the open set
        current = get_best(open_set)

        # Debugging: print the cost of the current node

        # Check if we've reached the goal state (now using current.state)
        if grid_robot_state.is_goal_state(current.state):
            path = []
            while current:
                path.append(current)
                print(f"Current Cost (g): {current.g}, Current State: {current.state.get_state_str()}")

                current = current.prev
            path.reverse()
            print_path(path)
            
            return path

        add_to_closed(current, closed_set)

        # Iterate through neighbors of the current state
        for neighbor, edge_cost in current.get_neighbors():
            curr_neighbor = search_node(neighbor, current.g + edge_cost, heuristic(neighbor), current)

            # Ensure consistent g-values
            if not duplicate_in_open(curr_neighbor, open_set) and not duplicate_in_closed(curr_neighbor, closed_set):
                add_to_open(curr_neighbor, open_set)

    return None
