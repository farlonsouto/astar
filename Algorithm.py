import heapq


class Node:
    """ A node in the graph """

    def __init__(self, position, parent=None, cost=0):
        self.position = position
        self.parent = parent
        self.cost = cost


# -----------------------------------------------------------------------------------------------------------

def estimatedCost(origin, destination):
    """ Heuristic function. Estimates the cost of going from an origin node to a destination node"""
    x1, y1 = origin.position
    x2, y2 = destination.position
    return abs(x1 - x2) + abs(y1 - y2)


# -----------------------------------------------------------------------------------------------------------

def get_neighbors(node):
    """ Looks around. Finds the adjacent nodes. """
    x, y = node.position
    neighbors = []

    # Add adjacent nodes (up, down, left, right)
    for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
        new_x, new_y = x + dx, y + dy
        neighbors.append(Node((new_x, new_y), parent=node, cost=node.cost + 1))

    return neighbors


# -----------------------------------------------------------------------------------------------------------

def findPath(start, goal):
    """ A* algorithm straightforward implementation """
    open_list = []
    closed_list = set()

    heapq.heappush(open_list, (start.cost, start))

    while open_list:
        current_cost, current_node = heapq.heappop(open_list)

        if current_node == goal:
            # Goal reached, construct and return the path
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        closed_list.add(current_node)

        for neighbor in get_neighbors(current_node):
            if neighbor in closed_list:
                continue

            new_cost = current_node.cost + 1
            if neighbor not in open_list:
                heapq.heappush(open_list, (new_cost + estimatedCost(neighbor, goal), neighbor))
            elif new_cost < neighbor.cost:
                neighbor.cost = new_cost
                neighbor.parent = current_node
