import heapq
import numbers

from Node import Node


class Algorithm:

    @classmethod
    def __estimatedCost(cls, origin, destination) -> int:
        """ Heuristic function. Estimates the cost of going from an origin node to a destination node.
            Args:
                origin: The current node in the graph.
                destination: The node in the graph to be reached, the goal.
            Returns:
                A numerical value representing the cost of moving from the origin node to the destination node.
        """
        x1, y1 = origin.position
        x2, y2 = destination.position
        return abs(x1 - x2) + abs(y1 - y2)

    # -----------------------------------------------------------------------------------------------------------

    @classmethod
    def __getNeighbors(cls, node) -> [Node]:
        """ Looks around (right, left, up and down). Finds the adjacent nodes.
            Args:
                node: An instance of Node representing a node in the graph.
            Returns:
                An array populated with non-null instances of the class Node representing the adjacent nodes.
        """
        x, y = node.position
        neighbors = []

        # Add adjacent nodes (up, down, left, right)
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            new_x, new_y = x + dx, y + dy
            neighbors.append(Node((new_x, new_y), node, node.cost + 1))

        return neighbors

    # -----------------------------------------------------------------------------------------------------------

    @classmethod
    def findPath(cls, start, goal):
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

            for neighbor in cls.__getNeighbors(current_node):
                if neighbor in closed_list:
                    continue

                new_cost = current_node.cost + 1
                if neighbor not in open_list:
                    heapq.heappush(open_list, (new_cost + cls.__estimatedCost(neighbor, goal), neighbor))
                elif new_cost < neighbor.cost:
                    neighbor.cost = new_cost
                    neighbor.parent = current_node
