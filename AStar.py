import heapq

from Node import Node
from TaskMap import *


class AStar:
    """
    TDT4136 - Introduction to Artificial Intelligence - NTNU - Autumn 2023
    Assignment 1: Implement the A* Algorithm
    """

    def __init__(self, mapManager: TaskMap):
        """ Creates a new instance of AStar
            Args:
                mapManager: Injects the dependency on a MapManager instance.
        """
        self.mapManager = mapManager
        bothMaps = mapManager.get_maps()
        numMap = bothMaps[0]
        stringMap = bothMaps[1]
        nodeMap_Aux = []

        # loads both the numeric and the string array data into the Node map (2D array)
        for i in range(len(numMap)):
            nodeArray = []
            for j in range(len(numMap[i])):
                value = stringMap[i][j]
                num = numMap[i][j]
                nodeArray.append(Node((i, j), None, num, num > 0, value == ' S ', value == ' G '))
            nodeMap_Aux.append(nodeArray)

        self.nodeMap = nodeMap_Aux

        startPos = mapManager.get_start_pos()
        goalPos = mapManager.get_goal_pos()

        self.startNode = self.nodeMap[startPos[0]][startPos[1]]
        self.goalNode = self.nodeMap[goalPos[0]][goalPos[1]]

    # -----------------------------------------------------------------------------------------------------------

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

    def __getNeighbors(self, node) -> [Node]:
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
            # check the borders, the size of the arrays
            if new_x in range(len(self.nodeMap)) and new_y in range(len(self.nodeMap[0])):
                # disregard non-walkable neighbors
                if self.nodeMap[new_x][new_y].isWalkable:
                    neighbors.append(self.nodeMap[new_x][new_y])
        return neighbors

    # -----------------------------------------------------------------------------------------------------------

    def findPath(self) -> list[Node]:
        """ A* algorithm straightforward implementation """
        open_list = []
        closed_list = set()

        heapq.heappush(open_list, (self.startNode.cost, self.startNode))

        while open_list:
            current_cost, current_node = heapq.heappop(open_list)

            # if the current node is the goal node, register the path
            if current_node == self.goalNode:
                print("Path to Goal found!")
                # Goal reached, construct and return the path
                path = []
                # builds the path by following the parent, then the grandparent and so forth
                while current_node:
                    path.append(current_node)
                    current_node = current_node.parent
                # the semantics is array[start : end : step], but array[::-1] reverses the array
                return path[::-1]

            # no matter what, the current node goes to the closed list
            closed_list.add(current_node)

            # Let's check the neighbours
            for neighbor in self.__getNeighbors(current_node):
                # Skipping neighbours that were already visited
                if neighbor in closed_list:
                    continue
                # Not the goal yet, thus the cost increase by 1
                new_cost = current_node.cost + 1
                # Were the paths through this neighbor investigated? If not, it goes into the open list
                if neighbor not in open_list:
                    heapq.heappush(open_list, (new_cost + self.__estimatedCost(neighbor, self.goalNode), neighbor))
                # OK, paths investigated: Is this path, through this neighbor, cheaper?
                elif new_cost < neighbor.cost:
                    # Actually, going through this neighbor is cheaper than estimated
                    neighbor.cost = new_cost
                    # Let's consider this neighbor to build the path
                    neighbor.parent = current_node
