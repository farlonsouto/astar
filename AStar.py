import heapq
import time

from Node import Node
from TaskMap import *


class AStar:
    """
    TDT4136 - Introduction to Artificial Intelligence - NTNU - Autumn 2023
    Assignment 1: Implement the A* Algorithm

    Pseudo-Algorithm Reference: https://brilliant.org/wiki/a-star-search/ """

    def __init__(self, mapManager: TaskMap):
        """ Creates a new instance of AStar
            Args:
                mapManager: Injects the dependency on a MapManager instance.
        """
        self.mapManager = mapManager
        bothMaps = mapManager.get_maps()
        numMap = bothMaps[0]
        self.stringMap = bothMaps[1]
        nodeMap_Aux = []

        # loads both the numeric and the string array data into the Node map (2D array)
        for i in range(len(numMap)):
            nodeArray = []
            for j in range(len(numMap[i])):
                value = self.stringMap[i][j]
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
        """ Manhattan heuristic function. Estimates the cost of going from an origin node to a destination node as a
            taxi driver would, i.e. not in a straight line, but through the blocks.
            Args:
                origin: The current node in the graph. destination: The node in the graph to be reached,
                the goal.
            Returns: A numerical value representing the cost of moving from the origin node to the destination
                node.
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

    def path(self) -> list[Node]:
        """ register the path """
        print("Path to Goal found!")
        # Goal reached, construct and return the path
        path = []
        # builds the path by following the parent, then the grandparent and so forth
        node = self.goalNode
        while node:
            # geta a grey path in the map
            self.stringMap[node.position[0]][node.position[1]] = " , "
            # append the node to the path
            path.append(node)
            # gets the current node's parent
            node = node.parent
        # the semantics is array[start : end : step], but array[::-1] reverses the array
        return path[::-1]

    # -----------------------------------------------------------------------------------------------------------

    def aStartInformedSearch(self, theClokIsTicking=False) -> list[Node]:
        """
        Straightforward implementation of the A* path finding algorithm. A call to this function will cause a temporary
        image to be generated showing the calculated path.
        Args:
            theClokIsTicking: If set True will cause goal movement at each 4th main loop iteration. False by default.
        returns:
            A list of Nodes corresponding to the path from the start Node to the goal Node.
        deprecated: Does not inform the correct list. The corresponding fix is to be released in the next version.
        """
        openList = []
        closedList = []
        heapq.heappush(openList, (self.startNode.cost, self.startNode))

        print("startNode: " + str(self.startNode.position))
        print("goalNode: " + str(self.goalNode.position))

        # self.mapManager.show_map()

        while True:

            if theClokIsTicking:
                self.mapManager.tick()

            currentNodeCost, currentNode = heapq.heappop(openList)
            # Let's place current node goes into the closed list, so we know it was already visited:
            heapq.heappush(closedList, currentNode)
            # let's look around in the neighborhood to find the path:
            neighborhood = self.__getNeighbors(currentNode)
            for neighbor in neighborhood:

                # Goal node just around?
                if neighbor is self.goalNode:
                    self.goalNode.parent = currentNode
                    # geta a grey path in the map
                    self.stringMap[currentNode.position[0]][currentNode.position[1]] = " * "
                    self.mapManager.show_map()
                    return self.path()

                # cost_node_n_f(n) = cost_from_start_to_here_g(n) + cost_heuristic_from_here_to_goal_h(n)
                neighborEstimatedCost = neighbor.cost + self.__estimatedCost(neighbor, self.goalNode)
                currentNodeCost = currentNodeCost + self.__estimatedCost(currentNode, self.goalNode)
                if neighborEstimatedCost < currentNodeCost and neighbor in closedList:
                    neighbor.cost = neighborEstimatedCost
                    neighbor.parent = currentNode
                    # gets a grey path in the map
                    self.stringMap[currentNode.position[0]][currentNode.position[1]] = " * "
                elif currentNodeCost < neighborEstimatedCost and neighbor in list(map(lambda item: item[1], openList)):
                    neighbor.cost = neighborEstimatedCost
                    currentNode.parent = neighbor
                    # gets a grey path in the map
                    self.stringMap[currentNode.position[0]][currentNode.position[1]] = " * "
                else:
                    heapq.heappush(openList, (neighborEstimatedCost, neighbor))
