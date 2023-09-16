class Node:
    """ A node in the graph """

    def __init__(self, position: tuple[int, int], parent=None, cost=0, walkable=True, isStart=False, isGoal=False):
        """
        Creates a new instance of graph Node.
        Args:
            position: The node position in 2D. Tuple t(i,j) where i is the line and j is the column.
            parent: The parent Node in the graph. By default, None.
            cost: The intrinsic cost of this Node. By default, zero.
            walkable: True if the cell is walkable, False otherwise. By default, walkable.
            isStart: True if the current Node is the start node.
            isGoal: True if the current Node is the goal node.
        """
        self.position = position
        self.parent = parent
        self.cost = cost
        self.isWalkable = walkable
        self.isStart = isStart
        self.isGoal = isGoal

    def __lt__(self, other):
        return self.cost < other.cost
