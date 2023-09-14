class Node:
    """ A node in the graph """

    def __init__(self, position, parent=None, cost=0):
        """
        Creates a new instance of graph Node.
        Aegs:
            position: The node position iin the graph.
            parent: The parent Node in the graph.
            cost: The intrinsic cost of this Node.
        """
        self.position = position
        self.parent = parent
        self.cost = cost

