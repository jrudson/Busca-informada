from grid import Node, NodeGrid
from math import inf
import heapq


class PathPlanner(object):
    """
    Represents a path planner, which may use Dijkstra, Greedy Search or A* to plan a path.
    """

    def __init__(self, cost_map):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)

    @staticmethod
    def construct_path(goal_node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :type goal_node: Node.
        :return: the path as a sequence of (x, y) positions: [(x1,y1),(x2,y2),(x3,y3),...,(xn,yn)].
        :rtype: list of tuples.
        """
        node = goal_node
        # Since we are going from the goal node to the start node following the parents, we
        # are transversing the path in reverse
        reversed_path = []
        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent
        return reversed_path[::-1]  # This syntax creates the reverse list

    def dijkstra(self, start_position, goal_position):
        """
        Plans a path using the Dijkstra algorithm.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        # Todo: implement the Dijkstra algorithm
        global path, f
        pq = []
        heapq.heappush(pq, (0, start_position))

        while len(pq) > 0:
            f, node = heapq.heappop(pq)
            if isinstance(node, tuple):
                get_node = self.node_grid.get_node(node[0], node[1])
            else:
                get_node = self.node_grid.get_node(node.i, node.j)

            get_node.closed = True

            for successor in self.node_grid.get_successors(get_node.i, get_node.j):
                cost_edge = self.cost_map.get_edge_cost((get_node.i, get_node.j), successor)
                node_cost = self.node_grid.get_node(get_node.i, get_node.j)
                successor_cost = self.node_grid.get_node(successor[0], successor[1])

                if get_node.f == inf:
                    get_node.f = 0

                if successor_cost.f > (get_node.f + cost_edge):
                    successor_cost.f = get_node.f + cost_edge
                    successor_cost.parent = get_node
                    heapq.heappush(pq, (successor_cost.f, successor_cost))

                if successor == goal_position:
                    path = self.construct_path(successor_cost)
                    return path, f


        self.node_grid.reset()
        return path, f

    def greedy(self, start_position, goal_position):
        # Todo: implement the Greedy Search algorithm
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path
        global path, cost
        pq = []
        start = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        start.f = start.distance_to(goal_node.i, goal_node.j)
        heapq.heappush(pq, (start.f, start))

        while len(pq) > 0:
            cost, node = heapq.heappop(pq)
            if isinstance(node, tuple):
                node_start = self.node_grid.get_node(node[0], node[1])
            else:
                node_start = self.node_grid.get_node(node.i, node.j)

            node_start.closed = True

            for successor in self.node_grid.get_successors(node_start.i, node_start.j):
                successor_node = self.node_grid.get_node(successor[0], successor[1])

                successor_node_g = successor_node.distance_to(goal_node.i, goal_node.j)

                if not successor_node.closed:
                    successor_node.parent = node
                    successor_node.f = successor_node_g
                    heapq.heappush(pq, (successor_node.f, successor_node))
                    successor_node.closed = True

                if successor == goal_position:
                    path = self.construct_path(successor_node)

        self.node_grid.reset()
        return path, cost

    def a_star(self, start_position, goal_position):

        # Todo: implement the A* algorithm
        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path
        global path
        pq = []
        start = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        start.f = start.distance_to(goal_node.i, goal_node.j)
        start.g = 0
        heapq.heappush(pq, (start.f, start))

        while len(pq) > 0:
            cost, node = heapq.heappop(pq)
            if isinstance(node, tuple):
                node_start = self.node_grid.get_node(node[0], node[1])
            else:
                node_start = self.node_grid.get_node(node.i, node.j)

            node_start.closed = True

            for successor in self.node_grid.get_successors(node_start.i, node_start.j):
                successor_node = self.node_grid.get_node(successor[0], successor[1])
                edge_cost = self.cost_map.get_edge_cost((node_start.i, node_start.j), successor)
                start_cost_test = successor_node.distance_to(goal_node.i, goal_node.j)

                if successor_node.f > (node.g + edge_cost + start_cost_test) and not successor_node.closed:
                    successor_node.g = node.g + edge_cost
                    successor_node.f = successor_node.g + start_cost_test
                    successor_node.parent = node
                    successor_node.closed = True
                    heapq.heappush(pq, (successor_node.f, successor_node))


                if successor == goal_position:
                    path = self.construct_path(successor_node)
                    return path, cost

        self.node_grid.reset()
        return path, cost
