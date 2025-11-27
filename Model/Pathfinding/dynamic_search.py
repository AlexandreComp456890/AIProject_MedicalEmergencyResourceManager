import heapq
import math
from Model.Utils.geo import cost_to_move, heuristic


# ============================================================
# PRIORITY QUEUE WRAPPER
# ============================================================

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return not self.elements

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

    def top_key(self):
        if self.empty():
            return float('inf'), float('inf')
        return self.elements[0][0]

    def remove(self, item):
        self.elements = [(p, i) for (p, i) in self.elements if i != item]
        heapq.heapify(self.elements)


# ============================================================
# D* LITE FOR GRAPH
# ============================================================

class DStarLite:
    def __init__(self, graph, start, goal):
        self.graph = graph
        self.start = start      # Robot start node
        self.goal = goal        # Target node

        # Core data
        self.rhs = {}
        self.g = {}
        self.U = PriorityQueue()

        # Last start position for incremental repair
        self.last_start = start

        # Initialize data structures
        for node in self.graph.nodes.keys():
            self.rhs[node] = float('inf')
            self.g[node] = float('inf')

        # Goal has rhs = 0
        self.rhs[self.goal] = 0
        self.U.put(self.goal, self.calculate_key(self.goal))

    # ============================================================
    # KEY CALCULATION
    # ============================================================

    def calculate_key(self, node):
        g_rhs = min(self.g[node], self.rhs[node])
        return (
            g_rhs + heuristic(self.graph, self.start, node),
            g_rhs
        )

    # ============================================================
    # UPDATE VERTEX
    # ============================================================

    def update_vertex(self, node):
        if node != self.goal:
            # RHS = min over successors
            min_cost = float('inf')
            for edge in self.graph.neighbors(node):
                succ = edge.to_node
                cost = cost_to_move(self.graph, node, succ)
                min_cost = min(min_cost, self.g[succ] + cost)

            self.rhs[node] = min_cost

        self.U.remove(node)

        if self.g[node] != self.rhs[node]:
            self.U.put(node, self.calculate_key(node))

    # ============================================================
    # COMPUTE SHORTEST PATH
    # ============================================================

    def compute_shortest_path(self):
        while (
            self.U.top_key() < self.calculate_key(self.start)
            or self.rhs[self.start] != self.g[self.start]
        ):
            k_old = self.U.top_key()
            u = self.U.get()
            k_new = self.calculate_key(u)

            if k_old < k_new:
                # Node's priority was outdated
                self.U.put(u, k_new)

            elif self.g[u] > self.rhs[u]:
                # Node has improved
                self.g[u] = self.rhs[u]
                for edge in self.graph.neighbors(u):
                    self.update_vertex(edge.to_node)

            else:
                # Node worsened, reset
                self.g[u] = float('inf')
                self.update_vertex(u)

                for edge in self.graph.neighbors(u):
                    self.update_vertex(edge.to_node)

    # ============================================================
    # EXTRACT CURRENT BEST PATH
    # ============================================================

    def get_path(self):
        path = [self.start]
        current = self.start

        while current != self.goal:
            min_cost = float('inf')
            next_node = None

            for edge in self.graph.neighbors(current):
                succ = edge.to_node
                c = cost_to_move(self.graph, current, succ)

                if c + self.g[succ] < min_cost:
                    min_cost = c + self.g[succ]
                    next_node = succ

            if next_node is None:
                return None  # No path

            current = next_node
            path.append(current)

        return path

    # ============================================================
    # WHEN GRAPH CHANGES
    # ============================================================

    def update_edge_cost(self, from_node, to_node, new_traffic_factor):
        """
        Called whenever traffic changes on a road segment.
        Updates the edge's traffic_factor and triggers a re-evaluation.
        """

        edge = self.graph.get_edge(from_node, to_node)
        if not edge:
            print(f"[D* Lite] WARNING: Edge {from_node}->{to_node} does not exist.")
            return

        # Update traffic factor
        edge.traffic_factor = new_traffic_factor

        # Recalculate RHS for the source node
        self.update_vertex(from_node)


    def replan(self, new_start):
        """
        Called whenever the robot moves OR A* detects a path change.
        """
        self.start = new_start
        self.compute_shortest_path()
        return self.get_path()
