import heapq
import math
from Model.Utils.geo import haversine

class AStar:
    def __init__(self, graph):
        self.graph = graph

    def cost_to_move(self, from_node, to_node) -> float:
        edge = self.graph.get_edge(from_node, to_node)
        
        if edge:
            #Velocidade em m/s
            vmps = edge.speed_limit / 3.6
            time_cost = edge.distance / vmps
            return time_cost
        return math.inf
    
    def heuristic(self, node_a, node_b) -> float:
        edge = self.graph.get_edge(node_a, node_b)
        
        if edge:
            heuristic_cost = edge.distance / (edge.speed_limit / 3.6) * edge.traffic_factor
            return heuristic_cost
        return math.inf

    def search(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_score: dict[int, float] = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstrói caminho
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1], g_score[goal]

            for edge in self.graph.neighbors(current):
                neighbor = edge.to_node
                tentative_g = g_score[current] + self.cost_to_move(current, neighbor)  

                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    print(f"({current:<3} -> {neighbor:<3}) | f_score: ", f_score)
                    heapq.heappush(open_set, (f_score, neighbor))

        return None, 0