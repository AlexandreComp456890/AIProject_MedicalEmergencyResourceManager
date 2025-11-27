import heapq
import math
from Model.Utils.geo import cost_to_move, heuristic

class AStar:
    def __init__(self, graph):
        self.graph = graph

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
                tentative_g = g_score[current] + cost_to_move(self.graph, current, neighbor)  

                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(self.graph, neighbor, goal)
                    print(f"({current:<3} -> {neighbor:<3}) | f_score: ", f_score)
                    heapq.heappush(open_set, (f_score, neighbor))

        return None, 0