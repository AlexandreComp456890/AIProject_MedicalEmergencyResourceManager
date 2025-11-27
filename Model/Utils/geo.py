import math

def haversine(lat1, lon1, lat2, lon2):
    R = 6371_000  # metros

    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)

    a = (math.sin(dlat/2)**2 +
         math.cos(math.radians(lat1)) *
         math.cos(math.radians(lat2)) *
         math.sin(dlon/2)**2)

    return 2 * R * math.asin(math.sqrt(a))

def cost_to_move(graph, from_node, to_node) -> float:
        edge = graph.get_edge(from_node, to_node)
        
        if edge:
            #Velocidade em m/s
            vmps = edge.speed_limit / 3.6
            time_cost = (edge.distance / vmps) * edge.traffic_factor
            return time_cost 
        return math.inf
    
def heuristic(graph, node_a, node_b) -> float:
     edge = graph.get_edge(node_a, node_b)
     
     if edge:
          heuristic_cost = edge.distance / (edge.speed_limit / 3.6) * edge.traffic_factor
          return heuristic_cost
     return math.inf