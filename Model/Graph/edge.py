from Model.Utils.RoadTypes import RoadTypes
from Model.Utils.geo import haversine


class Edge:
    def __init__(self, name: str, from_node: int, to_node: int, distance: float, 
                 traffic_factor: float = 1.0, speed_limit: int = 40, 
                 type_road: RoadTypes = RoadTypes.RESIDENTIAL, one_way: bool = False):
        """
        Classe que representa uma aresta em um grafo de transporte.
        
        :param name: Nome da estrada ou rota.
        :type name: str
        :param from_node: Nó de origem da aresta.
        :type from_node: int
        :param to_node: Nó de destino da aresta.
        :type to_node: int
        :param distance: Distância entre os nós.
        :type distance: float
        :param traffic_factor: Fator de tráfego que afeta o custo da aresta. Padrão é 1.0 (sem tráfego).
        :type traffic_factor: Optional[float]
        :param speed_limit: Limite de velocidade na estrada. Padrão é 40.0 Km/h.
        :type speed_limit: Optional[int]
        :param type_road: Tipo da estrada (ex: residencial, rodovia). Padrão é "residential".
        :type type_road: Optional[]
        :param one_way: Indica se a estrada é mão única. Padrão é False (via de mão dupla).
        :type one_way: Optional[bool]
        """
        self.name = name
        self.from_node = from_node
        self.to_node = to_node
        self.distance = distance
        self.traffic_factor = traffic_factor
        self.speed_limit = speed_limit
        self.type_road = type_road
        self.one_way = one_way  
    
    def __repr__(self):
        return f"{self.name} Edge({self.from_node} -> {self.to_node}, Distância = {self.distance:.5f}m)"
