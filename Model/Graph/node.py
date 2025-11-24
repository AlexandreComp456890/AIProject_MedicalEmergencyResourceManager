class Node:
    def __init__(self, node_id: int, lat: float, lon: float, isObjective: bool = False):
        """
       Classe que representa um nó em um grafo de transporte.

        :param node_id: Identificador único do nó.
        :type node_id: int
        :param lat: latitude do nó.
        :type lat: float
        :param lon: longitude do nó
        :type lon: float
        """
        self.id = node_id
        self.lat = lat
        self.lon = lon
        self.isObjective = isObjective

    def __repr__(self):
        return f"Node({self.id}, lat={self.lat}, lon={self.lon} {', Objective' if self.isObjective else ''})"