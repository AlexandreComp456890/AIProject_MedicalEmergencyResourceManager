from Model.Graph.node import Node
from Model.Graph.edge import Edge 

class Graph:
    def __init__(self):
        self.nodes: dict[int, Node] = {}           # node_id → Node
        self.adjacency: dict[int, list[Edge]] = {}       # node_id → list of Edge

    def add_node(self, node: Node):
        try:
            if node.id in self.nodes:
                raise ValueError("Nó já existe no grafo.")
            self.nodes[node.id] = node
            self.adjacency[node.id] = []
        except ValueError as e:
            print(f"Erro ao adicionar nó: {e}")

    def add_edge(self, edge: Edge) -> None:
        try:
            if edge.from_node not in self.adjacency:
                raise ValueError("Nó origem não existe.")
            self.adjacency[edge.from_node].append(edge)
        except ValueError as e:
            print(f"Erro ao adicionar aresta: {e}")

    def neighbors(self, node_id: int) -> list[Edge]:
        return self.adjacency.get(node_id, [])

    def get_node(self, node_id: int=0) -> Node:
        return self.nodes[node_id]

    def get_edge(self, from_node: int, to_node: int) -> Edge | None:
        for edge in self.adjacency.get(from_node, []):
            if edge.to_node == to_node:
                return edge
        return None