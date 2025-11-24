from Model.Graph.graph import Graph
from Model.Graph.node import Node
from Model.Graph.edge import Edge
from Model.Utils.geo import haversine
from Model.Pathfinding.initial_search import AStar
import random


def montar_exemplo():

    random.seed(45)
    # Base da cidade (região próxima a SP)
    base_lat = -23.5500
    base_lon = -46.6300

    g = Graph()

    # Criar uma grid de ~60 nós (6 avenidas x 10 ruas)
    num_avenidas = 6
    num_ruas = 10

    node_id = 1
    nodes_map = {}  # (i,j) → node_id

    for i in range(num_avenidas):
        for j in range(num_ruas):

            # Pequeno offset realista (~70–120 metros por quadra)
            lat = base_lat + (i * 0.0010)
            lon = base_lon + (j * 0.0012)

            node = Node(node_id, lat, lon)
            g.add_node(node)
            nodes_map[(i, j)] = node_id

            node_id += 1

    # Criar arestas entre nodes adjacentes
    for i in range(num_avenidas):
        for j in range(num_ruas):

            node = nodes_map[(i, j)]

            # Conecta com rua da direita
            if j + 1 < num_ruas:
                node2 = nodes_map[(i, j+1)]
                n1 = g.get_node(node)
                n2 = g.get_node(node2)
                dist = haversine(n1.lat, n1.lon, n2.lat, n2.lon)

                # Ruas leste-oeste
                name = f"Rua {i+1}.{j+1}"

                # custo variado simulando vias melhores/piores
                traffic = random.uniform(1.0, 2.0)
                speed = random.choice([40, 50, 60])

                g.add_edge(Edge(name, node, node2, dist, traffic_factor=traffic, speed_limit=speed))
                g.add_edge(Edge(name, node2, node, dist, traffic_factor=traffic, speed_limit=speed))

            # Conecta com avenida abaixo
            if i + 1 < num_avenidas:
                node2 = nodes_map[(i+1, j)]
                n1 = g.get_node(node)
                n2 = g.get_node(node2)
                dist = haversine(n1.lat, n1.lon, n2.lat, n2.lon)

                # Avenidas norte-sul
                name = f"Avenida {j+1}"

                traffic = random.uniform(1.0, 1.4)
                speed = random.choice([50, 60, 70])

                g.add_edge(Edge(name, node, node2, dist, traffic_factor=traffic, speed_limit=speed))
                g.add_edge(Edge(name, node2, node, dist, traffic_factor=traffic, speed_limit=speed))

    # Criar alguns retornos diagonais para dar alternativas
    # (estilo pontes, túneis ou curvas importantes)
    diagonais = [
        ((0,0),(1,1)), ((2,3),(3,4)), ((4,5),(5,6)),
        ((1,7),(2,8)), ((3,2),(4,3)), ((0,5),(1,6))
    ]

    for (a, b) in diagonais:
        n1 = nodes_map[a]
        n2 = nodes_map[b]

        p1 = g.get_node(n1)
        p2 = g.get_node(n2)

        dist = haversine(p1.lat, p1.lon, p2.lat, p2.lon)

        nome = "Diagonal Especial"

        g.add_edge(Edge(nome, n1, n2, dist, traffic_factor=1.0, speed_limit=70))
        g.add_edge(Edge(nome, n2, n1, dist, traffic_factor=1.0, speed_limit=70))

    return g



test_graph = montar_exemplo()
print(test_graph.nodes)
print(test_graph.adjacency)

if __name__ == "__main__":
    graph = montar_exemplo()
    astar = AStar(graph)
    path, amount = astar.search(1, 60)
    print("Caminho encontrado:", path, amount) if path else print("Caminho não encontrado.")