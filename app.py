import streamlit as st
import networkx as nx
import matplotlib.pyplot as plt
import pandas as pd
import os
import json
from Model.Graph.graph import Graph        # Classe do grafo
from Model.Graph.node import Node          # Classe do nó
from Model.Graph.edge import Edge          # Classe da aresta
from Model.Utils.RoadTypes import RoadTypes  # Tipos de estrada
from Model.Utils.geo import haversine      # Função Haversine para distância

# --------------------------
# Caminho para salvar grafo em TXT (mesma pasta do app.py)
# --------------------------
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SAVE_PATH = os.path.join(BASE_DIR, "grafo.txt")

# --------------------------
# Salvar grafo em arquivo TXT (JSON)
# --------------------------
def salvar_grafo(G):
    data = {
        "nodes": {nid: {"lat": n.lat, "lon": n.lon} for nid, n in G.nodes.items()},
        "edges": [
            {"from": e.from_node, "to": e.to_node, "name": e.name} 
            for lista in G.adjacency.values() for e in lista
        ]
    }
    with open(SAVE_PATH, "w") as f:
        json.dump(data, f)

# --------------------------
# Carregar grafo do arquivo TXT
# --------------------------
def carregar_grafo():
    G = Graph()
    if os.path.exists(SAVE_PATH) and os.path.getsize(SAVE_PATH) > 0:
        with open(SAVE_PATH, "r") as f:
            data = json.load(f)
        # Adiciona nós
        for nid, info in data["nodes"].items():
            G.add_node(Node(int(nid), info["lat"], info["lon"]))
        # Adiciona arestas
        for e in data["edges"]:
            G.add_edge(Edge(
                name=e["name"],
                from_node=e["from"],
                to_node=e["to"],
                distance=0,
                traffic_factor=1.0,
                speed_limit=40,
                type_road=RoadTypes.RESIDENTIAL,
                one_way=False
            ))
    return G

# --------------------------
# Resetar grafo (apaga arquivo e reinicia sessão)
# --------------------------
def resetar_grafo():
    if os.path.exists(SAVE_PATH):
        os.remove(SAVE_PATH)
    st.session_state.G = Graph()

# --------------------------
# Configuração inicial da página
# --------------------------
st.set_page_config(page_title="Graph Pathfinding", layout="centered")
st.title("Grafo")

# --------------------------
# Botão lateral para resetar grafo
# --------------------------
if st.sidebar.button("Resetar Grafo"):
    resetar_grafo()
    st.sidebar.success("Grafo resetado.")

# --------------------------
# Carrega grafo na sessão
# --------------------------
if "G" not in st.session_state:
    st.session_state.G = carregar_grafo()

G: Graph = st.session_state.G

# --------------------------
# Converte Graph para NetworkX
# --------------------------
def converter_para_networkx(G):
    nxg = nx.DiGraph()
    # Adiciona nós
    for node_id, node_obj in G.nodes.items():
        nxg.add_node(node_id, pos=(node_obj.lon, node_obj.lat))  # X=lon, Y=lat
    # Adiciona arestas com distância via Haversine
    for origem, lista_arestas in G.adjacency.items():
        for edge in lista_arestas:
            from_node = G.nodes[edge.from_node]
            to_node = G.nodes[edge.to_node]
            distancia = haversine(from_node.lat, from_node.lon, to_node.lat, to_node.lon)
            nxg.add_edge(edge.from_node, edge.to_node, weight=distancia)
    return nxg

# --------------------------
# Sidebar - Adicionar Nó
# --------------------------
st.sidebar.header("Adicionar Nó")
new_node_id = st.sidebar.number_input("ID do Nó", step=1)
lat = st.sidebar.number_input("Latitude (Y)", format="%.6f")
lon = st.sidebar.number_input("Longitude (X)", format="%.6f")

if st.sidebar.button("Adicionar Nó"):
    try:
        G.add_node(Node(new_node_id, lat, lon))
        st.session_state.G = G
        salvar_grafo(G)
        st.sidebar.success(f"Nó '{new_node_id}' adicionado e salvo.")
    except Exception as e:
        st.sidebar.error(str(e))

# --------------------------
# Sidebar - Adicionar Aresta
# --------------------------
st.sidebar.header("Adicionar Aresta")
if len(G.nodes) >= 2:
    node_a = st.sidebar.selectbox("Nó A", list(G.nodes.keys()))
    node_b = st.sidebar.selectbox("Nó B", list(G.nodes.keys()))
else:
    node_a = node_b = None

if st.sidebar.button("Adicionar Aresta"):
    if node_a != node_b:
        if node_a in G.nodes and node_b in G.nodes:
            # Aresta bidirecional
            G.add_edge(Edge(name=f"{node_a}_{node_b}", from_node=node_a, to_node=node_b,
                            distance=0, traffic_factor=1.0, speed_limit=40,
                            type_road=RoadTypes.RESIDENTIAL, one_way=False))
            G.add_edge(Edge(name=f"{node_b}_{node_a}", from_node=node_b, to_node=node_a,
                            distance=0, traffic_factor=1.0, speed_limit=40,
                            type_road=RoadTypes.RESIDENTIAL, one_way=False))
            st.session_state.G = G
            salvar_grafo(G)
            st.sidebar.success(f"Aresta {node_a} ↔ {node_b} adicionada e salva.")
        else:
            st.sidebar.error("Erro: Um ou ambos os nós não existem no grafo.")

# --------------------------
# Sidebar - Escolher algoritmo e nós
# --------------------------
st.sidebar.header("Algoritmo")
algo = st.sidebar.radio("Escolha", ["A*", "D* Lite"])

if len(G.nodes) >= 2:
    origem = st.sidebar.selectbox("Origem", list(G.nodes.keys()))
    destino = st.sidebar.selectbox("Destino", list(G.nodes.keys()))
else:
    origem = destino = None

# --------------------------
# D* Lite simples (curto caminho)
# --------------------------
def dstar_lite_simple(g, start, goal):
    try:
        nxg = converter_para_networkx(g)
        return nx.shortest_path(nxg, start, goal, weight="weight")
    except:
        return None

# --------------------------
# Calcular rota
# --------------------------
rota = None
distancia_total = None

if st.sidebar.button("Calcular Rota"):
    if origem == destino:
        st.warning("Origem e destino iguais.")
    else:
        try:
            nxg = converter_para_networkx(G)
            if algo == "A*":
                rota = nx.astar_path(nxg, origem, destino, weight="weight")
            else:
                rota = dstar_lite_simple(G, origem, destino)
            # Calcula distância total
            if rota:
                distancia_total = sum(nxg[rota[i]][rota[i+1]]['weight'] for i in range(len(rota)-1))
            st.success(f"Rota encontrada: {rota}")
            if distancia_total:
                st.info(f"Distância total: {distancia_total/1000:.2f} km")
        except Exception as e:
            st.error(f"Erro ao calcular rota: {e}")

# --------------------------
# Exibir grafo e tabela
# --------------------------
st.subheader("Grafo e Dados dos Nós")
col1, col2 = st.columns([2, 1])
nxg_draw = converter_para_networkx(G)

if len(nxg_draw.nodes) > 0:
    pos = nx.get_node_attributes(nxg_draw, "pos")

    # -------- COLUNA 1 – GRAFO --------
    with col1:
        plt.close('all')
        fig, ax = plt.subplots(figsize=(6, 5))
        nx.draw(nxg_draw, pos, with_labels=True, node_color="lightblue",
                node_size=1200, ax=ax, font_size=10, font_color="black")
        nx.draw_networkx_edges(nxg_draw, pos, width=2, edge_color="black", ax=ax)
        if rota:
            edges_on_path = list(zip(rota[:-1], rota[1:]))
            nx.draw_networkx_edges(nxg_draw, pos, edgelist=edges_on_path,
                                   width=4, edge_color="red", ax=ax)
        # Desenha labels das arestas sem sobreposição
        for (u, v, d) in nxg_draw.edges(data='weight'):
            x1, y1 = pos[u]
            x2, y2 = pos[v]
            dx = x2 - x1
            dy = y2 - y1
            label_x = x1 + dx * 0.5 - dy * 0.01
            label_y = y1 + dy * 0.5 + dx * 0.01
            ax.text(label_x, label_y, f"{d/1000:.2f} km", fontsize=8,
                    ha='center', va='center',
                    bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))

        ax.set_axis_off()
        st.pyplot(fig)

    # -------- COLUNA 2 – TABELA --------
    with col2:
        dados = {"ID": [], "Latitude": [], "Longitude": []}
        for node_id, (lon, lat) in pos.items():
            dados["ID"].append(node_id)
            dados["Latitude"].append(f"{lat:.6f}")
            dados["Longitude"].append(f"{lon:.6f}")
        df = pd.DataFrame(dados)
        st.write("Dados dos Nós")
        st.dataframe(df.reset_index(drop=True), use_container_width=True)
