import streamlit as st 
import networkx as nx
import matplotlib.pyplot as plt
import pandas as pd
import os
import json
from Model.Graph.graph import Graph
from Model.Graph.node import Node
from Model.Graph.edge import Edge
from Model.Utils.RoadTypes import RoadTypes
from Model.Utils.geo import haversine, cost_to_move, heuristic
from Model.Pathfinding.dynamic_search import DStarLite
from Model.Pathfinding.initial_search import AStar

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SAVE_PATH = os.path.join(BASE_DIR, "grafo.txt")


# ====================================================================
# FUNÇÃO: converter_para_networkx
# Converte o grafo do SEU modelo (Graph/Node/Edge) para um grafo do
# NetworkX, que é necessário para desenhar e rodar A*.
# ====================================================================
def converter_para_networkx(G):
    nxg = nx.DiGraph()  # cria grafo direcionado do NetworkX

    # adiciona todos os nós, com suas coordenadas
    for node_id, node_obj in G.nodes.items():
        nxg.add_node(node_id, pos=(node_obj.lon, node_obj.lat))

    count_edges = 0

    # percorre todas as arestas da sua estrutura Graph
    for origem, lista in G.adjacency.items():
        for edge in lista:

            from_node = G.nodes[edge.from_node]
            to_node = G.nodes[edge.to_node]

            # calcula distância real entre os pontos, se ainda não existir
            if getattr(edge, "distance", 0) and edge.distance > 0:
                dist = edge.distance
            else:
                dist = haversine(
                    from_node.lat, from_node.lon,
                    to_node.lat, to_node.lon
                )
                edge.distance = dist

            # peso da aresta = distancia × fator de trânsito
            peso = dist * edge.traffic_factor

            # insere aresta no NetworkX
            nxg.add_edge(
                edge.from_node,
                edge.to_node,
                weight=peso,
                traffic=edge.traffic_factor
            )
            count_edges += 1

    return nxg


# ====================================================================
# FUNÇÃO: salvar_grafo
# Salva todos os nós e arestas em arquivo grafo.txt
# Isso garante que o grafo persista entre execuções.
# ====================================================================
def salvar_grafo(G):
    try:
        data = {
            "nodes": {nid: {"lat": n.lat, "lon": n.lon} for nid, n in G.nodes.items()},
            "edges": [
                {"from": e.from_node, "to": e.to_node, "name": e.name}
                for lista in G.adjacency.values() for e in lista
            ]
        }

        with open(SAVE_PATH, "w") as f:
            json.dump(data, f)

    except Exception:
        pass


# ====================================================================
# FUNÇÃO: carregar_grafo
# Lê o arquivo grafo.txt e recria o grafo em memória.
# Se o arquivo não existir, começa com um grafo vazio.
# ====================================================================
def carregar_grafo():
    G = Graph()

    if os.path.exists(SAVE_PATH) and os.path.getsize(SAVE_PATH) > 0:
        try:
            with open(SAVE_PATH, "r") as f:
                data = json.load(f)

            # recria nós
            for nid, info in data["nodes"].items():
                if int(nid) not in G.nodes:
                    G.add_node(Node(int(nid), info["lat"], info["lon"]))

            # recria arestas
            for e in data["edges"]:
                if e["from"] in G.nodes and e["to"] in G.nodes:

                    from_node = G.nodes[e["from"]]
                    to_node = G.nodes[e["to"]]

                    dist = haversine(
                        from_node.lat, from_node.lon,
                        to_node.lat, to_node.lon
                    )

                    # recria aresta com dados padrões
                    G.add_edge(Edge(
                        name=e["name"],
                        from_node=e["from"],
                        to_node=e["to"],
                        distance=dist,
                        traffic_factor=1.0,
                        speed_limit=40,
                        type_road=RoadTypes.RESIDENTIAL,
                        one_way=False
                    ))

        except Exception:
            pass

    return G


# ====================================================================
# FUNÇÃO: resetar_grafo
# Apaga o arquivo grafo.txt e zera o grafo carregado.
# ====================================================================
def resetar_grafo():
    if os.path.exists(SAVE_PATH):
        os.remove(SAVE_PATH)
    st.session_state.G = Graph()
    if "nxg" in st.session_state:
        del st.session_state["nxg"]


# ====================================================================
# CONFIGURAÇÕES INICIAIS DA PÁGINA
# ====================================================================
st.set_page_config(page_title="Graph Pathfinding", layout="centered")
st.title("Grafo")


# ====================================================================
# BOTÃO: resetar grafo
# ====================================================================
if st.sidebar.button("Resetar Grafo"):
    resetar_grafo()
    st.sidebar.success("Grafo resetado.")


# ====================================================================
# CARREGA GRAFO NA PRIMEIRA EXECUÇÃO
# ====================================================================
if "G" not in st.session_state:
    st.session_state.G = carregar_grafo()
    try:
        st.session_state["nxg"] = converter_para_networkx(st.session_state.G)
    except Exception:
        pass

G: Graph = st.session_state.G

# ====================================================================
# PAINEL: ADICIONAR NÓ
# ====================================================================
st.sidebar.header("Adicionar Nó")
new_node_id = st.sidebar.number_input("ID do Nó", step=1)
lat = st.sidebar.number_input("Latitude (Y)", format="%.6f")
lon = st.sidebar.number_input("Longitude (X)", format="%.6f")

if st.sidebar.button("Adicionar Nó"):
    try:
        if int(new_node_id) in G.nodes:
            raise ValueError("Nó já existe no grafo.")
        G.add_node(Node(new_node_id, lat, lon))
        salvar_grafo(G)
        st.session_state["nxg"] = converter_para_networkx(G)
        st.sidebar.success(f"Nó '{new_node_id}' adicionado e salvo.")
    except Exception:
        pass

# ====================================================================
# PAINEL: ADICIONAR ARESTA
# ====================================================================
st.sidebar.header("Adicionar Aresta")

if len(G.nodes) >= 2:
    node_a = st.sidebar.selectbox("Nó A", list(G.nodes.keys()))
    node_b = st.sidebar.selectbox("Nó B", list(G.nodes.keys()))
else:
    node_a = node_b = None

if st.sidebar.button("Adicionar Aresta"):
    if node_a != node_b:
        if node_a in G.nodes and node_b in G.nodes:

            from_node = G.nodes[node_a]
            to_node = G.nodes[node_b]
            dist_calc = haversine(from_node.lat, from_node.lon, to_node.lat, to_node.lon)

            # adiciona ida
            G.add_edge(Edge(
                name=f"{node_a}_{node_b}",
                from_node=node_a,
                to_node=node_b,
                distance=dist_calc,
                traffic_factor=1.0,
                speed_limit=40,
                type_road=RoadTypes.RESIDENTIAL,
                one_way=False
            ))

            # adiciona volta (grafo bidirecional)
            G.add_edge(Edge(
                name=f"{node_b}_{node_a}",
                from_node=node_b,
                to_node=node_a,
                distance=dist_calc,
                traffic_factor=1.0,
                speed_limit=40,
                type_road=RoadTypes.RESIDENTIAL,
                one_way=False
            ))

            salvar_grafo(G)
            st.session_state["nxg"] = converter_para_networkx(G)
            st.sidebar.success(f"Aresta {node_a} ↔ {node_b} adicionada e salva.")
        else:
            st.sidebar.error("Erro: Um ou ambos os nós não existem no grafo.")


# ====================================================================
# PAINEL: ADICIONAR TRÂNSITO
# Esse trânsito NÃO salva no arquivo. Só existe na sessão atual.
# ====================================================================
st.sidebar.header("Adicionar Trânsito / Bloqueio")

if len(G.nodes) >= 2:
    traffic_a = st.sidebar.selectbox("Nó origem (trânsito)", list(G.nodes.keys()), key="traffic_a")
    traffic_b = st.sidebar.selectbox("Nó destino (trânsito)", list(G.nodes.keys()), key="traffic_b")
    traffic_factor = st.sidebar.slider("Fator de Trânsito", 1.0, 10.0, 1.10)
else:
    traffic_a = traffic_b = None

if st.sidebar.button("Aplicar Trânsito"):
    edge = G.get_edge(traffic_a, traffic_b)
    if edge:
        edge.traffic_factor = traffic_factor  # aumenta custo da aresta
        st.session_state["nxg"] = converter_para_networkx(G)
        st.sidebar.success(f"Trânsito aplicado em {traffic_a} -> {traffic_b} (x{traffic_factor})")
    else:
        st.sidebar.error("Aresta não existe")


# ====================================================================
# PAINEL: ORIGEM E DESTINO
# ====================================================================
st.sidebar.header("Origem e Destino")

if len(G.nodes) >= 2:
    origem = st.sidebar.selectbox("Origem", list(G.nodes.keys()), key="origem")
    destino = st.sidebar.selectbox("Destino", list(G.nodes.keys()), key="destino")
else:
    origem = destino = None


# ====================================================================
# BOTÃO: CALCULAR ROTA (A* + D* LITE)
# ====================================================================
rota = None
distancia_total = None

if st.sidebar.button("Calcular Rota"):
    try:

        # ======================================================
        # (A) PRIMEIRO: recalcula custo de TODAS as edges
        # ======================================================
        for lista in G.adjacency.values():
            for edge in lista:
                edge.cost = cost_to_move(G, edge.from_node, edge.to_node)

        # ======================================================
        # (B) A* USANDO SUA CLASSE
        # ======================================================
        astar = AStar(G)
        rota, distancia_total = astar.search(origem, destino)

        if rota is None:
            st.error("Nenhuma rota encontrada com A*.")
            raise Exception("Falha no A*")

        # ======================================================
        # (C) VERIFICA SE EXISTE TRÂNSITO
        # ======================================================
        existe = any(
            edge.traffic_factor > 1
            for lista in G.adjacency.values()
            for edge in lista
        )

        # ======================================================
        # (D) SE EXISTIR TRÂNSITO → D* LITE
        # ======================================================
        if existe:
            dstar = DStarLite(G, origem, destino)

            # atualiza custo de todas as arestas novamente no D*
            for lista in G.adjacency.values():
                for edge in lista:
                    novo = cost_to_move(G, edge.from_node, edge.to_node)
                    dstar.update_edge_cost(edge.from_node, edge.to_node, novo)

            nova = dstar.replan(origem)

            if nova:
                rota = nova
                distancia_total = sum(
                    cost_to_move(G, rota[i], rota[i+1])
                    for i in range(len(rota) - 1)
                )

        # ======================================================
        # (E) EXIBE RESULTADOS
        # ======================================================
        st.success(f"Rota encontrada: {rota}")
        st.info(f"Distância total: {distancia_total/1000:.2f} km")

    except Exception as e:
        st.error(f"Erro ao calcular rota: {e}")


# ====================================================================
# DESENHO DO GRAFO (NÓS, ARESTAS, TRÂNSITO E ROTA)
# ====================================================================
st.subheader("Grafo e Dados dos Nós")
col1, col2 = st.columns([2, 1])

nxg_draw = st.session_state.get("nxg", converter_para_networkx(G))

if len(nxg_draw.nodes) > 0:
    pos = nx.get_node_attributes(nxg_draw, "pos")

    with col1:
        plt.close("all")
        fig, ax = plt.subplots(figsize=(6, 5))

        # desenha os nós (círculos)
        nx.draw_networkx_nodes(
            nxg_draw, pos, node_color="lightblue", node_size=1200, ax=ax
        )

        # desenha os nomes dos nós
        nx.draw_networkx_labels(
            nxg_draw, pos, font_size=10, font_color="black", ax=ax
        )

        # ==========================================================
        # ORDEM DE DESENHO:
        # 1. Arestas normais (preto)
        # 2. Arestas com trânsito leve (amarelo)
        # 3. Arestas com trânsito pesado (vermelho)
        # 4. Rota encontrada (vermelho por cima de tudo)
        # ==========================================================

        # --- ARESTAS NORMAL (PRETO)
        for (u, v, data) in nxg_draw.edges(data=True):
            if data.get("traffic", 1.0) == 1:
                nx.draw_networkx_edges(
                    nxg_draw, pos,
                    edgelist=[(u, v)],
                    width=2,
                    edge_color="black",
                    ax=ax,
                    arrows=True,
                    arrowstyle="-|>",
                    arrowsize=18
                )

        # --- TRÂNSITO LEVE (AMARELO)
        for (u, v, data) in nxg_draw.edges(data=True):
            traf = data.get("traffic", 1.0)
            if 1 < traf < 8:
                nx.draw_networkx_edges(
                    nxg_draw, pos,
                    edgelist=[(u, v)],
                    width=3,
                    edge_color="yellow",
                    ax=ax,
                    arrows=True,
                    arrowstyle="-|>",
                    arrowsize=18
                )

        # --- TRÂNSITO PESADO
        for (u, v, data) in nxg_draw.edges(data=True):
            if data.get("traffic", 1.0) >= 8:
                nx.draw_networkx_edges(
                    nxg_draw, pos,
                    edgelist=[(u, v)],
                    width=4,
                    edge_color="yellow",
                    ax=ax,
                    arrows=True,
                    arrowstyle="-|>",
                    arrowsize=18
                )

        # --- ROTA FINAL (DESTAQUE)
        if rota:
            caminho = list(zip(rota[:-1], rota[1:]))
            nx.draw_networkx_edges(
                nxg_draw, pos,
                edgelist=caminho,
                width=4,
                edge_color="red",
                ax=ax,
                arrows=True,
                arrowstyle="-|>",
                arrowsize=22
            )

        # escreve valores dos pesos no meio das arestas
        for (u, v, data) in nxg_draw.edges(data=True):
            w = data["weight"]
            x1, y1 = pos[u]
            x2, y2 = pos[v]
            ax.text(
                (x1 + x2) / 2, (y1 + y2) / 2,
                f"{w/1000:.2f} km",
                fontsize=8,
                ha="center",
                va="center",
                color="black",
                bbox=dict(facecolor="white", alpha=0.7, edgecolor="none")
            )

        xs = [p[0] for p in pos.values()]
        ys = [p[1] for p in pos.values()]

        ax.set_xlim(min(xs) - 0.005, max(xs) + 0.005)
        ax.set_ylim(min(ys) - 0.005, max(ys) + 0.005)
        ax.set_axis_off()

        st.pyplot(fig)

    # tabela lateral com os dados dos nós
    with col2:
        df = pd.DataFrame({
            "ID": list(pos.keys()),
            "Latitude": [f"{p[1]:.6f}" for p in pos.values()],
            "Longitude": [f"{p[0]:.6f}" for p in pos.values()],
        })

        st.write("Dados dos Nós")
        st.dataframe(df.reset_index(drop=True), use_container_width=True)
