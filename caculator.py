import networkx as nx
import requests

base = "https://hackathon2025-dev.fpt.edu.vn/"
get = base + "api/maps/get_active_map/"
token = "c0435b497712ae6704789b28b710dc88"


def get_data():
    map = requests.get(get, params={"token": token, "map_type": "map_z"})
    return map.json()


def make_graph(edges):
    G = nx.DiGraph()

    for e in edges:
        G.add_edge(e["source"], e["target"], label=e["label"])

    return G


def find_path(G, start, end):
    path = nx.shortest_path(G, source=start["id"], target=end["id"])
    edge_labels = [G[u][v]["label"] for u, v in zip(path[:-1], path[1:])]
    return edge_labels


def problem_A():
    data = get_data()
    edges = data["edges"]
    nodes = data["nodes"]
    start = next((item for item in nodes if item["type"] == "Start"), None)
    end = next((item for item in nodes if item["type"] == "End"), None)

    G = make_graph(edges)
    path = find_path(G, start, end)
    return path