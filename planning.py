import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

NODES = {
    "0": (2.2, 1.6),
    "2": (2.6, 0.6),
    "3": (3.4, 1.4),
    "4": (2.4, 3.4),
    "5": (0.6, 2.2),
    "6": (1.4, 3.2),
    "7": (1, 1.6),
    "8": (3.6, 0.6),
    "9": (3.2, 3.2),
    "I1": (2.6, 2.6),
    "I2": (3.6, 2.4),
    "I3" : (1.6, 2),
    "I4": (0.5, 0.5),
    "I5": (0.6, 3.4)
}
EDGES = [
    ("0", "2"),
    ("0", "3"),
    ("0", "I1"),
    ("0", "I3"),
    ("0", "7"),
    ("2", "I4"),
    ("I4", "7"),
    ("7", "I3"),
    ("7", "5"),
    ("5", "I3"),
    ("5", "6"),
    ("5", "I5"),
    ("I5", "6"),
    ("6", "I3"),
    ("6", "4"),
    ("4", "I1"),
    ("I1", "9"),
    ("I1", "I3"),
    ("I1", "I2"),
    ("I2", "9"),
    ("I2", "3"),
    ("3", "8")
]

def init_graph():
    G = nx.Graph()
    weighted_edges = []
    for (a, b) in EDGES:
        (x_a, y_a) = NODES[a]
        (x_b, y_b) = NODES[b]
        dist = np.sqrt((x_a - x_b) ** 2 + (y_a - y_b) ** 2)
        weighted_edges.append((a, b, {'w': dist}))
        G.add_edges_from(weighted_edges);
    return G


def path(origin, target):
    G = init_graph()
    path_nodes = nx.shortest_path(G, origin, target, "w")
    path = []
    for node in path_nodes:
        path.append(NODES[node])
    return path
