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
    "I5": (0.6, 3.4),
    "I6": (2.8, 1.2),
    "I7": (2, 1),
    "I8": (3.2, 0.2),
    "I9": (1.6, 0.5)
    
}
EDGES = [
    ("0", "2"),
    ("0", "3"),
    ("0", "I1"),
    ("0", "I3"),
    ("0", "7"),
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
    ("3", "8"),
    ("4", "9"),
    ("I6", "3"),
    ("I7", "2"),
    ("I7", "0"),
    ("I8", "2"),
    ("I8", "8"),
    ("5", "I4"),
    ("I4", "I9"),
    ("I9", "2"),
    ("I6", "0")
]

def init_graph():
    G = nx.Graph()
    weighted_edges = []
    for (a, b) in EDGES:
        dist = norm(NODES[a], NODES[b])
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

def norm(A, B):
    (x_a, y_a) = A
    (x_b, y_b) = B
    dist = np.sqrt((x_a - x_b) ** 2 + (y_a - y_b) ** 2)
    return dist

def best_match(position, nodes):
    min = float("inf")
    k_m = None
    for (node, pos) in nodes.items():
        dist = norm(position, pos)
        if dist < min:
            k_m = node
            min = dist
    return k_m

def plan(origin_pos, goal_pos):
    origin = best_match(origin_pos, NODES)
    target = best_match(goal_pos, NODES)
    return path(origin, target)

def test():
    test_best_match()
    test_plan()

def test_plan():
    for k in NODES.keys():
        print "testing " + k
        result = plan(NODES[k], NODES[k]) == [NODES[k]]
        assert result
        print result
    
def test_best_match():
    for k in NODES.keys():
        result = best_match(NODES[k], NODES) == k
        assert result
        print result
    for k in NODES.keys():
        result = best_match(NODES[k], NODES) == k
        assert result
        print result
        
