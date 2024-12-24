import networkx as nx
import matplotlib.pyplot as plt
import random
import time


def kuhn_algorithm(graph):
    left_nodes = [node for node, data in graph.nodes(data=True) if data['bipartite'] == 0]
    right_nodes = [node for node, data in graph.nodes(data=True) if data['bipartite'] == 1]

    match_right = {v: None for v in right_nodes}
    visited = set()

    def try_kuhn(node):
        if node in visited:
            return False
        visited.add(node)
        for neighbor in graph.neighbors(node):
            if match_right[neighbor] is None or try_kuhn(match_right[neighbor]):
                match_right[neighbor] = node
                return True
        return False

    for node in left_nodes:
        visited.clear()
        try_kuhn(node)

    matching = [(match_right[v], v) for v in right_nodes if match_right[v] is not None]
    return matching


def visualize_bipartite_graph(graph, matching=None, matching_greedy=None):
    left_nodes = [node for node, data in graph.nodes(data=True) if data['bipartite'] == 0]
    right_nodes = [node for node, data in graph.nodes(data=True) if data['bipartite'] == 1]

    pos = {}
    pos.update((node, (0, i)) for i, node in enumerate(left_nodes))
    pos.update((node, (1, i)) for i, node in enumerate(right_nodes))

    nx.draw(graph, pos, with_labels=True,
            node_color=['lightblue' if data['bipartite'] == 0 else 'lightgreen' for node, data in
                        graph.nodes(data=True)])

    if matching:
        nx.draw_networkx_edges(graph, pos, edgelist=matching, edge_color="red", width=4)
    if matching_greedy:
        nx.draw_networkx_edges(graph, pos, edgelist=matching_greedy, edge_color="blue", width=2)

    plt.show()


def dict_to_nxgraph(G):
    graph = nx.Graph()
    left_nodes = [key for key in G.keys()]
    right_nodes = set()
    for key in left_nodes:
        right_nodes.update(G[key].keys())

    graph.add_nodes_from(left_nodes, bipartite=0)
    graph.add_nodes_from(right_nodes, bipartite=1)
    edges = [(key, key2) for key in G.keys() for key2 in G[key].keys()]
    graph.add_edges_from(edges)
    return graph


def greedy_algorithm(graph):
    matching = set()
    used_nodes = set()

    for u in graph.nodes:
        if graph.nodes[u]['bipartite'] == 0 and u not in used_nodes:
            for v in graph.neighbors(u):
                if v not in used_nodes:
                    matching.add((u, v))
                    used_nodes.add(u)
                    used_nodes.add(v)
                    break
    return matching


def random_graph(num_edges):
    num_left = random.randint(1, num_edges)
    num_right = random.randint(1, num_edges)
    B = nx.bipartite.gnmk_random_graph(num_left, num_right, num_edges)
    return B


G = {
    'r1': {'z1': 1, 'z2': 1, 'z6': 1},
    'r2': {'z1': 1, 'z4': 1, 'z8': 1},
    'r3': {'z2': 1, 'z3': 1, 'z5': 1},
    'r4': {'z2': 1, 'z5': 1, 'z7': 1},
    'r5': {'z4': 1, 'z6': 1},
    'r6': {'z7': 1, 'z8': 1}
}

#graph = dict_to_nxgraph(G)
graph_r = random_graph(2000)
start_time_kuhn = time.time()
#matching = kuhn_algorithm(graph)
matching = kuhn_algorithm(graph_r)
end_time_kuhn = time.time() - start_time_kuhn

start_time_greedy = time.time()
#greedy_matching = greedy_algorithm(graph)
greedy_matching = greedy_algorithm(graph_r)
end_time_greedy = time.time() - start_time_greedy


print(f"Максимальное паросочетание (Алгоритм Куна) содержит {len(matching)} ребер")
print(matching)
print(f"Время выполнения алгоритма: {end_time_greedy}")

print(f"Максимальное паросочетание (Жадный алгоритм) содержит {len(greedy_matching)} ребер")
print(greedy_matching)
print(f"Время выполнения алгоритма: {end_time_kuhn}")

print(graph_r)
#visualize_bipartite_graph(graph, matching, greedy_matching)
