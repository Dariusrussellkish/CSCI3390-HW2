from collections import deque
from copy import deepcopy
from timeit import default_timer as timer

import matplotlib.pyplot as plt
import networkx as nx
from numpy.random import choice, randint

# Darius Russell Kish


def generate_random_flow_network(n, c_min=1, c_max=1000, **kwargs):
    """ Generate a reasonable random flow network

    For now, uses barabasi_albert_graph converted to directed and
    randomly prunes edges to reduce reflexive edges. A random source
    is picked and its reachable subgraph is used. A random target != source is
    picked, and capacities are assigned to edges. The source node and target
    node are renamed 'source' and 'target' respectively.

    Args:
        n (int): size of the initial G
        c_min (int, optional): the minimum edge capacity
        c_max (int, optional): the maximum edge capacity
        **kwargs: arbitrary keyword arguments.
            Can be used to pass m to barabasi_albert_graph.
    Returns:
        networkx.DiGraph
    """

    G = nx.generators.barabasi_albert_graph(n, **kwargs).to_directed()
    source = choice(G.nodes())

    for frm, too in list(G.edges()):  # remove random edges with p=2/3
        if randint(0, 3) > 1:
            G.remove_edge(frm, too)

    sg = set()
    for frm, too in nx.dfs_edges(G, source):
        sg.add(frm)
        sg.add(too)

    sg = list(sg)

    while sg == []:
        G = nx.generators.barabasi_albert_graph(n, **kwargs).to_directed()
        source = choice(G.nodes())

        for frm, too in list(G.edges()):  # remove random edges with p=2/3
            if randint(0, 3) > 1:
                G.remove_edge(frm, too)

        sg = set()
        for frm, too in nx.dfs_edges(G, source):
            sg.add(frm)
            sg.add(too)
        sg = list(sg)

    target = choice(sg)
    while source == target:
        target = choice(sg)

    G = G.subgraph(sg)
    for frm, too in G.edges():
        G[frm][too]['capacity'] = randint(c_min, c_max + 1)

    G = nx.relabel_nodes(G, {source: 'source', target: 'target'})
    return G


def visualize_flow(G, capacity='capacity', **kwargs):
    pos = nx.layout.spring_layout(G, k=2, iterations=250, **kwargs)
    nodes = nx.draw_networkx_nodes(G, pos, node_color='blue', **kwargs)
    edges = nx.draw_networkx_edges(G,
                                   pos,
                                   arrowstyle='->',
                                   arrowsize=10,
                                   width=2,
                                   **kwargs)
    nx.draw_networkx_labels(G,
                            pos,
                            font_size=12,
                            font_family='sans-serif',
                            **kwargs)

    try:
        edge_labels = {(u, v): G.get_edge_data(u, v)[capacity]
                       for (u, v) in G.edges}
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, **kwargs)
    except KeyError:
        pass

    plt.axis('off')
    plt.show()


def read_graph(filename, visualize=visualize_flow, **kwargs):
    """ Read in an EdgeList

    Highly adaptable EdgeList reader.

    Note:
        The default visualize_flow visualizer expects capacities in
        an edge attribute 'capacities', this can be changed with kwargs.

    Args:
        filename (path-like): path to edge-list file
        visualize (optional): visualization function, passes kwargs.
            None skips visualization.
        **kwargs: arbitrary keyword arguments.
            Can be used to pass to visualize or read_edgelist
    Returns:
        networkx.DiGraph
    """

    G = nx.read_edgelist(filename, **kwargs, create_using=nx.DiGraph)
    if visualize is not None:
        visualize(G, **kwargs)
    return G


def bfs(G, source, **kwargs):
    visited = set()  # Line 1
    queue = deque()  # Line 3
    queue.append(source)  # Line 4
    while queue:
        node = queue.popleft()  # Line 6
        visited.add(node)  # ~Line 9

        for child in G[node]:  # Line 8
            if child in visited:
                continue
            queue.append(child)

    return visited


def bfs_path(G, source, target, **kwargs):
    visited = set()  # Line 1
    queue = deque()  # Line 3
    path_queue = deque()
    queue.append(source)  # Line 4
    path_queue.append([source])
    while queue:
        node = queue.popleft()  # Line 6
        path = path_queue.popleft()
        visited.add(node)  # ~Line 9

        if node == target:
            return path

        for child in G[node]:  # Line 8
            if child in visited:
                continue
            new_path = list(path)
            new_path.append(child)
            queue.append(child)
            path_queue.append(new_path)

    raise nx.NetworkXNoPath()


def ford_fulkerson(G,
                   s,
                   t,
                   path_find=bfs_path,
                   capacity='capacity',
                   flowkey='weight',
                   weight='capacity',
                   **kwargs):
    """ Implementation of the iterative ford-fulkerson algorithm

    Iterative residual graph generation until no path from s to t is found.
    Utilizes path finding functions in the form of NetworkX path finding functions.
    Works at least with dijkstra_path and bfs_path. The path_find function needs
    to return a path and take a Graph and the source/sink nodes.

    Args:
        G (NetworkX.DiGraph): a DiGraph that conforms to the definition of a flow
            network. Assumes edge attributes have 'capacity' initialized.
        s: source node key in G
        t: target node kye in G
        path_find (function, optional): path finding algorithm.
            Default is the bfs_path implemented in this file.
        capacity (optional): capacity edge attribute key
        weight (optional): flow edge attribute key
        **kwargs: arbitrary keyword arguments

    Returns:
        Gf: the final residual graph
        f: the maximum flow
    """
    flow = 0
    residual_graph = deepcopy(G)  # Isolate Gf variable from G

    for frm, too in residual_graph.edges():  # Ensure Gf begins with no flow
        residual_graph[frm][too][flowkey] = 0

    try:
        path = path_find(residual_graph, s, target=t, weight=weight,
                         **kwargs)  # Find path in Gf from s to t

    except nx.NetworkXNoPath:  # Handle NetworkX path finding algos
        return flow

    while path is not None:  # Handle my BFS implementation no Path
        # Below lines convert node path to edge path

        edges = []
        for i in range(1, len(path)):
            edges.append((path[i - 1], path[i]))

        # Get minimum capacity in edge path
        f = min(
            list([residual_graph[frm][too][capacity] for frm, too in edges]))

        flow += f  # Add flows

        # Modify Gf
        for frm, too in edges:
            try:
                residual_graph[frm][too][flowkey] += f  # add flow
            except KeyError:
                residual_graph[frm][too][flowkey] = f
            residual_graph[frm][too][capacity] -= f  # reduce capacity
            if residual_graph[frm][too][
                    capacity] == 0:  # if exhausted remove edge from Gf
                residual_graph.remove_edge(frm, too)
            residual_graph.add_edge(too, frm)  # Add the negative edge
            try:
                residual_graph[too][frm][capacity]
            except KeyError:
                residual_graph[too][frm][capacity] = 0
            residual_graph[too][frm][
                capacity] += f  # add the flow as the backflow capacity

        try:
            path = path_find(residual_graph, s, target=t,
                             weight=weight)  # Find path in Gf from s to t

        except nx.NetworkXNoPath:
            return flow

    return flow


def benchmark(_max=100000, step=250, inner=25, **kwargs):
    for n in range(10, _max, step):
        for _ in range(inner):
            G = generate_random_flow_network(n, m=choice([2, 3, 4, 5]))
            start = timer()
            f = ford_fulkerson(G, 'source', 'target', **kwargs)
            end = timer()
            f_ref = nx.maximum_flow_value(G, 'source', 'target')
            if f != f_ref:
                raise ValueError(
                    f"Calculated flow: {f} does not match {f_ref}")
            print(
                f"{G.number_of_nodes()},{G.number_of_edges()},{f},{end-start}")


if __name__ == "__main__":
    # G = generate_random_flow_network(10, m=3)
    # #visualize_flow(G)
    # Gf, f = ford_fulkerson(G,
    #                        'source',
    #                        'target',
    #                        path_find=nx.dijkstra_path,
    #                        flowkey='weight')
    # f_ref = nx.maximum_flow_value(G, 'source', 'target')
    # print(f, f_ref)
    # #visualize_flow(Gf)
    print("DIJKSTRA")
    benchmark(path_find=nx.dijkstra_path)
    print("BFS")
    benchmark()
