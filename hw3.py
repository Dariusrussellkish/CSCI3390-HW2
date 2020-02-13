import networkx as nx
from collections import deque
from copy import deepcopy
import matplotlib as mpl
import matplotlib.pyplot as plt
from numpy import sqrt
from numpy.random import choice, randint
from timeit import default_timer as timer 

def generate_random_flow_network(n, m=3, _min=1, _max=1000):
    G = nx.generators.barabasi_albert_graph(n, m).to_directed() 
    source = choice(G.nodes())

    sg = set()
    for frm, too in nx.dfs_edges(G, source):
        sg.add(frm)
        sg.add(too)

    target = choice(list(sg))
    while source == target:
        target = choice(list(sg))

    G = G.subgraph(sg)
    for frm, too in G.edges():
        G[frm][too]['capacity'] = randint(_min, _max+1)

    G = nx.relabel_nodes(G, {source: 'source', target:'target'})
    return G

def visualize_flow(G):
    pos = nx.layout.kamada_kawai_layout(G)
    nodes = nx.draw_networkx_nodes(G, pos, node_color='blue')
    edges = nx.draw_networkx_edges(G, pos, arrowstyle='->', arrowsize=10, width=2)
    nx.draw_networkx_labels(G, pos, font_size=12, font_family='sans-serif')

    edge_labels = dict([((u, v,), G.get_edge_data(u, v)['capacity']) for u, v in G.edges])
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

    plt.axis('off')
    plt.show()

def visualize(G, directed=True):
    if directed == True:
        pos = nx.layout.kamada_kawai_layout(G)
        nodes = nx.draw_networkx_nodes(G, pos, node_color='blue')
        edges = nx.draw_networkx_edges(G, pos, arrowstyle='->', arrowsize=10, width=2)
    else:
        nx.draw(G)
    ax = plt.gca()
    ax.set_axis_off()
    plt.show()

    
def bfs(G, source, **kwargs):
    visited = set() # Line 1
    queue = deque() # Line 3
    queue.append(source) # Line 4
    while queue:
        node = queue.popleft() # Line 6
        visited.add(node) # ~Line 9

        for child in G[node]: # Line 8
            if child in visited:
                continue
            else:
                queue.append(child)

    return visited

def bfs_path(G, source, target, **kwargs):
    visited = set() # Line 1
    queue = deque() # Line 3
    path_queue = deque()
    queue.append(source) # Line 4
    path_queue.append([source])
    while queue:
        node = queue.popleft() # Line 6
        path = path_queue.popleft()
        visited.add(node) # ~Line 9

        if node == target:
            return path

        for child in G[node]: # Line 8
            if child in visited:
                continue
            else:
                new_path = list(path)
                new_path.append(child)
                queue.append(child)
                path_queue.append(new_path)

    raise nx.NetworkXNoPath()


def ford_fulkerson(G, s, t, path_find=bfs_path, capacity='capacity', weight='weight'):
    flow = 0
    residual_graph = deepcopy(G) # Isolate Gf variable from G

    for frm, too in residual_graph.edges(): # Ensure Gf begins with no flow
        residual_graph[frm][too]['weight'] = 0

    try:
        path = path_find(residual_graph, s, target=t, weight=weight) # Find path in Gf from s to t

    except nx.NetworkXNoPath as e: # Handle NetworkX path finding algos
        return residual_graph, f

    while path is not None: # Handle my BFS implementation no Path
        # Below lines convert node path to edge path

        edges = []
        for i in range(1, len(path)):
            edges.append((path[i-1],path[i]))

        # Get minimum capacity in edge path
        f = min(list([residual_graph[frm][too][capacity] for frm,too in edges]))
        if f == 0:
            continue

        flow += f # Add flows

        # Modify Gf
        for frm, too in edges:
            residual_graph[frm][too][weight] += f # add flow
            residual_graph[frm][too][capacity] -= f # reduce capacity
            if residual_graph[frm][too][capacity] == 0: # if exhausted remove edge from Gf
                residual_graph.remove_edge(frm, too)
            residual_graph.add_edge(too, frm) # Add the negative edge
            try:
                residual_graph[too][frm][capacity]
            except KeyError:
                residual_graph[too][frm][capacity] = 0
            residual_graph[too][frm][capacity] += f # add the flow as the backflow capacity

        try:
            path = path_find(residual_graph, s, target=t, weight=weight) # Find path in Gf from s to t

        except nx.NetworkXNoPath:
            return residual_graph, f

    return residual_graph, flow


def benchmark(_max=100000000, step=2500, inner=25): 
    for n in range(10, _max, step):  
        for _ in range(inner):  
            G = generate_random_flow_network(n, m=choice([2,3,4,5]))  
            start = timer()  
            resid, f = ford_fulkerson(G, 'source', 'target')  
            end = timer()
            print(f"{G.number_of_nodes()},{G.number_of_edges()},{f},{end-start}")  

