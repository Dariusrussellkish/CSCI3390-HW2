# CSCI3390-HW2
- Darius Russell Kish (contributing), russeldk [at] bc.edu
- Yuezhen Chen
- Jessica Fong Ng
- Matthew Uy

There were very few instructions on how to structure this pset – I am including
a \_\_init\_\_.py file so that the functions can be imported into a testbench.

## Part 1
>Create a program that takes user’s input and then produce and visualize a directed
network using NetworkX. User input is specified in the form of adjacency matrix.

The function _read_graph_() takes an __edge list__ formatted input file and
generates a _NetworkX.DiGraph_. There is a keyword _visualize_ which controls
the visualization function, if set to _None_ then there is no visualization.

## Part 2
>Implement the BFS we learned in class (Note1). In your code, you need to add
comments to indicate which line of your code corresponds to a particular line of
pseudo-code presented in Note1.

The function _bfs_() most strictly follows the algorithm described for BFS
Explore. The algorithm is the same nearly line for line, except some syntax
and cleanup that reorder non-sequential operations.

A function _bfs_path()_ is utilized in the Ford-Fulkerson implementation. It
utilizes _NetworkX.bfs_tree()_ converted into a reversed _DiGraph_, and then
follows the single path from _target_ to _source_. This is the most efficient
representation of a BFS path find in Python NetworkX, since a path-storing
queue based system is bottlenecked by path deepcopies.

## Part 3 & 4
> Implement the iterative version of the Ford-Fulkerson algorithm presented in class. In
your code, you need to add comments to indicate which line of your code corresponds
to a particular line of pseudo-code presented in Note3. In your algorithm, use BFS
implemented in the previous step to find the path in each iteration.

The function _ford_fulkerson()_ is a general purpose Ford-Fulkerson function that
can take in an arbitrary path-finding algorithm, in addition to custom naming
of flow, capacity, and weight attribute names for the path finding algo.

Path finding algorithms currently known to work are my _bfs_path()_ and
_NetworkX.dijkstra_path()_. Other path finding algorithms should take a graph G,
a source and target node as keywords, and arbitrary keyword arguments. They must
return the path as a _List of Nodes_, _None_, or _NetworkX.NetworkXNoPath_ error.

## Benchmarking

Benchmarking was done using Intel Xeon Gold 6148 CPU @ 2.40GHz on the BC Research
Cluster for 8 hours until the job was terminated. Timing was done just on
the Ford-Fulkerson function call using python's _timeit_ module, and N, M, and F
were tracked.

### Random Flow Network Generation

We generated Random Flow Networks based on an initial Barabasi-Albert undirected
graph with _m_ in [2,3,4,5]. This graph was converted to a DiGraph. The resulting
graph had edges deleted with a probability of 1/3 for edge deletion. A random
_source_ was picked from the node set, and all reachable nodes were calculated.
If rch(_source_) was empty, then the generation process was restarted. From
that a random target was picked from rch(_source_), and the graph was shrunk to
the subset of rch(_source_) nodes. Every edge was then assigned a random capacity
within some range, and the graph was returned. Despite seeding the generation
with some initial N for Barabasi-Albert, there is no guarantee the resultant
graph will be proportional to N.


Our benchmark tested graphs with seed N size of 10 to ~100000 (walltime limit
hit before this), with step sizes of 250 nodes and 25 replicates per step size.


### BFS Benchmarking

Below are the distributions of N, M and F in the BFS runs.

<div style="text-align:center"><img src=bfs_node_dist.png width=400/></div>
<div style="text-align:center"><img src=bfs_edge_dist.png width=400/></div>
<div style="text-align:center"><img src=bfs_flow_dist.png width=400/></div>

We know that the _bfs_path()_ algorithm runs in <img src="https://render.githubusercontent.com/render/math?math=O(nm)"> time, so that makes
our Ford-Fulkerson using it equivalent to Edmonds-Karp, which has time complexity
of <img src="https://render.githubusercontent.com/render/math?math=O(nm^2)">.
We however do not observe this behavior, and instead see a runtime of
<img src="https://render.githubusercontent.com/render/math?math=O(nmf)">, which
is the same as normal Ford-Fulkerson. We are unsure why, since our BFS
path-finding algorithm utilizes <img src="https://render.githubusercontent.com/render/math?math=O(n+m)"> time and thus should be equivalent to Edmonds-Karp.



### Dijksta Benchmarking

Below are the distributions of N, M and F in the Dijkstra runs.

<div style="text-align:center"><img src=dijkstra_node_dist.png width=400/></div>
<div style="text-align:center"><img src=dijkstra_edge_dist.png width=400/></div>
<div style="text-align:center"><img src=dijkstra_flow_dist.png width=400/></div>

The current implementation of _NetworkX.dijkstra_path()_ will find the _minimum_
capacity path from _source_ to _target_ – this is similar to forcing the worst
case of the normal Ford-Fulkerson and we expect the runtime to be
<img src="https://render.githubusercontent.com/render/math?math=O(mf)"> in this
case as we are dominated by f.

<div style="text-align:center"><img src=dijkstra_complexity.png width=400/></div>
