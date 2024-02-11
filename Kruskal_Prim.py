import heapq

# Structure to represent an edge
class Edge:
    def __init__(self, src, dest, weight):
        self.src = src
        self.dest = dest
        self.weight = weight

# Function to add an edge to the graph
def addEdge(graph, src, dest, weight):
    graph[src].append((dest, weight))
    graph[dest].append((src, weight))

# Function to find the parent of a vertex in a disjoint set
def findParent(parent, vertex):
    if parent[vertex] == -1:
        return vertex
    return findParent(parent, parent[vertex])

# Function to perform union of two disjoint sets
def performUnion(parent, x, y):
    xParent = findParent(parent, x)
    yParent = findParent(parent, y)
    parent[xParent] = yParent

# Function to run Kruskal's algorithm
def kruskalAlgorithm(graph, numVertices):
    edges = []  # To store all the edges of the graph
    mst = []    # To store the minimum spanning tree

    # Populate the edges list with all the edges in the graph
    for u in range(numVertices):
        for edge in graph[u]:
            v, weight = edge
            if u < v:
                edges.append(Edge(u, v, weight))

    # Sort the edges in ascending order of weights
    edges.sort(key=lambda x: x.weight)

    parent = [-1] * numVertices  # To track the parent of each vertex

    for edge in edges:
        u = edge.src
        v = edge.dest

        uParent = findParent(parent, u)
        vParent = findParent(parent, v)

        # Check if including this edge creates a cycle
        if uParent != vParent:
            # Include the edge in the minimum spanning tree
            mst.append(edge)
            # Perform union of the two disjoint sets
            performUnion(parent, uParent, vParent)

    # Print the Minimum Spanning Tree (MST)
    print("Minimum Spanning Tree (MST) using Kruskal's algorithm:")
    for edge in mst:
        print(f"{edge.src} - {edge.dest} : {edge.weight}")

# Function to run Prim's algorithm
def primAlgorithm(graph, numVertices, startVertex):
    mst = [[] for _ in range(numVertices)]  # To store the MST
    key = [float('inf')] * numVertices      # To store the minimum weights
    visited = [False] * numVertices         # To track visited vertices
    parent = [-1] * numVertices             # To store the MST

    pq = []  # Priority queue
    heapq.heappush(pq, (0, startVertex))

    # Initialize the starting vertex
    key[startVertex] = 0

    while pq:
        _, u = heapq.heappop(pq)
        visited[u] = True

        for edge in graph[u]:
            v, weight = edge

            if not visited[v] and weight < key[v]:
                parent[v] = u
                key[v] = weight
                heapq.heappush(pq, (key[v], v))

    # Construct the MST adjacency list
    for i in range(numVertices):
        if i == startVertex :
            continue
        mst[parent[i]].append((i, key[i]))

    # Print the Minimum Spanning Tree (MST)
    print("Minimum SpanningTree (MST) using Prim's algorithm:")
    for i in range(numVertices):
        for edge in mst[i]:
            print(f"{i} - {edge[0]} : {edge[1]}")

while True:
    numVertices = 6  # Total number of vertices in the graph
    startVertex = 0  # Starting vertex for Prim's algorithm

    graph = [[] for _ in range(numVertices)]  # Graph adjacency list

    addEdge(graph, 0, 1, 7)
    addEdge(graph, 0, 2, 8)
    addEdge(graph, 1, 2, 3)
    addEdge(graph, 1, 3, 6)
    addEdge(graph, 2, 3, 4)
    addEdge(graph, 2, 4, 3)
    addEdge(graph, 3, 4, 2)
    addEdge(graph, 3, 5, 5)
    addEdge(graph, 4, 5, 2)
    print("================================================================================")
    print("Choose the algorithm to find the minimum spanning tree:")
    print("1. Prim's algorithm")
    print("2. Kruskal's algorithm")
    print("3. Exit")
    choice = int(input("Enter your choice : "))

    if choice == 1:
        startVertex = int(input(f"Enter the starting vertex (0 to {numVertices - 1}): "))
        primAlgorithm(graph, numVertices, startVertex)
    elif choice == 2:
        kruskalAlgorithm(graph, numVertices)
    elif choice == 3:
        print("Bye!")
        break
    else:
        print("Invalid choice!")
