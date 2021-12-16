# Graph source from https://www.geeksforgeeks.org/vertex-cover-problem-set-1-introduction-approximate-algorithm-2/
# Weighted Graph source from https://www.bogotobogo.com/python/python_graph_data_structures.php

# Python3 program to print Vertex Cover
# of a given undirected graph
from collections import defaultdict
import math
 
# This class represents a directed graph
# using adjacency list representation
class Graph:
 
    def __init__(self, vertices):
         
        # No. of vertices
        self.V = vertices
         
        # Default dictionary to store graph
        self.graph = defaultdict(list)
 
    # Function to add an edge to graph
    def addEdge(self, u, v):
        self.graph[u].append(v)
 
    # The function to print vertex cover
    def getVertexCover(self):
         
        # Initialize all vertices as not visited.
        visited = [False] * (self.V)
         
        # Consider all edges one by one
        for u in range(self.V):
             
            # An edge is only picked when
            # both visited[u] and visited[v]
            # are false
            if not visited[u]:
                 
                # Go through all adjacents of u and
                # pick the first not yet visited
                # vertex (We are basically picking
                # an edge (u, v) from remaining edges.
                for v in self.graph[u]:
                    if not visited[v]:
                         
                        # Add the vertices (u, v) to the
                        # result set. We make the vertex
                        # u and v visited so that all
                        # edges from/to them would
                        # be ignored
                        visited[v] = True
                        visited[u] = True
                        break
 
        # Print the vertex cover
        mvc_set = []
        for j in range(self.V):
            if visited[j]:
                mvc_set.append(j)
        
        # print("VC set", mvc_set)
        # print("MVC size lower bound", math.ceil(len(mvc_set)/2))
        return mvc_set


 
    def __init__(self, vertices):
         
        # No. of vertices
        self.V = vertices
         
        # Default dictionary to store graph
        self.graph = defaultdict(list)
        self.weights = defaultdict(list)
 
    # Function to add an edge to graph
    def addEdge(self, u, v, weight):
        self.graph[u].append(v)
        self.weights.append((u,v),weight)
 
    # The function to print vertex cover
    def getVertexCover(self):
         
        # Initialize all vertices as not visited.
        visited = [False] * (self.V)
         
        # Consider all edges one by one
        for u in range(self.V):
             
            # An edge is only picked when
            # both visited[u] and visited[v]
            # are false
            if not visited[u]:
                 
                # Go through all adjacents of u and
                # pick the first not yet visited
                # vertex (We are basically picking
                # an edge (u, v) from remaining edges.
                for v in self.graph[u]:
                    if not visited[v]:
                         
                        # Add the vertices (u, v) to the
                        # result set. We make the vertex
                        # u and v visited so that all
                        # edges from/to them would
                        # be ignored
                        visited[v] = True
                        visited[u] = True
                        print()
                        break
 
        # Print the vertex cover
        mvc_set = []
        for j in range(self.V):
            if visited[j]:
                mvc_set.append(j)
        
        # print("VC set", mvc_set)
        # print("MVC size lower bound", math.ceil(len(mvc_set)/2))
        return mvc_set

class Vertex:
    def __init__(self, node):
        self.id = node
        self.adjacent = {}

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])

    def add_neighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def get_connections(self):
        return self.adjacent.keys()  

    def get_id(self):
        return self.id

    def get_weight(self, neighbor):
        return self.adjacent[neighbor]

class WeightedGraph:
    def __init__(self, vertices):
        self.vert_dict = {}
        self.num_vertices = 0
        for vertex in vertices:
            self.add_vertex(vertex)

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node)
        self.vert_dict[node] = new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def add_edge(self, frm, to, cost = 0):
        if frm not in self.vert_dict:
            self.add_vertex(frm)
        if to not in self.vert_dict:
            self.add_vertex(to)

        self.vert_dict[frm].add_neighbor(self.vert_dict[to], cost)
        self.vert_dict[to].add_neighbor(self.vert_dict[frm], cost)

    def get_vertices(self):
        return self.vert_dict.keys()

if __name__ == '__main__':

    g = WeightedGraph(['a','b','c','d','e','f'])

    g.add_edge('a', 'b', 7)  
    g.add_edge('a', 'c', 9)
    g.add_edge('a', 'f', 14)
    g.add_edge('b', 'c', 10)
    g.add_edge('b', 'd', 15)
    g.add_edge('c', 'd', 11)
    g.add_edge('c', 'f', 2)
    g.add_edge('d', 'e', 6)
    g.add_edge('e', 'f', 9)

    for v in g:
        for w in v.get_connections():
            vid = v.get_id()
            wid = w.get_id()
            print('( %s , %s, %3d)'  % ( vid, wid, v.get_weight(w)))

    for v in g:
        print('g.vert_dict[%s]=%s' %(v.get_id(), g.vert_dict[v.get_id()]))