# Source from https://www.geeksforgeeks.org/vertex-cover-problem-set-1-introduction-approximate-algorithm-2/

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
