#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <functional>
#include <stack>
#include <set>

const int INF = 1e9; // Use a large value to represent infinity

template<typename T>
class Graph {
public:
    enum GraphType { UNDIRECTED, DIRECTED };

private:
    std::unordered_map<T, std::vector<std::pair<T, int>>> adjList;
    GraphType graphType;

public:
    Graph(GraphType type = UNDIRECTED);

    void addEdge(T u, T v, int weight = 1);
    void removeEdge(T u, T v);
    void removeVertex(T v);
    std::vector<T> bfs(T start);
    std::vector<T> dfs(T start);
    std::unordered_map<T, int> dijkstra(T start);
    std::unordered_map<T, int> shortestPath(T start);
    void floydWarshall(std::vector<std::vector<int>>& dist);
    bool hasCycle();
    bool isDAG();
    std::unordered_map<T, int> shortestPathDAG(T start);

private:
    bool cycleUtil(T node, std::unordered_map<T, bool>& visited, T parent);
    void topologicalSortUtil(T v, std::unordered_map<T, bool>& visited, std::stack<T>& Stack);
};

#include "Graph.inl"

#endif // GRAPH_H
