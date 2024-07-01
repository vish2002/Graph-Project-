#include <iostream>
#include "Graph.h"

int main() {
    // Create a directed graph
    Graph<int> g(Graph<int>::DIRECTED);

    g.addEdge(0, 1, 5);
    g.addEdge(0, 2, 1);
    g.addEdge(1, 2, 3);
    g.addEdge(2, 0, 2);
    g.addEdge(2, 3, 1);
    g.addEdge(3, 3, 1);

    // BFS
    std::vector<int> bfsResult = g.bfs(2);
    std::cout << "BFS starting from node 2: ";
    for (int node : bfsResult) {
        std::cout << node << " ";
    }
    std::cout << std::endl;

    // DFS
    std::vector<int> dfsResult = g.dfs(2);
    std::cout << "DFS starting from node 2: ";
    for (int node : dfsResult) {
        std::cout << node << " ";
    }
    std::cout << std::endl;

    // Dijkstra's
    std::unordered_map<int, int> dijkstraResult = g.dijkstra(0);
    std::cout << "Dijkstra's shortest paths from node 0: ";
    for (const auto& pair : dijkstraResult) {
        std::cout << "Node " << pair.first << " - Distance " << pair.second << ", ";
    }
    std::cout << std::endl;

    // General shortest path algorithm
    std::unordered_map<int, int> spResult = g.shortestPath(0);
    std::cout << "General shortest paths from node 0: ";
    for (const auto& pair : spResult) {
        std::cout << "Node " << pair.first << " - Distance " << pair.second << ", ";
    }
    std::cout << std::endl;

    // Floyd-Warshall
    int V = 4; // Number of vertices
    std::vector<std::vector<int>> dist(V, std::vector<int>(V, INF));
    g.floydWarshall(dist);
    std::cout << "Floyd-Warshall all pairs shortest paths: \n";
    for (int i = 0; i < V; ++i) {
        for (int j = 0; j < V; ++j) {
            if (dist[i][j] == INF) {
                std::cout << "INF ";
            } else {
                std::cout << dist[i][j] << " ";
            }
        }
        std::cout << std::endl;
    }

    // Cycle Detection
    std::cout << "Graph has cycle: " << (g.hasCycle() ? "Yes" : "No") << std::endl;

    return 0;
}
