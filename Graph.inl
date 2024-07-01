// Graph.inl

template<typename T>
Graph<T>::Graph(GraphType type) : graphType(type) {}

template<typename T>
void Graph<T>::addEdge(T u, T v, int weight) {
    adjList[u].emplace_back(v, weight);
    if (graphType == Graph<T>::UNDIRECTED) {
        adjList[v].emplace_back(u, weight);
    }
}

template<typename T>
void Graph<T>::removeEdge(T u, T v) {
    adjList[u].erase(std::remove_if(adjList[u].begin(), adjList[u].end(),
        [v](const std::pair<T, int>& p) { return p.first == v; }), adjList[u].end());
    if (graphType == Graph<T>::UNDIRECTED) {
        adjList[v].erase(std::remove_if(adjList[v].begin(), adjList[v].end(),
            [u](const std::pair<T, int>& p) { return p.first == u; }), adjList[v].end());
    }
}

template<typename T>
void Graph<T>::removeVertex(T v) {
    adjList.erase(v);
    for (auto& pair : adjList) {
        removeEdge(pair.first, v);
    }
}

template<typename T>
std::vector<T> Graph<T>::bfs(T start) {
    std::vector<T> result;
    std::queue<T> q;
    std::unordered_map<T, bool> visited;

    q.push(start);
    visited[start] = true;

    while (!q.empty()) {
        T node = q.front();
        q.pop();
        result.push_back(node);

        for (auto& neighbor : adjList[node]) {
            if (!visited[neighbor.first]) {
                q.push(neighbor.first);
                visited[neighbor.first] = true;
            }
        }
    }

    return result;
}

template<typename T>
std::vector<T> Graph<T>::dfs(T start) {
    std::vector<T> result;
    std::stack<T> s;
    std::unordered_map<T, bool> visited;

    s.push(start);
    visited[start] = true;

    while (!s.empty()) {
        T node = s.top();
        s.pop();
        result.push_back(node);

        for (auto& neighbor : adjList[node]) {
            if (!visited[neighbor.first]) {
                s.push(neighbor.first);
                visited[neighbor.first] = true;
            }
        }
    }

    return result;
}

template<typename T>
std::unordered_map<T, int> Graph<T>::dijkstra(T start) {
    std::unordered_map<T, int> distances;
    for (auto& pair : adjList) {
        distances[pair.first] = std::numeric_limits<int>::max();
    }
    distances[start] = 0;

    using Pair = std::pair<int, T>;
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> pq;
    pq.push({0, start});

    while (!pq.empty()) {
        int dist = pq.top().first;
        T node = pq.top().second;
        pq.pop();

        for (auto& neighbor : adjList[node]) {
            int newDist = dist + neighbor.second;
            if (newDist < distances[neighbor.first]) {
                distances[neighbor.first] = newDist;
                pq.push({newDist, neighbor.first});
            }
        }
    }

    return distances;
}

template<typename T>
std::unordered_map<T, int> Graph<T>::shortestPath(T start) {
    if (isDAG()) {
        return shortestPathDAG(start);
    }

    int edges = 0;
    for (const auto& pair : adjList) {
        edges += pair.second.size();
    }

    int vertices = adjList.size();
    if (edges > vertices * (vertices - 1) / 2) {
        std::vector<std::vector<int>> dist(vertices, std::vector<int>(vertices, INF));
        floydWarshall(dist);
        std::unordered_map<T, int> distances;
        for (int i = 0; i < vertices; ++i) {
            distances[i] = dist[start][i];
        }
        return distances;
    } else {
        return dijkstra(start);
    }
}

template<typename T>
void Graph<T>::floydWarshall(std::vector<std::vector<int>>& dist) {
    int V = adjList.size();
    for (int i = 0; i < V; ++i) {
        for (int j = 0; j < V; ++j) {
            if (i == j) {
                dist[i][j] = 0;
            } else {
                dist[i][j] = INF;
            }
        }
    }

    for (auto& pair : adjList) {
        for (auto& neighbor : pair.second) {
            dist[pair.first][neighbor.first] = neighbor.second;
        }
    }

    for (int k = 0; k < V; ++k) {
        for (int i = 0; i < V; ++i) {
            for (int j = 0; j < V; ++j) {
                if (dist[i][k] != INF && dist[k][j] != INF) {
                    dist[i][j] = std::min(dist[i][j], dist[i][k] + dist[k][j]);
                }
            }
        }
    }
}

template<typename T>
bool Graph<T>::hasCycle() {
    std::unordered_map<T, bool> visited;
    for (auto& pair : adjList) {
        if (!visited[pair.first]) {
            if (cycleUtil(pair.first, visited, -1)) {
                return true;
            }
        }
    }
    return false;
}

template<typename T>
bool Graph<T>::isDAG() {
    return !hasCycle();
}

template<typename T>
bool Graph<T>::cycleUtil(T node, std::unordered_map<T, bool>& visited, T parent) {
    visited[node] = true;
    for (auto& neighbor : adjList[node]) {
        if (!visited[neighbor.first]) {
            if (cycleUtil(neighbor.first, visited, node)) {
                return true;
            }
        } else if (neighbor.first != parent) {
            return true;
        }
    }
    return false;
}

template<typename T>
void Graph<T>::topologicalSortUtil(T v, std::unordered_map<T, bool>& visited, std::stack<T>& Stack) {
    visited[v] = true;

    for (auto& neighbor : adjList[v]) {
        if (!visited[neighbor.first]) {
            topologicalSortUtil(neighbor.first, visited, Stack);
        }
    }

    Stack.push(v);
}

template<typename T>
std::unordered_map<T, int> Graph<T>::shortestPathDAG(T start) {
    std::stack<T> Stack;
    std::unordered_map<T, bool> visited;
    for (auto& pair : adjList) {
        if (!visited[pair.first]) {
            topologicalSortUtil(pair.first, visited, Stack);
        }
    }

    std::unordered_map<T, int> distances;
    for (auto& pair : adjList) {
        distances[pair.first] = INF;
    }
    distances[start] = 0;

    while (!Stack.empty()) {
        T node = Stack.top();
        Stack.pop();

        if (distances[node] != INF) {
            for (auto& neighbor : adjList[node]) {
                if (distances[neighbor.first] > distances[node] + neighbor.second) {
                    distances[neighbor.first] = distances[node] + neighbor.second;
                }
            }
        }
    }
    
    return distances;
}
