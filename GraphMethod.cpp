#include <iostream>
#include <vector>
#include "GraphMethod.h"
#include <stack>
#include <queue>
#include <map>
#include <set>
#include <list>
#include <utility>
#include <climits>
#include <algorithm>

using namespace std;

// BFS - Breadth First Search
bool BFS(Graph* graph, char option, int vertex, ofstream* fout)
{
    if(!graph || !fout) return false;
    
    int size = graph->getSize();
    if(vertex < 0 || vertex >= size) return false;
    
    vector<bool> visited(size, false);
    queue<int> q;
    vector<int> result;
    
    q.push(vertex);
    visited[vertex] = true;
    
    while(!q.empty()) {
        int curr = q.front();
        q.pop();
        result.push_back(curr);
        
        map<int, int> adjacent;
        if(option == 'O') {
            // Directed graph
            graph->getAdjacentEdgesDirect(curr, &adjacent);
        } else {
            // Undirected graph
            graph->getAdjacentEdges(curr, &adjacent);
        }
        
        // Visit adjacent vertices in ascending order (map is already sorted)
        for(auto& edge : adjacent) {
            if(!visited[edge.first]) {
                visited[edge.first] = true;
                q.push(edge.first);
            }
        }
    }
    
    // Output result
    if(option == 'O') {
        *fout << "Directed Graph BFS" << endl;
    } else {
        *fout << "Undirected Graph BFS" << endl;
    }
    *fout << "Start: " << vertex << endl;
    
    for(size_t i = 0; i < result.size(); i++) {
        if(i > 0) *fout << " -> ";
        *fout << result[i];
    }
    *fout << endl;
    
    return true;
}

// DFS - Depth First Search (using iteration to match specific traversal order)
bool DFS(Graph* graph, char option, int vertex, ofstream* fout)
{
    if(!graph || !fout) return false;
    
    int size = graph->getSize();
    if(vertex < 0 || vertex >= size) return false;
    
    vector<bool> visited(size, false);
    stack<int> s;
    vector<int> result;
    
    s.push(vertex);
    
    while(!s.empty()) {
        int curr = s.top();
        s.pop();
        
        if(visited[curr]) continue;
        
        visited[curr] = true;
        result.push_back(curr);
        
        map<int, int> adjacent;
        if(option == 'O') {
            // Directed graph
            graph->getAdjacentEdgesDirect(curr, &adjacent);
        } else {
            // Undirected graph
            graph->getAdjacentEdges(curr, &adjacent);
        }
        
        // Push adjacent vertices in reverse order (to visit in ascending order)
        vector<int> neighbors;
        for(auto& edge : adjacent) {
            if(!visited[edge.first]) {
                neighbors.push_back(edge.first);
            }
        }
        // Push in reverse order so that smallest comes out first
        for(int i = neighbors.size() - 1; i >= 0; i--) {
            s.push(neighbors[i]);
        }
    }
    
    // Output result
    if(option == 'O') {
        *fout << "Directed Graph DFS" << endl;
    } else {
        *fout << "Undirected Graph DFS" << endl;
    }
    *fout << "Start: " << vertex << endl;
    
    for(size_t i = 0; i < result.size(); i++) {
        if(i > 0) *fout << " -> ";
        *fout << result[i];
    }
    *fout << endl;
    
    return true;
}

// Union-Find data structure for Kruskal's algorithm
class UnionFind {
private:
    vector<int> parent;
    vector<int> rank_;
    
public:
    UnionFind(int n) {
        parent.resize(n);
        rank_.resize(n, 0);
        for(int i = 0; i < n; i++) {
            parent[i] = i;
        }
    }
    
    int find(int x) {
        if(parent[x] != x) {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    }
    
    bool unite(int x, int y) {
        int px = find(x);
        int py = find(y);
        if(px == py) return false;
        
        if(rank_[px] < rank_[py]) {
            parent[px] = py;
        } else if(rank_[px] > rank_[py]) {
            parent[py] = px;
        } else {
            parent[py] = px;
            rank_[px]++;
        }
        return true;
    }
};

// Kruskal's algorithm for Minimum Spanning Tree
bool Kruskal(Graph* graph, ofstream* fout)
{
    if(!graph || !fout) return false;
    
    int size = graph->getSize();
    
    // Collect all edges (treating as undirected)
    vector<tuple<int, int, int>> edges; // (weight, from, to)
    
    for(int i = 0; i < size; i++) {
        map<int, int> adjacent;
        graph->getAdjacentEdgesDirect(i, &adjacent);
        for(auto& edge : adjacent) {
            // Store each edge once (from smaller to larger vertex or just all)
            edges.push_back(make_tuple(edge.second, i, edge.first));
        }
    }
    
    // Sort edges by weight
    sort(edges.begin(), edges.end());
    
    UnionFind uf(size);
    vector<map<int, int>> mst(size); // MST adjacency list
    int totalCost = 0;
    int edgeCount = 0;
    
    for(auto& edge : edges) {
        int weight = get<0>(edge);
        int from = get<1>(edge);
        int to = get<2>(edge);
        
        if(uf.unite(from, to)) {
            // Add edge to MST (bidirectional)
            mst[from][to] = weight;
            mst[to][from] = weight;
            totalCost += weight;
            edgeCount++;
            
            if(edgeCount == size - 1) break;
        }
    }
    
    // Check if MST is complete (all vertices connected)
    if(edgeCount != size - 1) {
        return false;
    }
    
    // Output MST
    for(int i = 0; i < size; i++) {
        *fout << "[" << i << "]";
        bool first = true;
        for(auto& edge : mst[i]) {
            if(first) {
                *fout << " " << edge.first << "(" << edge.second << ")";
                first = false;
            } else {
                *fout << " " << edge.first << "(" << edge.second << ")";
            }
        }
        *fout << endl;
    }
    *fout << "Cost: " << totalCost << endl;
    
    return true;
}

// Dijkstra's algorithm
bool Dijkstra(Graph* graph, char option, int vertex, ofstream* fout)
{
    if(!graph || !fout) return false;
    
    int size = graph->getSize();
    if(vertex < 0 || vertex >= size) return false;
    
    // Check for negative weights
    for(int i = 0; i < size; i++) {
        map<int, int> adjacent;
        if(option == 'O') {
            graph->getAdjacentEdgesDirect(i, &adjacent);
        } else {
            graph->getAdjacentEdges(i, &adjacent);
        }
        for(auto& edge : adjacent) {
            if(edge.second < 0) return false;
        }
    }
    
    vector<int> dist(size, INT_MAX);
    vector<int> prev(size, -1);
    vector<bool> visited(size, false);
    
    dist[vertex] = 0;
    
    // Priority queue: (distance, vertex)
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push(make_pair(0, vertex));
    
    while(!pq.empty()) {
        int curr = pq.top().second;
        pq.pop();
        
        if(visited[curr]) continue;
        visited[curr] = true;
        
        map<int, int> adjacent;
        if(option == 'O') {
            graph->getAdjacentEdgesDirect(curr, &adjacent);
        } else {
            graph->getAdjacentEdges(curr, &adjacent);
        }
        
        for(auto& edge : adjacent) {
            int next = edge.first;
            int weight = edge.second;
            
            if(!visited[next] && dist[curr] != INT_MAX && dist[curr] + weight < dist[next]) {
                dist[next] = dist[curr] + weight;
                prev[next] = curr;
                pq.push(make_pair(dist[next], next));
            }
        }
    }
    
    // Output result
    if(option == 'O') {
        *fout << "Directed Graph Dijkstra" << endl;
    } else {
        *fout << "Undirected Graph Dijkstra" << endl;
    }
    *fout << "Start: " << vertex << endl;
    
    for(int i = 0; i < size; i++) {
        *fout << "[" << i << "] ";
        if(dist[i] == INT_MAX) {
            *fout << "x" << endl;
        } else {
            // Reconstruct path
            vector<int> path;
            int curr = i;
            while(curr != -1) {
                path.push_back(curr);
                curr = prev[curr];
            }
            reverse(path.begin(), path.end());
            
            for(size_t j = 0; j < path.size(); j++) {
                if(j > 0) *fout << " -> ";
                *fout << path[j];
            }
            *fout << " (" << dist[i] << ")" << endl;
        }
    }
    
    return true;
}

// Bellman-Ford algorithm
bool Bellmanford(Graph* graph, char option, int s_vertex, int e_vertex, ofstream* fout) 
{
    if(!graph || !fout) return false;
    
    int size = graph->getSize();
    if(s_vertex < 0 || s_vertex >= size || e_vertex < 0 || e_vertex >= size) return false;
    
    vector<int> dist(size, INT_MAX);
    vector<int> prev(size, -1);
    
    dist[s_vertex] = 0;
    
    // Collect all edges
    vector<tuple<int, int, int>> edges; // (from, to, weight)
    for(int i = 0; i < size; i++) {
        map<int, int> adjacent;
        if(option == 'O') {
            graph->getAdjacentEdgesDirect(i, &adjacent);
        } else {
            graph->getAdjacentEdges(i, &adjacent);
        }
        for(auto& edge : adjacent) {
            edges.push_back(make_tuple(i, edge.first, edge.second));
        }
    }
    
    // Relax edges |V| - 1 times
    for(int i = 0; i < size - 1; i++) {
        for(auto& edge : edges) {
            int from = get<0>(edge);
            int to = get<1>(edge);
            int weight = get<2>(edge);
            
            if(dist[from] != INT_MAX && dist[from] + weight < dist[to]) {
                dist[to] = dist[from] + weight;
                prev[to] = from;
            }
        }
    }
    
    // Check for negative cycle
    for(auto& edge : edges) {
        int from = get<0>(edge);
        int to = get<1>(edge);
        int weight = get<2>(edge);
        
        if(dist[from] != INT_MAX && dist[from] + weight < dist[to]) {
            return false; // Negative cycle detected
        }
    }
    
    // Output result
    if(option == 'O') {
        *fout << "Directed Graph Bellman-Ford" << endl;
    } else {
        *fout << "Undirected Graph Bellman-Ford" << endl;
    }
    
    if(dist[e_vertex] == INT_MAX) {
        *fout << "x" << endl;
        *fout << "Cost: x" << endl;
    } else {
        // Reconstruct path
        vector<int> path;
        int curr = e_vertex;
        while(curr != -1) {
            path.push_back(curr);
            curr = prev[curr];
        }
        reverse(path.begin(), path.end());
        
        for(size_t j = 0; j < path.size(); j++) {
            if(j > 0) *fout << " -> ";
            *fout << path[j];
        }
        *fout << endl;
        *fout << "Cost: " << dist[e_vertex] << endl;
    }
    
    return true;
}

// Floyd-Warshall algorithm
bool FLOYD(Graph* graph, char option, ofstream* fout)
{
    if(!graph || !fout) return false;
    
    int size = graph->getSize();
    
    // Initialize distance matrix
    vector<vector<int>> dist(size, vector<int>(size, INT_MAX));
    
    // Set diagonal to 0
    for(int i = 0; i < size; i++) {
        dist[i][i] = 0;
    }
    
    // Fill in direct edges
    for(int i = 0; i < size; i++) {
        map<int, int> adjacent;
        if(option == 'O') {
            graph->getAdjacentEdgesDirect(i, &adjacent);
        } else {
            graph->getAdjacentEdges(i, &adjacent);
        }
        for(auto& edge : adjacent) {
            dist[i][edge.first] = edge.second;
        }
    }
    
    // Floyd-Warshall
    for(int k = 0; k < size; k++) {
        for(int i = 0; i < size; i++) {
            for(int j = 0; j < size; j++) {
                if(dist[i][k] != INT_MAX && dist[k][j] != INT_MAX) {
                    if(dist[i][k] + dist[k][j] < dist[i][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                    }
                }
            }
        }
    }
    
    // Check for negative cycle (negative value on diagonal)
    for(int i = 0; i < size; i++) {
        if(dist[i][i] < 0) return false;
    }
    
    // Output result
    if(option == 'O') {
        *fout << "Directed Graph Floyd" << endl;
    } else {
        *fout << "Undirected Graph Floyd" << endl;
    }
    
    // Print header row
    *fout << "   ";
    for(int i = 0; i < size; i++) {
        *fout << " [" << i << "]";
    }
    *fout << endl;
    
    // Print matrix
    for(int i = 0; i < size; i++) {
        *fout << "[" << i << "]";
        for(int j = 0; j < size; j++) {
            if(dist[i][j] == INT_MAX) {
                *fout << " x  ";
            } else {
                *fout << " " << dist[i][j] << "  ";
            }
        }
        *fout << endl;
    }
    
    return true;
}

// Closeness Centrality
bool Centrality(Graph* graph, ofstream* fout) {
    if(!graph || !fout) return false;
    
    int size = graph->getSize();
    
    // Use Floyd-Warshall to get all shortest paths
    vector<vector<int>> dist(size, vector<int>(size, INT_MAX));
    
    // Set diagonal to 0
    for(int i = 0; i < size; i++) {
        dist[i][i] = 0;
    }
    
    // Fill in direct edges (as undirected)
    for(int i = 0; i < size; i++) {
        map<int, int> adjacent;
        graph->getAdjacentEdges(i, &adjacent);
        for(auto& edge : adjacent) {
            dist[i][edge.first] = edge.second;
        }
    }
    
    // Floyd-Warshall
    for(int k = 0; k < size; k++) {
        for(int i = 0; i < size; i++) {
            for(int j = 0; j < size; j++) {
                if(dist[i][k] != INT_MAX && dist[k][j] != INT_MAX) {
                    if(dist[i][k] + dist[k][j] < dist[i][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                    }
                }
            }
        }
    }
    
    // Check for negative cycle
    for(int i = 0; i < size; i++) {
        if(dist[i][i] < 0) return false;
    }
    
    // Calculate closeness centrality for each vertex
    // Centrality(v) = (n-1) / sum of shortest distances from v
    vector<int> sumDist(size, 0);
    vector<bool> reachable(size, true);
    
    for(int i = 0; i < size; i++) {
        int sum = 0;
        bool canReachAll = true;
        for(int j = 0; j < size; j++) {
            if(i != j) {
                if(dist[i][j] == INT_MAX) {
                    canReachAll = false;
                    break;
                }
                sum += dist[i][j];
            }
        }
        if(canReachAll) {
            sumDist[i] = sum;
        } else {
            reachable[i] = false;
        }
    }
    
    // Find most central vertex(s)
    // Centrality = (n-1) / sumDist, so max centrality means min sumDist
    int minSum = INT_MAX;
    for(int i = 0; i < size; i++) {
        if(reachable[i] && sumDist[i] < minSum) {
            minSum = sumDist[i];
        }
    }
    
    // Output result
    for(int i = 0; i < size; i++) {
        *fout << "[" << i << "] ";
        if(!reachable[i]) {
            *fout << "x" << endl;
        } else {
            *fout << (size - 1) << "/" << sumDist[i];
            if(sumDist[i] == minSum) {
                *fout << " <- Most Central";
            }
            *fout << endl;
        }
    }
    
    return true;
}