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
#include <cfloat>

using namespace std;

// BFS implementation
bool BFS(Graph* graph, char option, int vertex, ofstream* fout)
{
	if (!graph || !fout) return false;
	
	int size = graph->getSize();
	bool directed = (option == 'O');
	
	// Check if start vertex is valid
	if (vertex < 0 || vertex >= size) return false;
	
	vector<bool> visited(size, false);
	queue<int> q;
	vector<int> result;
	
	// Start BFS from given vertex
	visited[vertex] = true;
	q.push(vertex);
	
	while (!q.empty()) {
		int curr = q.front();
		q.pop();
		result.push_back(curr);
		
		// Get adjacent vertices
		map<int, int> adj;
		if (directed) {
			graph->getAdjacentEdgesDirect(curr, &adj);
		} else {
			graph->getAdjacentEdges(curr, &adj);
		}
		
		// Visit adjacent vertices in ascending order (map is already sorted by key)
		for (auto& edge : adj) {
			int next = edge.first;
			if (!visited[next]) {
				visited[next] = true;
				q.push(next);
			}
		}
	}
	
	// Print result
	*fout << "========BFS========" << endl;
	if (directed) {
		*fout << "Directed Graph BFS" << endl;
	} else {
		*fout << "Undirected Graph BFS" << endl;
	}
	*fout << "Start: " << vertex << endl;
	
	for (size_t i = 0; i < result.size(); i++) {
		*fout << result[i];
		if (i < result.size() - 1) {
			*fout << " -> ";
		}
	}
	*fout << endl;
	*fout << "===================" << endl << endl;
	
	return true;
}

// DFS implementation
bool DFS(Graph* graph, char option, int vertex, ofstream* fout)
{
	if (!graph || !fout) return false;
	
	int size = graph->getSize();
	bool directed = (option == 'O');
	
	// Check if start vertex is valid
	if (vertex < 0 || vertex >= size) return false;
	
	vector<bool> visited(size, false);
	stack<int> s;
	vector<int> result;
	
	// Start DFS from given vertex
	s.push(vertex);
	
	while (!s.empty()) {
		int curr = s.top();
		s.pop();
		
		if (visited[curr]) continue;
		
		visited[curr] = true;
		result.push_back(curr);
		
		// Get adjacent vertices
		map<int, int> adj;
		if (directed) {
			graph->getAdjacentEdgesDirect(curr, &adj);
		} else {
			graph->getAdjacentEdges(curr, &adj);
		}
		
		// Push adjacent vertices in descending order (so we visit in ascending order)
		vector<int> neighbors;
		for (auto& edge : adj) {
			if (!visited[edge.first]) {
				neighbors.push_back(edge.first);
			}
		}
		// Sort in descending order for stack
		sort(neighbors.rbegin(), neighbors.rend());
		for (int next : neighbors) {
			s.push(next);
		}
	}
	
	// Print result
	*fout << "========DFS========" << endl;
	if (directed) {
		*fout << "Directed Graph DFS" << endl;
	} else {
		*fout << "Undirected Graph DFS" << endl;
	}
	*fout << "Start: " << vertex << endl;
	
	for (size_t i = 0; i < result.size(); i++) {
		*fout << result[i];
		if (i < result.size() - 1) {
			*fout << " -> ";
		}
	}
	*fout << endl;
	*fout << "===================" << endl << endl;
	
	return true;
}

// Union-Find data structure for Kruskal's algorithm
class UnionFind {
private:
	vector<int> parent;
	vector<int> rank_arr;
	
public:
	UnionFind(int n) {
		parent.resize(n);
		rank_arr.resize(n, 0);
		for (int i = 0; i < n; i++) {
			parent[i] = i;
		}
	}
	
	int find(int x) {
		if (parent[x] != x) {
			parent[x] = find(parent[x]); // Path compression
		}
		return parent[x];
	}
	
	bool unite(int x, int y) {
		int px = find(x);
		int py = find(y);
		if (px == py) return false;
		
		// Union by rank
		if (rank_arr[px] < rank_arr[py]) {
			parent[px] = py;
		} else if (rank_arr[px] > rank_arr[py]) {
			parent[py] = px;
		} else {
			parent[py] = px;
			rank_arr[px]++;
		}
		return true;
	}
};

// Kruskal's MST algorithm
bool Kruskal(Graph* graph, ofstream* fout)
{
	if (!graph || !fout) return false;
	
	int size = graph->getSize();
	
	// Collect all edges (undirected) using a set to avoid duplicates
	set<tuple<int, int, int>> edgeSet; // (weight, min_vertex, max_vertex)
	for (int i = 0; i < size; i++) {
		map<int, int> adj;
		graph->getAdjacentEdgesDirect(i, &adj);
		for (auto& edge : adj) {
			int to = edge.first;
			int weight = edge.second;
			// Normalize edge representation: (weight, min_vertex, max_vertex)
			int minV = min(i, to);
			int maxV = max(i, to);
			edgeSet.insert(make_tuple(weight, minV, maxV));
		}
	}
	
	// Convert set to vector for processing
	vector<tuple<int, int, int>> edges(edgeSet.begin(), edgeSet.end());
	
	// Edges are already sorted by the set (first by weight, then by vertices)
	
	UnionFind uf(size);
	vector<map<int, int>> mst(size); // MST adjacency list
	int totalCost = 0;
	int edgeCount = 0;
	
	// Process edges in order of weight
	for (auto& e : edges) {
		int weight = get<0>(e);
		int from = get<1>(e);
		int to = get<2>(e);
		
		if (uf.unite(from, to)) {
			// Add edge to MST (undirected)
			mst[from][to] = weight;
			mst[to][from] = weight;
			totalCost += weight;
			edgeCount++;
			
			if (edgeCount == size - 1) break;
		}
	}
	
	// Check if MST is complete (all vertices connected)
	if (edgeCount != size - 1) {
		return false; // Not all vertices are reachable
	}
	
	// Print MST
	*fout << "========KRUSKAL========" << endl;
	for (int i = 0; i < size; i++) {
		*fout << "[" << i << "]";
		bool first = true;
		for (auto& edge : mst[i]) {
			if (first) {
				*fout << " ";
				first = false;
			} else {
				*fout << " ";
			}
			*fout << edge.first << "(" << edge.second << ")";
		}
		*fout << endl;
	}
	*fout << "Cost: " << totalCost << endl;
	*fout << "=======================" << endl << endl;
	
	return true;
}

// Dijkstra's shortest path algorithm
bool Dijkstra(Graph* graph, char option, int vertex, ofstream* fout)
{
	if (!graph || !fout) return false;
	
	int size = graph->getSize();
	bool directed = (option == 'O');
	
	// Check for negative weights
	for (int i = 0; i < size; i++) {
		map<int, int> adj;
		if (directed) {
			graph->getAdjacentEdgesDirect(i, &adj);
		} else {
			graph->getAdjacentEdges(i, &adj);
		}
		for (auto& edge : adj) {
			if (edge.second < 0) {
				return false; // Negative weight found
			}
		}
	}
	
	vector<int> dist(size, INT_MAX);
	vector<int> parent(size, -1);
	priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
	
	dist[vertex] = 0;
	pq.push({0, vertex});
	
	while (!pq.empty()) {
		int d = pq.top().first;
		int u = pq.top().second;
		pq.pop();
		
		if (d > dist[u]) continue;
		
		map<int, int> adj;
		if (directed) {
			graph->getAdjacentEdgesDirect(u, &adj);
		} else {
			graph->getAdjacentEdges(u, &adj);
		}
		
		for (auto& edge : adj) {
			int v = edge.first;
			int w = edge.second;
			
			if (dist[u] != INT_MAX && dist[u] + w < dist[v]) {
				dist[v] = dist[u] + w;
				parent[v] = u;
				pq.push({dist[v], v});
			}
		}
	}
	
	// Print result
	*fout << "========DIJKSTRA========" << endl;
	if (directed) {
		*fout << "Directed Graph Dijkstra" << endl;
	} else {
		*fout << "Undirected Graph Dijkstra" << endl;
	}
	*fout << "Start: " << vertex << endl;
	
	for (int i = 0; i < size; i++) {
		*fout << "[" << i << "] ";
		if (dist[i] == INT_MAX) {
			*fout << "x" << endl;
		} else {
			// Reconstruct path
			vector<int> path;
			int curr = i;
			while (curr != -1) {
				path.push_back(curr);
				curr = parent[curr];
			}
			reverse(path.begin(), path.end());
			
			for (size_t j = 0; j < path.size(); j++) {
				*fout << path[j];
				if (j < path.size() - 1) {
					*fout << " -> ";
				}
			}
			*fout << " (" << dist[i] << ")" << endl;
		}
	}
	*fout << "========================" << endl << endl;
	
	return true;
}

// Bellman-Ford shortest path algorithm
bool Bellmanford(Graph* graph, char option, int s_vertex, int e_vertex, ofstream* fout) 
{
	if (!graph || !fout) return false;
	
	int size = graph->getSize();
	bool directed = (option == 'O');
	
	vector<int> dist(size, INT_MAX);
	vector<int> parent(size, -1);
	
	dist[s_vertex] = 0;
	
	// Collect all edges
	vector<tuple<int, int, int>> edges; // (from, to, weight)
	for (int i = 0; i < size; i++) {
		map<int, int> adj;
		if (directed) {
			graph->getAdjacentEdgesDirect(i, &adj);
		} else {
			graph->getAdjacentEdges(i, &adj);
		}
		for (auto& edge : adj) {
			edges.push_back(make_tuple(i, edge.first, edge.second));
		}
	}
	
	// Relax edges |V| - 1 times
	for (int i = 0; i < size - 1; i++) {
		for (auto& e : edges) {
			int u = get<0>(e);
			int v = get<1>(e);
			int w = get<2>(e);
			
			if (dist[u] != INT_MAX && dist[u] + w < dist[v]) {
				dist[v] = dist[u] + w;
				parent[v] = u;
			}
		}
	}
	
	// Check for negative cycle
	for (auto& e : edges) {
		int u = get<0>(e);
		int v = get<1>(e);
		int w = get<2>(e);
		
		if (dist[u] != INT_MAX && dist[u] + w < dist[v]) {
			return false; // Negative cycle detected
		}
	}
	
	// Print result
	*fout << "========BELLMANFORD========" << endl;
	if (directed) {
		*fout << "Directed Graph Bellman-Ford" << endl;
	} else {
		*fout << "Undirected Graph Bellman-Ford" << endl;
	}
	
	if (dist[e_vertex] == INT_MAX) {
		*fout << "x" << endl;
	} else {
		// Reconstruct path
		vector<int> path;
		int curr = e_vertex;
		while (curr != -1) {
			path.push_back(curr);
			curr = parent[curr];
		}
		reverse(path.begin(), path.end());
		
		for (size_t j = 0; j < path.size(); j++) {
			*fout << path[j];
			if (j < path.size() - 1) {
				*fout << " -> ";
			}
		}
		*fout << endl;
		*fout << "Cost: " << dist[e_vertex] << endl;
	}
	*fout << "============================" << endl << endl;
	
	return true;
}

// Floyd-Warshall all-pairs shortest path algorithm
bool FLOYD(Graph* graph, char option, ofstream* fout)
{
	if (!graph || !fout) return false;
	
	int size = graph->getSize();
	bool directed = (option == 'O');
	
	// Initialize distance matrix
	vector<vector<int>> dist(size, vector<int>(size, INT_MAX));
	
	// Set diagonal to 0
	for (int i = 0; i < size; i++) {
		dist[i][i] = 0;
	}
	
	// Fill in edge weights
	for (int i = 0; i < size; i++) {
		map<int, int> adj;
		if (directed) {
			graph->getAdjacentEdgesDirect(i, &adj);
		} else {
			graph->getAdjacentEdges(i, &adj);
		}
		for (auto& edge : adj) {
			dist[i][edge.first] = edge.second;
		}
	}
	
	// Floyd-Warshall algorithm
	for (int k = 0; k < size; k++) {
		for (int i = 0; i < size; i++) {
			for (int j = 0; j < size; j++) {
				if (dist[i][k] != INT_MAX && dist[k][j] != INT_MAX) {
					if (dist[i][k] + dist[k][j] < dist[i][j]) {
						dist[i][j] = dist[i][k] + dist[k][j];
					}
				}
			}
		}
	}
	
	// Check for negative cycle
	for (int i = 0; i < size; i++) {
		if (dist[i][i] < 0) {
			return false; // Negative cycle detected
		}
	}
	
	// Print result
	*fout << "========FLOYD========" << endl;
	if (directed) {
		*fout << "Directed Graph Floyd" << endl;
	} else {
		*fout << "Undirected Graph Floyd" << endl;
	}
	
	// Print column headers
	*fout << "    ";
	for (int i = 0; i < size; i++) {
		*fout << "[" << i << "] ";
	}
	*fout << endl;
	
	// Print distance matrix
	for (int i = 0; i < size; i++) {
		*fout << "[" << i << "] ";
		for (int j = 0; j < size; j++) {
			if (dist[i][j] == INT_MAX) {
				*fout << "x";
			} else {
				*fout << dist[i][j];
			}
			// Calculate padding for alignment
			int num = dist[i][j];
			int digits = 1;
			if (num == INT_MAX) digits = 1;
			else {
				if (num < 0) digits++;
				while (num / 10 != 0) { num /= 10; digits++; }
			}
			// Add spaces for alignment based on column header width
			int header_width = 1;
			int col = j;
			while (col / 10 != 0) { col /= 10; header_width++; }
			*fout << string(header_width + 3 - digits, ' ');
		}
		*fout << endl;
	}
	*fout << "=====================" << endl << endl;
	
	return true;
}

// Closeness Centrality calculation using Floyd-Warshall distances
bool Centrality(Graph* graph, ofstream* fout) {
	if (!graph || !fout) return false;
	
	int size = graph->getSize();
	
	// Initialize distance matrix (undirected)
	vector<vector<int>> dist(size, vector<int>(size, INT_MAX));
	
	// Set diagonal to 0
	for (int i = 0; i < size; i++) {
		dist[i][i] = 0;
	}
	
	// Fill in edge weights (undirected - use getAdjacentEdges)
	for (int i = 0; i < size; i++) {
		map<int, int> adj;
		graph->getAdjacentEdges(i, &adj);
		for (auto& edge : adj) {
			dist[i][edge.first] = edge.second;
		}
	}
	
	// Floyd-Warshall algorithm
	for (int k = 0; k < size; k++) {
		for (int i = 0; i < size; i++) {
			for (int j = 0; j < size; j++) {
				if (dist[i][k] != INT_MAX && dist[k][j] != INT_MAX) {
					if (dist[i][k] + dist[k][j] < dist[i][j]) {
						dist[i][j] = dist[i][k] + dist[k][j];
					}
				}
			}
		}
	}
	
	// Calculate closeness centrality for each vertex
	*fout << "========CENTRALITY========" << endl;
	
	for (int i = 0; i < size; i++) {
		*fout << "[" << i << "] ";
		
		// Check if all vertices are reachable
		bool allReachable = true;
		int sumDist = 0;
		for (int j = 0; j < size; j++) {
			if (i != j && dist[i][j] == INT_MAX) {
				allReachable = false;
				break;
			}
			sumDist += dist[i][j];
		}
		
		if (!allReachable || sumDist == 0) {
			*fout << "x" << endl;
		} else {
			// Closeness centrality = (n-1) / sum of distances
			double centrality = (double)(size - 1) / sumDist;
			*fout << fixed;
			fout->precision(2);
			*fout << centrality << endl;
		}
	}
	*fout << "==========================" << endl << endl;
	
	return true;
}