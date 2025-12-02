#include "ListGraph.h"
#include <iostream>
#include <utility>

// Constructor: Initialize adjacency list for each vertex
ListGraph::ListGraph(bool type, int size) : Graph(type, size)
{
	m_List = new map<int, int>[size]; // Allocate adjacency list for each vertex
	kw_graph = new vector<int>[size]; // Allocate vector for each vertex (used for undirected edges tracking)
}

// Destructor: Free allocated memory
ListGraph::~ListGraph()	
{
	delete[] m_List;
	delete[] kw_graph;
}

// Get adjacent edges (undirected - both directions)
void ListGraph::getAdjacentEdges(int vertex, map<int, int>* m)
{
	// Copy edges from adjacency list
	for (auto& edge : m_List[vertex]) {
		(*m)[edge.first] = edge.second;
	}
	// For undirected graph, also check reverse edges
	for (int i = 0; i < m_Size; i++) {
		if (i != vertex && m_List[i].find(vertex) != m_List[i].end()) {
			(*m)[i] = m_List[i][vertex];
		}
	}
}

// Get adjacent edges (directed - only outgoing edges)
void ListGraph::getAdjacentEdgesDirect(int vertex, map<int, int>* m)
{
	// Copy only outgoing edges from adjacency list
	for (auto& edge : m_List[vertex]) {
		(*m)[edge.first] = edge.second;
	}
}

// Insert an edge into the graph
void ListGraph::insertEdge(int from, int to, int weight)
{
	m_List[from][to] = weight; // Add edge from 'from' to 'to' with given weight
}

// Print the adjacency list representation
bool ListGraph::printGraph(ofstream *fout)
{
	if (!fout || !fout->is_open()) return false;
	
	*fout << "========PRINT========" << endl;
	for (int i = 0; i < m_Size; i++) {
		*fout << "[" << i << "]";
		bool first = true;
		for (auto& edge : m_List[i]) {
			if (first) {
				*fout << " -> ";
				first = false;
			} else {
				*fout << " -> ";
			}
			*fout << "(" << edge.first << "," << edge.second << ")";
		}
		*fout << endl;
	}
	*fout << "=====================" << endl << endl;
	return true;
}