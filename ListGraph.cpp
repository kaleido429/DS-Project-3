#include "ListGraph.h"
#include <iostream>
#include <utility>

ListGraph::ListGraph(bool type, int size) : Graph(type, size)
{
	// Allocate array of maps for adjacency list
	m_List = new map<int, int>[size];
}

ListGraph::~ListGraph()	
{
	// Deallocate the adjacency list array
	if(m_List)
		delete[] m_List;
}

void ListGraph::getAdjacentEdges(int vertex, map<int, int>* m)	 //Definition of getAdjacentEdges(No Direction == Undirected)
{
	// Copy all edges from the adjacency list of the vertex (for undirected graph)
	// Also include edges where this vertex is the destination
	for(auto& edge : m_List[vertex]) {
		(*m)[edge.first] = edge.second;
	}
	// For undirected, also check all other vertices for edges pointing to this vertex
	for(int i = 0; i < m_Size; i++) {
		if(i != vertex && m_List[i].find(vertex) != m_List[i].end()) {
			(*m)[i] = m_List[i][vertex];
		}
	}
}

void ListGraph::getAdjacentEdgesDirect(int vertex, map<int, int>* m)	//Definition of getAdjacentEdges(Directed graph)
{
	// Copy all edges from the adjacency list of the vertex (for directed graph)
	for(auto& edge : m_List[vertex]) {
		(*m)[edge.first] = edge.second;
	}
}

void ListGraph::insertEdge(int from, int to, int weight) //Definition of insertEdge
{
	// Insert edge into the adjacency list
	m_List[from][to] = weight;
}

bool ListGraph::printGraph(ofstream *fout)	//Definition of print Graph
{
	if(!fout || !fout->is_open())
		return false;
	
	// Print adjacency list for each vertex in ascending order
	for(int i = 0; i < m_Size; i++) {
		*fout << "[" << i << "]";
		// Edges are already in ascending order because map is sorted
		for(auto& edge : m_List[i]) {
			*fout << " -> (" << edge.first << "," << edge.second << ")";
		}
		*fout << endl;
	}
	return true;
}