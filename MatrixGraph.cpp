#include "MatrixGraph.h"
#include <iostream>
#include <vector>
#include <string>

MatrixGraph::MatrixGraph(bool type, int size) : Graph(type, size)
{
	// Allocate 2D array for adjacency matrix
	m_Mat = new int*[size];
	for(int i = 0; i < size; i++) {
		m_Mat[i] = new int[size];
		// Initialize all values to 0
		for(int j = 0; j < size; j++) {
			m_Mat[i][j] = 0;
		}
	}
}

MatrixGraph::~MatrixGraph()
{
	// Deallocate the adjacency matrix
	if(m_Mat) {
		for(int i = 0; i < m_Size; i++) {
			delete[] m_Mat[i];
		}
		delete[] m_Mat;
	}
}

void MatrixGraph::getAdjacentEdges(int vertex, map<int, int>* m)
{	
	// For undirected graph, get all connected vertices
	for(int i = 0; i < m_Size; i++) {
		if(m_Mat[vertex][i] != 0) {
			(*m)[i] = m_Mat[vertex][i];
		}
		// Also check edges coming into this vertex
		if(m_Mat[i][vertex] != 0) {
			(*m)[i] = m_Mat[i][vertex];
		}
	}
}

void MatrixGraph::getAdjacentEdgesDirect(int vertex, map<int, int>* m)
{
	// For directed graph, only get outgoing edges
	for(int i = 0; i < m_Size; i++) {
		if(m_Mat[vertex][i] != 0) {
			(*m)[i] = m_Mat[vertex][i];
		}
	}
}

void MatrixGraph::insertEdge(int from, int to, int weight)	
{
	// Insert edge into the adjacency matrix
	m_Mat[from][to] = weight;
}

bool MatrixGraph::printGraph(ofstream *fout)	
{
	if(!fout || !fout->is_open())
		return false;
	
	// Print header row
	*fout << "   ";
	for(int i = 0; i < m_Size; i++) {
		*fout << " [" << i << "]";
	}
	*fout << endl;
	
	// Print each row
	for(int i = 0; i < m_Size; i++) {
		*fout << "[" << i << "]";
		for(int j = 0; j < m_Size; j++) {
			*fout << " " << m_Mat[i][j] << "  ";
		}
		*fout << endl;
	}
	return true;
}