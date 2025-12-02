#include "MatrixGraph.h"
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>

// Constructor: Allocate and initialize adjacency matrix
MatrixGraph::MatrixGraph(bool type, int size) : Graph(type, size)
{
	m_Mat = new int*[size]; // Allocate rows
	for (int i = 0; i < size; i++) {
		m_Mat[i] = new int[size]; // Allocate columns
		for (int j = 0; j < size; j++) {
			m_Mat[i][j] = 0; // Initialize all edges to 0 (no edge)
		}
	}
}

// Destructor: Free allocated memory
MatrixGraph::~MatrixGraph()
{
	for (int i = 0; i < m_Size; i++) {
		delete[] m_Mat[i];
	}
	delete[] m_Mat;
}

// Get adjacent edges (undirected - both directions)
void MatrixGraph::getAdjacentEdges(int vertex, map<int, int>* m)
{	
	for (int i = 0; i < m_Size; i++) {
		// Check both directions for undirected graph
		if (m_Mat[vertex][i] != 0) {
			(*m)[i] = m_Mat[vertex][i];
		}
		if (m_Mat[i][vertex] != 0) {
			(*m)[i] = m_Mat[i][vertex];
		}
	}
}

// Get adjacent edges (directed - only outgoing edges)
void MatrixGraph::getAdjacentEdgesDirect(int vertex, map<int, int>* m)
{
	for (int i = 0; i < m_Size; i++) {
		if (m_Mat[vertex][i] != 0) {
			(*m)[i] = m_Mat[vertex][i];
		}
	}
}

// Insert an edge into the matrix
void MatrixGraph::insertEdge(int from, int to, int weight)	
{
	m_Mat[from][to] = weight;
}

// Print the adjacency matrix representation
bool MatrixGraph::printGraph(ofstream *fout)	
{
	if (!fout || !fout->is_open()) return false;
	
	*fout << "========PRINT========" << endl;
	
	// Print column headers
	*fout << "    ";
	for (int i = 0; i < m_Size; i++) {
		*fout << "[" << i << "] ";
	}
	*fout << endl;
	
	// Print matrix rows
	for (int i = 0; i < m_Size; i++) {
		*fout << "[" << i << "] ";
		for (int j = 0; j < m_Size; j++) {
			*fout << m_Mat[i][j];
			// Calculate padding for alignment
			int num = m_Mat[i][j];
			int digits = 1;
			if (num < 0) digits++;
			while (num / 10 != 0) { num /= 10; digits++; }
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