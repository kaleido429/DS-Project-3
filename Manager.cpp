#include "Manager.h"
#include "GraphMethod.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

Manager::Manager()	
{
	graph = nullptr;	
	fout.open("log.txt", ios::app);
	load = 0;	// Nothing is loaded
}

Manager::~Manager()
{
	if(load)	// If graph is loaded, delete graph
		delete graph;	
	if(fout.is_open())	// If fout is opened, close file
		fout.close();	// Close log.txt file
}

void Manager::run(const char* command_txt){
	ifstream fin;	// Command File Input File Stream
	fin.open(command_txt, ios_base::in); // File open with read mode
		
	if(!fin) { // If command file cannot be read, print error
		fout << "command file open error" << endl;
		return;	// Return
	}
	
	string line;
	while (getline(fin, line)) {
		if (line.empty()) continue; // Skip empty lines
		
		// Parse the command line
		istringstream iss(line);
		string cmd;
		iss >> cmd;
		
		if (cmd == "LOAD") {
			string filename;
			string extra;
			if (!(iss >> filename) || (iss >> extra)) {
				// No filename provided or too many arguments
				printErrorCode(100);
				continue;
			}
			if (!LOAD(filename.c_str())) {
				printErrorCode(100);
			}
		}
		else if (cmd == "PRINT") {
			string extra;
			if (iss >> extra) {
				// Too many arguments
				printErrorCode(200);
				continue;
			}
			if (!PRINT()) {
				printErrorCode(200);
			}
		}
		else if (cmd == "BFS") {
			char option;
			int vertex;
			string extra;
			if (!(iss >> option >> vertex) || (iss >> extra)) {
				printErrorCode(300);
				continue;
			}
			if (!mBFS(option, vertex)) {
				printErrorCode(300);
			}
		}
		else if (cmd == "DFS") {
			char option;
			int vertex;
			string extra;
			if (!(iss >> option >> vertex) || (iss >> extra)) {
				printErrorCode(400);
				continue;
			}
			if (!mDFS(option, vertex)) {
				printErrorCode(400);
			}
		}
		else if (cmd == "DIJKSTRA") {
			char option;
			int vertex;
			string extra;
			if (!(iss >> option >> vertex) || (iss >> extra)) {
				printErrorCode(600);
				continue;
			}
			if (!mDIJKSTRA(option, vertex)) {
				printErrorCode(600);
			}
		}
		else if (cmd == "KRUSKAL") {
			string extra;
			if (iss >> extra) {
				printErrorCode(500);
				continue;
			}
			if (!mKRUSKAL()) {
				printErrorCode(500);
			}
		}
		else if (cmd == "BELLMANFORD") {
			char option;
			int s_vertex, e_vertex;
			string extra;
			if (!(iss >> option >> s_vertex >> e_vertex) || (iss >> extra)) {
				printErrorCode(700);
				continue;
			}
			if (!mBELLMANFORD(option, s_vertex, e_vertex)) {
				printErrorCode(700);
			}
		}
		else if (cmd == "FLOYD") {
			char option;
			string extra;
			if (!(iss >> option) || (iss >> extra)) {
				printErrorCode(800);
				continue;
			}
			if (!mFLOYD(option)) {
				printErrorCode(800);
			}
		}
		else if (cmd == "CENTRALITY") {
			string extra;
			if (iss >> extra) {
				printErrorCode(900);
				continue;
			}
			if (!mCentrality()) {
				printErrorCode(900);
			}
		}
		else if (cmd == "EXIT") {
			break;
		}
	}
	
	fin.close();
	return;
}

bool Manager::LOAD(const char* filename)
{
	ifstream fin(filename);
	if (!fin) {
		return false; // File not found
	}
	
	// Delete existing graph if loaded
	if (load && graph) {
		delete graph;
		graph = nullptr;
		load = 0;
	}
	
	// Determine graph type based on filename
	string fname(filename);
	bool isList = (fname == "graph_L.txt");
	bool isMatrix = (fname == "graph_M.txt");
	
	if (!isList && !isMatrix) {
		fin.close();
		return false;
	}
	
	// Read graph type (0=undirected, 1=directed)
	char graphType;
	fin >> graphType;
	bool directed = (graphType == 'Y');
	
	// Read number of vertices
	int size;
	fin >> size;
	
	// Create appropriate graph
	if (isList) {
		graph = new ListGraph(directed, size);
	} else {
		graph = new MatrixGraph(directed, size);
	}
	
	// Read edges
	int from, to, weight;
	while (fin >> from >> to >> weight) {
		graph->insertEdge(from, to, weight);
	}
	
	fin.close();
	load = 1;
	
	fout << "========LOAD========" << endl;
	fout << "Success" << endl;
	fout << "====================" << endl << endl;
	
	return true;
}

bool Manager::PRINT()	
{
	if (!load || !graph) {
		return false;
	}
	return graph->printGraph(&fout);
}

bool Manager::mBFS(char option, int vertex)	
{
	if (!load || !graph) {
		return false;
	}
	// Check if vertex is valid
	if (vertex < 0 || vertex >= graph->getSize()) {
		return false;
	}
	return BFS(graph, option, vertex, &fout);
}

bool Manager::mDFS(char option, int vertex)	
{
	if (!load || !graph) {
		return false;
	}
	// Check if vertex is valid
	if (vertex < 0 || vertex >= graph->getSize()) {
		return false;
	}
	return DFS(graph, option, vertex, &fout);
}

bool Manager::mDIJKSTRA(char option, int vertex)	
{
	if (!load || !graph) {
		return false;
	}
	// Check if vertex is valid
	if (vertex < 0 || vertex >= graph->getSize()) {
		return false;
	}
	return Dijkstra(graph, option, vertex, &fout);
}

bool Manager::mKRUSKAL()
{
	if (!load || !graph) {
		return false;
	}
 	return Kruskal(graph, &fout);
}

bool Manager::mBELLMANFORD(char option, int s_vertex, int e_vertex) 
{
	if (!load || !graph) {
		return false;
	}
	// Check if vertices are valid
	if (s_vertex < 0 || s_vertex >= graph->getSize() ||
		e_vertex < 0 || e_vertex >= graph->getSize()) {
		return false;
	}
	return Bellmanford(graph, option, s_vertex, e_vertex, &fout);
}

bool Manager::mFLOYD(char option)
{
	if (!load || !graph) {
		return false;
	}
	return FLOYD(graph, option, &fout);
}

bool Manager::mCentrality() {
	if (!load || !graph) {
		return false;
	}
	return Centrality(graph, &fout);
}

void Manager::printErrorCode(int n)
{
	fout << "========ERROR========" << endl;
	fout << n << endl;
	fout << "====================" << endl << endl;
}

