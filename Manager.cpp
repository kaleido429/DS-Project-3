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
	fout.open("log.txt", ios::trunc); // Open with truncate mode to overwrite existing log
	load = 0;	//Anything is not loaded
}

Manager::~Manager()
{
	if(load)	//if graph is loaded, delete graph
		delete graph;	
	if(fout.is_open())	//if fout is opened, close file
		fout.close();	//close log.txt File
}

void Manager::run(const char* command_txt){
	ifstream fin;	//Command File Input File Stream
	fin.open(command_txt, ios_base::in);//File open with read mode
		
	if(!fin) { //If command File cannot be read, Print error
		fout<<"command file open error"<<endl;
		return;	//Return
	}
	
	string line;
	while(getline(fin, line)) {
		// Skip empty lines
		if(line.empty()) continue;
		
		// Parse command and arguments
		vector<string> tokens;
		stringstream ss(line);
		string token;
		while(ss >> token) {
			tokens.push_back(token);
		}
		
		if(tokens.empty()) continue;
		
		string cmd = tokens[0];
		
		if(cmd == "LOAD") {
			if(tokens.size() != 2) {
				printErrorCode(100);
			} else {
				if(!LOAD(tokens[1].c_str())) {
					printErrorCode(100);
				}
			}
		}
		else if(cmd == "PRINT") {
			if(tokens.size() != 1) {
				printErrorCode(200);
			} else {
				if(!PRINT()) {
					printErrorCode(200);
				}
			}
		}
		else if(cmd == "BFS") {
			if(tokens.size() != 3) {
				printErrorCode(300);
			} else {
				char option = tokens[1][0];
				int vertex = stoi(tokens[2]);
				if(!mBFS(option, vertex)) {
					printErrorCode(300);
				}
			}
		}
		else if(cmd == "DFS") {
			if(tokens.size() != 3) {
				printErrorCode(400);
			} else {
				char option = tokens[1][0];
				int vertex = stoi(tokens[2]);
				if(!mDFS(option, vertex)) {
					printErrorCode(400);
				}
			}
		}
		else if(cmd == "KRUSKAL") {
			if(tokens.size() != 1) {
				printErrorCode(500);
			} else {
				if(!mKRUSKAL()) {
					printErrorCode(500);
				}
			}
		}
		else if(cmd == "DIJKSTRA") {
			if(tokens.size() != 3) {
				printErrorCode(600);
			} else {
				char option = tokens[1][0];
				int vertex = stoi(tokens[2]);
				if(!mDIJKSTRA(option, vertex)) {
					printErrorCode(600);
				}
			}
		}
		else if(cmd == "BELLMANFORD") {
			if(tokens.size() != 4) {
				printErrorCode(700);
			} else {
				char option = tokens[1][0];
				int s_vertex = stoi(tokens[2]);
				int e_vertex = stoi(tokens[3]);
				if(!mBELLMANFORD(option, s_vertex, e_vertex)) {
					printErrorCode(700);
				}
			}
		}
		else if(cmd == "FLOYD") {
			if(tokens.size() != 2) {
				printErrorCode(800);
			} else {
				char option = tokens[1][0];
				if(!mFLOYD(option)) {
					printErrorCode(800);
				}
			}
		}
		else if(cmd == "CENTRALITY") {
			if(tokens.size() != 1) {
				printErrorCode(900);
			} else {
				if(!mCentrality()) {
					printErrorCode(900);
				}
			}
		}
		else if(cmd == "EXIT") {
			fout << "========EXIT========" << endl;
			fout << "Success" << endl;
			fout << "====================" << endl;
			break;
		}
	}
	
	fin.close();
	return;
}

bool Manager::LOAD(const char* filename)
{
	ifstream fin(filename);
	if(!fin.is_open()) {
		return false;
	}
	
	// Delete existing graph if any
	if(load && graph) {
		delete graph;
		graph = nullptr;
		load = 0;
	}
	
	string type;
	int size;
	
	fin >> type >> size;
	
	if(type == "L") {
		// List format - false indicates adjacency list storage type
		graph = new ListGraph(false, size);
		
		int from, to, weight;
		while(fin >> from >> to >> weight) {
			graph->insertEdge(from, to, weight);
		}
	}
	else if(type == "M") {
		// Matrix format - true indicates adjacency matrix storage type
		graph = new MatrixGraph(true, size);
		
		for(int i = 0; i < size; i++) {
			for(int j = 0; j < size; j++) {
				int weight;
				fin >> weight;
				if(weight != 0) {
					graph->insertEdge(i, j, weight);
				}
			}
		}
	}
	else {
		fin.close();
		return false;
	}
	
	fin.close();
	load = 1;
	
	fout << "========LOAD========" << endl;
	fout << "Success" << endl;
	fout << "====================" << endl;
	fout << endl;
	
	return true;
}

bool Manager::PRINT()	
{
	if(!load || !graph) {
		return false;
	}
	
	fout << "========PRINT========" << endl;
	graph->printGraph(&fout);
	fout << "====================" << endl;
	fout << endl;
	
	return true;
}

bool Manager::mBFS(char option, int vertex)	
{
	if(!load || !graph) {
		return false;
	}
	// Check if vertex is valid
	if(vertex < 0 || vertex >= graph->getSize()) {
		return false;
	}
	
	fout << "========BFS========" << endl;
	if(!BFS(graph, option, vertex, &fout)) {
		return false;
	}
	fout << "====================" << endl;
	fout << endl;
	
	return true;
}

bool Manager::mDFS(char option, int vertex)	
{
	if(!load || !graph) {
		return false;
	}
	// Check if vertex is valid
	if(vertex < 0 || vertex >= graph->getSize()) {
		return false;
	}
	
	fout << "========DFS========" << endl;
	if(!DFS(graph, option, vertex, &fout)) {
		return false;
	}
	fout << "====================" << endl;
	fout << endl;
	
	return true;
}

bool Manager::mDIJKSTRA(char option, int vertex)	
{
	if(!load || !graph) {
		return false;
	}
	// Check if vertex is valid
	if(vertex < 0 || vertex >= graph->getSize()) {
		return false;
	}
	
	fout << "========DIJKSTRA========" << endl;
	if(!Dijkstra(graph, option, vertex, &fout)) {
		return false;
	}
	fout << "====================" << endl;
	fout << endl;
	
	return true;
}

bool Manager::mKRUSKAL()
{
	if(!load || !graph) {
		return false;
	}
	
	fout << "========KRUSKAL========" << endl;
	if(!Kruskal(graph, &fout)) {
		return false;
	}
	fout << "====================" << endl;
	fout << endl;
	
	return true;
}

bool Manager::mBELLMANFORD(char option, int s_vertex, int e_vertex) 
{
	if(!load || !graph) {
		return false;
	}
	// Check if vertices are valid
	if(s_vertex < 0 || s_vertex >= graph->getSize() ||
	   e_vertex < 0 || e_vertex >= graph->getSize()) {
		return false;
	}
	
	fout << "========BELLMANFORD========" << endl;
	if(!Bellmanford(graph, option, s_vertex, e_vertex, &fout)) {
		return false;
	}
	fout << "====================" << endl;
	fout << endl;
	
	return true;
}

bool Manager::mFLOYD(char option)
{
	if(!load || !graph) {
		return false;
	}
	
	fout << "========FLOYD========" << endl;
	if(!FLOYD(graph, option, &fout)) {
		return false;
	}
	fout << "====================" << endl;
	fout << endl;
	
	return true;
}

bool Manager::mCentrality() {
	if(!load || !graph) {
		return false;
	}
	
	fout << "========CENTRALITY========" << endl;
	if(!Centrality(graph, &fout)) {
		return false;
	}
	fout << "====================" << endl;
	fout << endl;
	
	return true;
}

void Manager::printErrorCode(int n)
{
	fout<<"========ERROR========"<<endl;
	fout<<n<<endl;
	fout<<"====================="<<endl << endl;
}

