//============================================================================
// Name        : Graph.cpp
// Desc.       : Implement graph.h, See comment below
// Author      : Jeffrey Caruso, Yusuf Pisan
// Date    	   : Fall 2023
//============================================================================
/**
 * A graph is made up of vertices and edges.
 * Vertex labels are unique.
 * A vertex can be connected to other vertices via weighted, directed edge.
 * A vertex cannot connect to itself or have multiple edges to the same vertex
 */

#include "graph.h"
#include <algorithm>
#include <cassert>
#include <map>
#include <utility>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <queue>
#include <stack>
#include <set>
#include <limits>
#include <cstring>

using namespace std;

Graph::Graph(bool directedEdges) { DirectionalEdges = directedEdges; }

Graph::~Graph() {}

// @return total number of vertices
int Graph::verticesSize() const { return -1; }

// @return total number of edges
int Graph::edgesSize() const
{
	//Your code here

	return -1;
}

// @return number of edges from given vertex, -1 if vertex not found
int Graph::neighborsSize(const string &Label) const
{
	//Your code here
	
	return -1;
}

// @return true if vertex added, false if it already is in the graph
bool Graph::add(const string &Label)
{
	//Your code here
	return false;
}

/** return true if vertex already in graph */
bool Graph::contains(const std::string &Label) const { return vertices.find(Label) != vertices.end(); }


//Sorting Algo
// <
//for getEdgesAsString
bool cmp(Edge &v1, Edge &v2)
{
	if (strcmp(v1.getEnd(v1).c_str(), v2.getEnd(v2).c_str()) == 0)
	{
		return false;
	}
	else
	{
		if (strcmp(v1.getEnd(v1).c_str(), v2.getEnd(v2).c_str()) < 0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
}

// @return string representing edges and weights, "" if vertex not found
// A-3->B, A-5->C should return B(3),C(5)
string Graph::getEdgesAsString(const string &Label) const
{
	//Your code here

	string ans = "";
	return ans;
}

// @return true if successfully connected, false if not connected.
bool Graph::connect(const string &From, const string &To, int Weight)
{
	//Your code here

	return false;
}

// @return true if edge successfully deleted
bool Graph::disconnect(const string &From, const string &To)
{
	//Your code here

	return false;
}

// depth-first traversal starting from given startLabel
void Graph::dfs(const string &StartLabel, void Visit(const string &Label))
{
	//Your code here

}

// breadth-first traversal starting from startLabel
void Graph::bfs(const string &StartLabel, void Visit(const string &Label))
{
	//Your code here

}

// used to sort pairs of vertices and weights
struct CompareBySecond
{
	bool operator()(const std::pair<std::string, int> &a,
					const std::pair<std::string, int> &b) const
	{
		return a.second > b.second;
	}
};

// store the weights in a map
// store the previous label in a map
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &StartLabel) const
{
	//Your code here
	map<string, int> weights;
	map<string, string> previous;

	return make_pair(weights, previous);
}

/**
 * minimum spanning tree
 * @param function to be called on each edge
 * @return length of the minimum spanning tree or -1 if start vertex not found
 */
int Graph::mst(const string &StartLabel,
			   void Visit(const string &From, const string &To,
						  int Weight))
{
	//Your code here
	return -1;
}

// read a text file and create the graph
//provided by starter file
bool Graph::readFile(const char *filename)
{
	ifstream myfile(filename);
	if (!myfile.is_open())
	{
		cerr << "Failed to open " << filename << endl;
		return false;
	}
	int edges = 0;
	int weight = 0;
	string fromVertex;
	string toVertex;
	myfile >> edges;
	for (int i = 0; i < edges; ++i)
	{
		myfile >> fromVertex >> toVertex >> weight;
		connect(fromVertex, toVertex, weight);
	}
	myfile.close();
	return true;
}
