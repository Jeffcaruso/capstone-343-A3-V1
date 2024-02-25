//============================================================================
// Name        : Graph
// File Desc.  : implement Graph
// Author(s)   : Yusuf Pisan pisan@uw.edu, Jeffrey Caruso jc12321@uw.edu
// Date    	   : Fall 2023
//============================================================================
#include "graph.h"
#include <algorithm>
#include <climits>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <utility>
#include <vector>

using namespace std;

// Graph()
// @param directionalEdges If graph will be direct.
// @return An instance of this class.
Graph::Graph(bool directionalEdges) {
  this->directionalEdges = directionalEdges;
}

// destructor()
// This method destroys an instance of the Graph class by iterating through vector of vertices.
Graph::~Graph() {
  for (auto iter = vertices.begin(); iter != vertices.end();) {
    delete iter->second;
    iter->second = nullptr;
    iter++;
  }

  vertices.clear();
}

// verticesSize()
// @return Number of vertices.
int Graph::verticesSize() const { return vertices.size(); }

// edgesSize()
// @return Total number of edges.
int Graph::edgesSize() const { return numOfEdges; }


// neighborsSize()
// @return number of edges from given vertex, -1 if vertex not found
int Graph::neighborsSize(const string &label) const {
  return -1;
}

// add()
bool Graph::add(const string &label) {
  return false;
}

// contains()
/** return true if vertex already in graph */
bool Graph::contains(const string &label) const {
  return false;
}

// getEdgesAsString()
string Graph::getEdgesAsString(const string &label) const {
  return "";
}

// connect()
bool Graph::connect(const string &from, const string &to, int weight) {
  return false;
}

// disconnect()
bool Graph::disconnect(const string &from, const string &to) {
  return false;
}

// dfs()
void Graph::dfs(const string &startLabel, void visit(const string &label)) {
  return;
}

// bfs()
void Graph::bfs(const string &startLabel, void visit(const string &label)) {
  return;
}

// dijsktra()
// store the weights in a map
// store the previous label in a map
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &startLabel) const
{
  map<string, int> Weights;
  map<string, string> Previous;
  return make_pair(Weights, Previous);
}

/**
 * minimum spanning tree mst (mst prim)
 * @param function to be called on each edge
 * @return length of the minimum spanning tree or -1 if start vertex not found
 */
int Graph::mst(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const {
 return -1;
}

// getVertex()
//-----------------------------------------------------
// Description: This method retrieves a vertex from the
// vertices map from this instance of the graph class.
Vertex *Graph::getVertex(const string &label) const {
  return nullptr;
}

// read a text file and create the graph
bool Graph::readFile(const string &filename) {
  return false;
}