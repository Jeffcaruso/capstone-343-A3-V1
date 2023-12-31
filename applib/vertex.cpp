//============================================================================
// Name        : Vertex.cpp
// Desc/       : Implement vertex.h, See comment below
// Author      : Jeffrey Caruso, Yusuf Pisan
// Date    	   : Fall 2023
//============================================================================
/**
 * A Graph is made up of Vertex objects that hold data values
 * A vertex is connected to other vertices via Edges
 * A vertex can be visited/unvisited
 * Can connect to another vertex via directed edge with weight
 * The edge can be disconnected
 * A vertex cannot have an edge back to itself
 * getNextNeighbor returns the next neighbor each time it is called
 * when there are no more neighbors, the vertex label is returned
 */

#include "vertex.h"
#include "edge.h"
#include <algorithm>

using namespace std;

ostream &operator<<(ostream &Os, const Vertex &V) { return Os; }
Vertex::Vertex(const string &Label) {label = Label;}

Vertex::~Vertex() {}
