//============================================================================
// Name        : Edge.h
// Desc/       : See comment below
// Author      : Jeffrey Caruso, Yusuf Pisan
// Date    	   : Fall 2023
//============================================================================
/**
 * Edge is the simplest structure of the graph
 * All edges are directed
 * Each edge belongs to a vertex
 */

#ifndef EDGE_H
#define EDGE_H

#include "vertex.h"
#include <string>

// forward declaration for class Vertex
class Vertex;

using namespace std;

class Edge
{

	friend class Vertex;

	friend class Graph;

	friend bool operator==(const Edge &v1, const Edge &v2);
	friend bool operator<(Edge &v1, Edge &v2);

	string start;
	string end;
	int weight;
public:
	string getStart(Edge &e);
	string getEnd(Edge &e);
	int getWeight(Edge &e);


private:
	/** constructor with label and weight */
	Edge(Vertex *From, Vertex *To, int Weight);
};

#endif
