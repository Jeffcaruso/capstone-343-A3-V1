//============================================================================
// Name        : Edge.cpp
// Desc/       : Implement edge.h, See comment below
// Author      : Jeffrey Caruso, Yusuf Pisan
// Date    	   : Fall 2023
//============================================================================
/**
 * Edge is the simplest structure of the graph
 * All edges are directed
 * Each edge belongs to a vertex
 */

#include "edge.h"

#include <cstring>
#include <iostream>

string Edge::getStart(Edge &e)
{
    return e.start;
}

string Edge::getEnd(Edge &e)
{
    return e.end;
}

int Edge::getWeight(Edge &e)
{
    return e.weight;
}

Edge::Edge(Vertex *From, Vertex *To, int Weight)
{
    start = (*From).label; 
    end = (*To).label; 
    weight = Weight;
}

//v1 equal to v2
bool operator==(const Edge &v1, const Edge &v2){
    //Your code here

    return false;
}

//v1 less than v2
bool operator<(Edge &v1, Edge &v2)
{
    //Your code here

    return false;
}


