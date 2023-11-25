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
    // TODO: insert return statement here
    if (strcmp(v1.start.c_str(), v2.start.c_str()) == 0)
    {
        if (strcmp(v1.end.c_str(), v2.end.c_str()) == 0)
        {
            if(v1.weight == v2.weight)
            {
                return true;
            }
        }
    }
    return false;
}

//v1 less than v2
bool operator<(Edge &v1, Edge &v2)
{
    if (strcmp(v1.end.c_str(), v2.end.c_str()) < 0)
    {
        return true; // expected
        // return false;
    }
    else
    {
        //would also include == -> false
        return false;
    }
}


