/**
 * Edge is the simplest structure of the graph
 * All edges are directed
 * Each edge belongs to a vertex
 */

#include "edge.h"

#include <cstring>
#include <iostream>


Edge::Edge(Vertex *From, Vertex *To, int Weight) {
    start = (*From).label; 
    end = (*To).label; 
    weight = Weight;
}

bool &operator==(Edge &v1, Edge &v2){
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

bool &operator<(Edge &v1, Edge &v2)
{
    // TODO: insert return statement here
    if (strcmp(v1.start.c_str(), v2.start.c_str()) == 0)
    {
        return false;
    }
    else
    {
        if(strcmp(v1.end.c_str(), v2.end.c_str()) < 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
