/**
 * Edge is the simplest structure of the graph
 * All edges are directed
 * Each edge belongs to a vertex
 */

#include "edge.h"


Edge::Edge(Vertex *From, Vertex *To, int Weight) {
    start = (*From).label; 
    end = (*To).label; 
    weight = Weight;
}

Edge &operator==(Edge &v1, Edge &v2){
    // TODO: insert return statement here
    if (strcmp(v1.start, v2.start) == 0)
    {
        if (strcmp(v1.end, v2.end) == 0)
        {
            if(v1.weight == v2.weight)
            {
                return true;
            }
        }
    }
    return false;
}

Edge &operator<(Edge &v1, Edge &v2)
{
    // TODO: insert return statement here
    if (strcmp(v1.start, v2.start) == 0)
    {
        return false;
    }
    else
    {
        if(strcmp(v1.start, v2.start) < 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
