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
bool operator<(const Edge &v1, const Edge &v2)
{
    if (strcmp(v1.start.c_str(), v2.start.c_str()) == 0)
    {
        return false;
    }
    else
    {
        //I think these are correct, but jist in case I got them flipped, lets see...
        // v#.end.c_str() ?
        if(strcmp(v1.start.c_str(), v2.start.c_str()) < 0)
        {
            return true;  //expected
            //return false;
        }
        else
        {
            return false; // expected
            //return true;
        }
    }
}

//alt option to actually sort...?
bool cmp(const Edge &v1, const Edge &v2)
{
    if (strcmp(v1.start.c_str(), v2.start.c_str()) == 0)
    {
        return false;
    }
    else
    {
        //I think these are correct, but jist in case I got them flipped, lets see...
        // v#.end.c_str() ?
        if(strcmp(v1.start.c_str(), v2.start.c_str()) < 0)
        {
            return true;  //expected
            //return false;
        }
        else
        {
            return false; // expected
            //return true;
        }
    }
}

