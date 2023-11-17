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
