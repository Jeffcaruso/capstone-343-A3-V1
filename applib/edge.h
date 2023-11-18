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

  friend &operator==(Vertex &v1, Vertex &v2);
  friend &operator<(Vertex &v1, Vertex &v2);

private:
  /** constructor with label and weight */
  Edge(Vertex *From, Vertex *To, int Weight);

  string start;
  string end;
  int weight;
};

#endif
