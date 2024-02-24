#ifndef EDGE_H
#define EDGE_H

#include "vertex.h"

using namespace std;

class Vertex;

class Edge {
public:
  // Cosntructor for this class.
  Edge() = default;

  // Starting point of edge.
  Vertex *source;

  // Ending point of edge.
  Vertex *target;

  // Weight of edge.
  int weight;

private:
};
#endif