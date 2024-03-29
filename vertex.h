//============================================================================
// Name        : Vertex 
// File Desc.  : Define Vertex
// Author(s)   : Yusuf Pisan pisan@uw.edu, Jeffrey Caruso jc12321@uw.edu
// Date    	   : Fall 2023
//============================================================================
#ifndef VERTEX_H
#define VERTEX_H

#include "edge.h"

#include <string>
#include <vector>

using namespace std;

class Edge;

class Vertex {
public:
  // Constructo for this class.
  Vertex() = default;

  // Destructor for this class.
  ~Vertex();

  // Label corresponding to this vertex.
  string label;

  // Vector of edges connnected to this vertex.
  vector<Edge *> edges;

private:
};
#endif