//============================================================================
// Name        : Vertex 
// File Desc.  : Implement Vertex
// Author(s)   : Yusuf Pisan pisan@uw.edu, Jeffrey Caruso jc12321@uw.edu
// Date    	   : Fall 2023
//============================================================================
#include "vertex.h"

/// destructor()
//-----------------------------------------------------
// Description: This method destroys an instance of the
// Vertex class, iterates and deletes all edges that
// connect to this vertex.
//
// Precondition: An instance of this class should exist.
//
// Postcondition: None.
//-----------------------------------------------------
//
Vertex::~Vertex() {
  for (Edge *edge : edges) {
    delete edge;
  }

  edges.clear();
}