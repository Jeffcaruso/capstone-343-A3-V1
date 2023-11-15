/**
 * A vertex stores a list of all connected verticies.
 * Vertexes store edges, and the weights of those edges.
 */

#include "vertex.h"
#include <map>
#include <string>
#include <vector>

using namespace std;

// constructor, vertex with no connections
Vertex::Vertex(string newLabel) { this->label = newLabel; }

/** destructor, delete the vertex */
Vertex::~Vertex() {}

// Determine if a vertex is connected to another one
// @return true if the vertex is connected to the given vertex
bool Vertex::connects(const string &label) const {
  return (edges.count(label) != 0);
}

// Add a connection to another vertex
// @return true if the vertex is NOW connected to the given vertex
bool Vertex::add(string label, int weight) {
  if (edges.count(label) == 0) {
    edges[label] = weight;
    return true;
  }
  return false; // Element already exists as a connection
}

// Removes a connection to another vertex
// @return true if the vertex is NO LONGER connected to the given vertex
bool Vertex::remove(string label) {
  if (edges.count(label) != 0) {
    edges.erase(label);
    return true;
  }
  return false; // Element doesn't exist
}

// Returns the number of connected edges a vertex has
// @return int number representing number of outgoing connecting edges
int Vertex::getDegree() { return this->edges.size(); }

// Returns the contents of the connection map as string
// @return string representing the connections of the vertex
string Vertex::getEdgesString() {
  string retVal;
  for (auto const &edge : this->edges) {
    retVal += ("," + edge.first + "(" + to_string(edge.second) + ")");
  }
  retVal.erase(0, 1); // Remove extra comma at the front of the string
  return retVal;
}

// Returns the contents of the connection map as vector
// @return vector representing the connections of the vertex
vector<string> Vertex::getEdgesVector() {
  vector<string> retVal;
  for (auto const &edge : this->edges) {

    retVal.push_back(edge.first);
  }
  return retVal;
}

// Returns the contents of the connection map as vector pair
// @return vector representing the connections and weights of the vertex
vector<pair<string, int>> Vertex::getWeightedEdgesVector() {
  vector<pair<string, int>> retVal;
  for (auto const &edge : this->edges) {
    retVal.push_back(edge);
  }
  return retVal;
}