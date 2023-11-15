/**
 * A vertex stores a list of all connected verticies.
 * Vertexes store edges, and the weights of those edges. 
 */

#ifndef VERTEX_H
#define VERTEX_H

#include <map>
#include <string>
#include <vector>

using namespace std;

class Vertex {
public:
  // constructor, vertex with no connections
  explicit Vertex(string label);

  // copy not allowed
  Vertex(const Vertex &other) = delete;

  // move not allowed
  Vertex(Vertex &&other) = delete;

  // assignment not allowed
  Vertex &operator=(const Vertex &other) = delete;

  // move assignment not allowed
  Vertex &operator=(Vertex &&other) = delete;

  /** destructor, delete the vertex */
  ~Vertex();

  // Determine if a vertex is connected to another one
  // @return true if the vertex has a connection to the given vertex
  bool connects(const string &label) const;

  // Add a connection to another vertex
  // @return true if the vertex is NOW connected to the given vertex
  bool add(string label, int weight);

  // Removes a connection to another vertex
  // @return true if the vertex is NO LONGER connected to the given vertex
  bool remove(string label);

  // Returns the number of connected edges a vertex has
  // @return int number representing number of outgoing connecting edges
  int getDegree();

  // Returns the contents of the connection map as string
  // @return string representing the connections of the vertex
  string getEdgesString();

  // Returns the contents of the connection map as vector
  // @return vector representing the connections of the vertex
  vector<string> getEdgesVector();

  // Returns the contents of the connection map as vector pair
  // @return vector representing the connections and weights of the vertex
  vector<pair<string,int>> getWeightedEdgesVector();

private:
  //A map containing the label names of all connecting edges from this vertex, and their weighted values.
  map<string,int> edges; 
  //The label of our vertex
  string label;
};

#endif // VERTEX_H