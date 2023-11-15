/**
 * A graph is made up of vertices and edges.
 * Vertex labels are unique.
 * A vertex can be connected to other vertices via weighted, directed edge.
 * A vertex cannot connect to itself or have multiple edges to the same vertex
 */

#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <string>
#include <vector>

using namespace std;

class Graph {

  // some forward declarations
  struct Edge;
  struct Vertex;

  // a map of vertexes and thier labels
  map<string, Vertex *> vmap;

  // a map of the edges in the graph
  vector<Edge *> edges;

private:
  // implementation of an edge
  struct Edge {
    Vertex *to;
    Vertex *from;
    int weight;

    // if the to or/and from vertex don't exist they will be created
    // an edge storing to and from vertex * and weight will be made
    explicit Edge(string inFrom, string inTo, int inWeight, Graph *g) {
      weight = inWeight;
      // todo if directional
      if (g->vmap.count(inTo) == 1 &&
          g->vmap.count(inFrom) == 1) { // both vertex are already in the graph
        to = g->vmap[inTo];     // setting the edge's to* to the vertex in vmap
        from = g->vmap[inFrom]; // same lol
      } else if (g->vmap.count(inTo) == 1) { // need to create new from vertex
        to = g->vmap[inTo];
        Vertex *newFrom = new Vertex(inFrom);
        g->vmap[inFrom] = newFrom;
        from = newFrom;
      } else if (g->vmap.count(inFrom) == 1) { // need to create new to vertex
        from = g->vmap[inFrom];
        Vertex *newTo = new Vertex(inTo);
        g->vmap[inTo] = newTo;
        to = newTo;
      } else {
        Vertex *newTo = new Vertex(inTo);
        g->vmap[inTo] = newTo;
        to = newTo;
        Vertex *newFrom = new Vertex(inFrom);
        g->vmap[inFrom] = newFrom;
        from = newFrom;
      }
    }
  };

  // implementation of a vertex
  struct Vertex {
    string label;
    vector<Edge *> neighbors;

    explicit Vertex(string str) { label = str; }
  };

public:
  // method that checks if the edge already exists
  bool isDupeEdge(string from, string to);

  // method that checks if the edge is in the provided vector
  static bool isEdgeInVector(const Edge *edgeToFind,
                             const std::vector<Edge *> &edgeVector);

  // variable reflecting directionality of graph
  bool isDirectional = true;

  // checks to see if the edge already exists
  bool edgeExists(const string &from, const string &to);

  // returns a list of Edges adjacent to the miniumum spanning tree
  vector<Edge *> adjToMst(vector<Edge *> mst, set<string> visited) const;

  // returns the edge the closest to mst
  static Edge *getMinEdge(vector<Edge *> adjToMst);

  // returns the index of the edge
  int edgeIndex(const string &to);

  // returns the index of the edge in the neighbor list of a vertex
  int edgeNeighborIndex(const string &from, const string &to) const;

  // sorts the provided stack
  static stack<Vertex *> sortStack(std::stack<Vertex *> inputStack);

  // gets the vertex that matches the input
  Vertex *get(string input) const;

  // the first node in the graph;
  Vertex *source;

  // default constructor
  Graph();

  // constructor, empty graph
  explicit Graph(bool directionalEdges);

  // copy not allowed
  Graph(const Graph &other) = delete;

  // move not allowed
  Graph(Graph &&other) = delete;

  // assignment not allowed
  Graph &operator=(const Graph &other) = delete;

  // move assignment not allowed
  Graph &operator=(Graph &&other) = delete;

  /** destructor, delete all vertices and edges */
  ~Graph();

  // @return true if vertex added, false if it already is in the graph
  bool add(const string &label);

  // @return true if vertex is in the graph
  bool contains(const string &label) const;

  // @return total number of vertices
  int verticesSize() const;

  // Add an edge between two vertices, create new vertices if necessary
  // A vertex cannot connect to itself, cannot have P->P
  // For digraphs (directed graphs), only one directed edge allowed, P->Q
  // Undirected graphs must have P->Q and Q->P with same weight
  // @return true if successfully connected
  bool connect(const string &from, const string &to, int weight = 0);

  // Remove edge from graph
  // @return true if edge successfully deleted
  bool disconnect(const string &from, const string &to);

  // @return total number of edges
  int edgesSize() const;

  // @return number of edges from given vertex, -1 if vertex not found
  int vertexDegree(const string &label) const;

  // @return string representing edges and weights, "" if vertex not found
  // A-3->B, A-5->C should return B(3),C(5)
  string getEdgesAsString(const string &label) const;

  // Read edges from file
  // first line of file is an integer, indicating number of edges
  // each line represents an edge in the form of "string string int"
  // vertex labels cannot contain spaces
  // @return true if file successfully read
  bool readFile(const string &filename);

  // depth-first traversal starting from given startLabel
  void dfs(const string &startLabel, void visit(const string &label));

  // breadth-first traversal starting from startLabel
  // call the function visit on each vertex label */
  void bfs(const string &startLabel, void visit(const string &label));

  // dijkstra's algorithm to find shortest distance to all other vertices
  // and the path to all other vertices
  // Path cost is recorded in the map passed in, e.g. weight["F"] = 10
  // How to get to the vertex is recorded previous["F"] = "C"
  // @return a pair made up of two map objects, Weights and Previous
  pair<map<string, int>, map<string, string>>
  dijkstra(const string &startLabel) const;

  // minimum spanning tree using Prim's algorithm
  // ONLY works for NONDIRECTED graphs
  // ASSUMES the edge [P->Q] has the same weight as [Q->P]
  // @return length of the minimum spanning tree or -1 if start vertex not
  int mstPrim(const string &startLabel,
              void visit(const string &from, const string &to,
                         int weight)) const;
};

#endif // GRAPH_H