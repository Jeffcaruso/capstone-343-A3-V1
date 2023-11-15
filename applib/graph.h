/**
 * A graph is made up of vertices and edges.
 * Vertex labels are unique.
 * A vertex can be connected to other vertices via weighted, directed edge.
 * A vertex cannot connect to itself or have multiple edges to the same vertex
 */

#ifndef GRAPH_H
#define GRAPH_H

#include <map>
#include <queue>
#include <set>
#include <stack>
#include <string>
#include <vector>

using namespace std;

class Graph {
private:
  // a stucture to represent the connection between two vertexs
  struct Edge {
    string start;
    string destination;
    int weight;
    Edge(const string &s, const string &d, int w)
        : start(s), destination(d), weight(w) {}
  };
  // a stucture to represent a vertex
  struct Vertex {
    string label;
    vector<Edge *> edges;
    explicit Vertex(const string &l) : label(l) {}
    ~Vertex() {
      for (Edge *e : edges) {
        delete e;
      }
      edges.clear();
    }
  };

  map<string, Vertex *> vertices;
  bool directionalEdges;

public:
  // constructor, empty graph
  explicit Graph(bool directionalEdges = true);

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

  // used to recursively travel the graph in depth-first order
  void dfsHelper(const string &currentLabel, void visit(const string &label),
                 set<string> &seen);

  // breadth-first traversal starting from startLabel
  // call the function visit on each vertex label */
  void bfs(const string &startLabel, void visit(const string &label));

  // used to recursively travel the graph in breadth-first order
  void bfsHelper(const string &currentLabel, void visit(const string &label),
                 set<string> &seen);

  // dijkstra's algorithm to find shortest distance to all other vertices
  // and the path to all other vertices
  // Path cost is recorded in the map passed in, e.g. weight["F"] = 10
  // How to get to the vertex is recorded previous["F"] = "C"
  // @return a pair made up of two map objects, Weights and Previous
  pair<map<string, int>, map<string, string>>
  dijkstra(const string &startLabel) const;

  // return the label for the vertex with the minimum weight
  static string removeMin(map<Vertex *, int> &q);

  // minimum spanning tree using Prim's algorithm
  // ONLY works for NONDIRECTED graphs
  // ASSUMES the edge [P->Q] has the same weight as [Q->P]
  // @return length of the minimum spanning tree or -1 if start vertex not
  int mstPrim(const string &startLabel,
              void visit(const string &from, const string &to,
                         int weight)) const;

  // goes through the weights of the edges returning the lowest one and tracking
  // the edge that was visited.
  static int
  mstPrimHelper(const string &currentLabel,
                void visit(const string &from, const string &to, int weight),
                map<string, vector<Edge *>> edgeCosts, stack<string> unvisited,
                map<string, string> visitedEdges);

  // minimum spanning tree using Kruskal's algorithm
  // ONLY works for NONDIRECTED graphs
  // ASSUMES the edge [P->Q] has the same weight as [Q->P]
  // @return length of the minimum spanning tree or -1 if start vertex not
  // int mstKruskal(void visit(const string &from, const string &to,
  //                           int weight)) const;
};

#endif // GRAPH_H