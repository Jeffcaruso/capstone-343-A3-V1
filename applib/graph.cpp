#include "graph.h"
#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits.h>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <utility>
#include <vector>

using namespace std;

// constructor, empty graph
// directionalEdges defaults to true
Graph::Graph(bool directionalEdges) {
  this->directionalEdges = directionalEdges;
}

// destructor
Graph::~Graph() {
  for (auto i = vertices.begin(); i != vertices.end(); i++) {
    delete i->second;
    i->second = nullptr;
  }
  vertices.clear();
}

// @return total number of vertices
int Graph::verticesSize() const { return vertices.size(); }

// @return total number of edges
int Graph::edgesSize() const {
  int totalEdges = 0;
  for (const auto &pair : vertices) {
    totalEdges += pair.second->edges.size();
  }
  return totalEdges;
}

// @return number of edges from given vertex, -1 if vertex not found
int Graph::vertexDegree(const string &label) const {
  if (!contains(label)) {
    return -1;
  }
  return vertices.at(label)->edges.size();
}

// @return true if vertex added, false if it already is in the graph
bool Graph::add(const string &label) {
  if (contains(label)) {
    return false;
  }
  vertices[label] = new Vertex(label);
  return true;
}

/** return true if vertex already in graph */
bool Graph::contains(const string &label) const {
  // returns the vertex with a given label
  return vertices.find(label) != vertices.end();
}

// @return string representing edges and weights, "" if vertex not found
// A-3->B, A-5->C should return B(3),C(5)
string Graph::getEdgesAsString(const string &label) const {
  string result;
  vector<string> sorted;

  if (vertices.at(label)->edges.empty()) {
    return result;
  }

  // sorts the destination and weight of given edge in the sorted vector
  for (Edge *e : vertices.at(label)->edges) {
    string sWeight = to_string(e->weight);
    // formats the string as it gets pushed onto the vector
    sorted.push_back(e->destination + "(" + sWeight + ")");
  }

  // sorts the edge destinations alphabetically
  sort(sorted.begin(), sorted.end());

  // appends the detinations strings to a single string
  for (string s : sorted) {
    result += s + ",";
  }
  result.replace(result.length() - 1, result.length(), "");
  return result;
}

// @return true if successfully connected
bool Graph::connect(const string &from, const string &to, int weight) {
  // checks if either the start/destination are not in the graph
  // or if you're trying to connect a vertex to itself
  if (from == to) {
    return false;
  }

  // adds the vertex if they don't exists
  if (!(contains(from))) {
    add(from);
  }
  if (!(contains(to))) {
    add(to);
  }

  // loops though the edges of a vertex and checks if
  // an edge with the given start and destination exists
  for (Edge *e : vertices[from]->edges) {
    if ((e->start == from && e->destination == to)) {
      return false;
    }
  }

  // adds the new edge to vectors of edges in the vertex
  vertices[from]->edges.push_back(new Edge(from, to, weight));
  if (!directionalEdges) {
    vertices[to]->edges.push_back(new Edge(to, from, weight));
  }
  return true;
}

// disconnects an edge between to vertexs
bool Graph::disconnect(const string &from, const string &to) {
  // checks if the start or destination are contained in the graph
  if (!(contains(from)) || !(contains(to))) {
    return false;
  }

  // goes to vertex with the given label of the start
  for (auto i = vertices[from]->edges.begin(); i != vertices[from]->edges.end();
       i++) {
    // checks if the destinationo of the vertex's edge
    // is equal to the given destination
    if ((*i)->destination == to) {
      Edge *e = *i;
      delete e;
      vertices[from]->edges.erase(i);
      return true;
    }
  }

  return false;
}

// depth-first traversal starting from given startLabel
void Graph::dfs(const string &startLabel, void visit(const string &label)) {
  // Create a set to keep track of seen vertices
  set<string> seen;
  // Call the DFS helper function
  if (contains(startLabel)) {
    dfsHelper(startLabel, visit, seen);
  }
}

// checks if the current vertex has been seen then moves to the neighbor 
void Graph::dfsHelper(const string &currentLabel,
                      void visit(const string &label), set<string> &seen) {
  seen.insert(currentLabel);
  visit(currentLabel);

  if (!vertices[currentLabel]->edges.empty()) {
    queue<string> sorted;
    vector<string> unsorted;
    for (Edge *e : vertices[currentLabel]->edges) {
      unsorted.push_back(e->destination);
    }

    sort(unsorted.begin(), unsorted.end());

    for (string s : unsorted) {
      sorted.push(s);
    }

    // Iterate through the edges of the current vertex
    while (!sorted.empty()) {
      // Check if the neighbor has not been visited yet
      if (seen.find(sorted.front()) == seen.end()) {
        dfsHelper(sorted.front(), visit, seen);
      }
      sorted.pop();
    }
  }
}

// breadth-first traversal starting from startLabel
void Graph::bfs(const string &startLabel, void visit(const string &label)) {
  // Create a set to keep track of visited vertices
  set<string> seen;
  queue<string> levels;
  // Call the BFS helper function
  if (contains(startLabel)) {
    bfsHelper(startLabel, visit, seen);
  }
}

// checks if the vertex has been seen then checks all current neighbors
void Graph::bfsHelper(const string &currentLabel,
                      void visit(const string &label), set<string> &seen) {
  if (seen.find(currentLabel) == seen.end()) {
    seen.insert(currentLabel);
    visit(currentLabel);
  }

  if (!vertices[currentLabel]->edges.empty()) {
    queue<string> sorted;
    vector<Edge *> currentEdges = vertices[currentLabel]->edges;

    sort(currentEdges.begin(), currentEdges.end());

    for (Edge *e : currentEdges) {
      sorted.push(e->destination);
    }

    string nextLabel = sorted.front();
    // Iterate through the edges of the current vertex
    while (!sorted.empty()) {
      // Check if the neighbor has not been visited yet
      if (seen.find(sorted.front()) == seen.end()) {
        seen.insert(sorted.front());
        visit(sorted.front());
        bfsHelper(nextLabel, visit, seen);
      }
      sorted.pop();
    }
  }
}

// store the weights in a map
// store the previous label in a map
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &startLabel) const {
  map<string, int> weights;
  map<string, string> previous;
  map<Vertex *, int> q;

  if (!contains(startLabel)) {
    return make_pair(weights, previous);
  }

  weights[startLabel] = 0;

  //
  for (const auto pair : vertices) {
    Vertex *current = pair.second;
    if (current->label != startLabel) {
      weights[current->label] = INT_MAX;
      previous[current->label] = "";
    }
    q[current] = weights[current->label];
  }

  while (!q.empty()) {
    string u = removeMin(q);

    if (vertices.at(u)->edges.empty()) {
      break;
    }

    for (Edge *e : vertices.at(u)->edges) {
      int alt = weights[u] + e->weight;
      if (alt < weights[e->destination]) {
        weights[e->destination] = alt;
        previous[e->destination] = u;
        q[vertices.at(e->destination)] = alt;
      }
    }
  }

  auto pIndex = previous.begin();

  for (auto wIndex = weights.begin(); wIndex != weights.end(); wIndex++) {
    if (wIndex->second == INT_MAX) {
      wIndex = weights.erase(wIndex);
      pIndex = previous.erase(pIndex);
    }
    pIndex++;
  }

  weights.erase(startLabel);
  previous.erase(startLabel);

  return make_pair(weights, previous);
}

// removes the lowest value from priority map
string Graph::removeMin(map<Vertex *, int> &q) {
  Vertex *v;
  int min = INT_MAX;
  // for each vertex compare the priority to the minimum
  for (auto pair : q) {
    if (pair.second < min) {
      v = pair.first;
      min = pair.second;
    }
  }

  if (v != nullptr) {
    q.erase(v);
  }

  return v->label;
}

// minimum spanning tree using Prim's algorithm
int Graph::mstPrim(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const {
  // if the vertex doesn't exist return -1
  if (!contains(startLabel)) {
    return -1;
  }

  stack<string> unvisited;
  map<string, vector<Edge *>> edgeCosts;
  map<string, string> visitedEdges;

  // collect all the edges
  for (auto pair : vertices) {
    string label = pair.first;
    edgeCosts[label] = pair.second->edges;
    unvisited.push(pair.first);
  }
  return mstPrimHelper(startLabel, visit, edgeCosts, unvisited, visitedEdges);
}

// finds the least code edge of each vertex and tracks the edges visited
int Graph::mstPrimHelper(const string &currentLabel,
                         void visit(const string &from, const string &to,
                                    int weight),
                         map<string, vector<Edge *>> edgeCosts,
                         stack<string> unvisited,
                         map<string, string> visitedEdges) {
  int min = INT_MAX;
  string start;
  string destination;
  // for each edge compare the weight to the minimum weight currently
  for (Edge *e : edgeCosts[currentLabel]) {
    if ((e->weight < min) && (visitedEdges[e->start] != e->destination)) {
      min = e->weight;
      start = e->start;
      destination = e->destination;
    }
  }

  // if no new minimum is found set the min to zero
  if (min == INT_MAX) {
    min = 0;
  }

  // remove the current node from the stack
  string prev = unvisited.top();
  unvisited.pop();
  // when the stack is empty return no change
  if (unvisited.empty()) {
    return 0;
  }
  // update the edges visited
  visitedEdges[start] = destination;
  visitedEdges[destination] = start;
  visit(start, destination, min);
  return min + mstPrimHelper(unvisited.top(), visit, edgeCosts, unvisited,
                             visitedEdges);
}

// // minimum spanning tree using Kruskal's algorithm
// int Graph::mstKruskal(void visit(const string &from, const string &to,
//                                  int weight)) const {
//   return -1;
// }

// read a text file and create the graph
bool Graph::readFile(const string &filename) {
  ifstream myfile(filename);
  if (!myfile.is_open()) {
    cerr << "Failed to open " << filename << endl;
    return false;
  }
  int edges = 0;
  int weight = 0;
  string fromVertex;
  string toVertex;
  myfile >> edges;
  for (int i = 0; i < edges; ++i) {
    myfile >> fromVertex >> toVertex >> weight;
    connect(fromVertex, toVertex, weight);
  }
  myfile.close();
  return true;
}