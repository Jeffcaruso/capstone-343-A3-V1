#include "graph.h"
#include <algorithm>
#include <cassert>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std;

// constructor, empty graph
// directionalEdges defaults to true
Graph::Graph(bool directionalEdges) {
  source = nullptr;
  isDirectional = directionalEdges;
}

// default constructor
Graph::Graph() {
  source = nullptr;
  isDirectional = true;
}

// destructor
Graph::~Graph() {
  // Clean up all vertices
  for (auto &entry : vmap) {
    delete entry.second;
  }

  //  Clean up all edges
  for (Edge *edge : edges) {
    delete edge;
  }
}

// @return total number of vertices
int Graph::verticesSize() const { return vmap.size(); }

// @return total number of edges
int Graph::edgesSize() const { return edges.size(); }

// @return number of edges from given vertex, -1 if vertex not found
int Graph::vertexDegree(const string &label) const {
  if (vmap.count(label) == 0) {
    return -1;
  }
  std::map<string, Vertex *>::const_iterator it =
      vmap.find(label); // access element of map this way to obey const
  vector<Edge *> edges = it->second->neighbors;
  return edges.size();
}

// @return true if vertex added, false if it already is in the graph
bool Graph::add(const string &label) {
  if (vmap.count(label) == 1) {
    return false;
  }
  vmap[label] = new Vertex(label); //(label);
  return true;
}

/** return true if vertex already in graph */
bool Graph::contains(const string &label) const {
  for (auto element : vmap) {
    if (element.first == label) {
      return true;
    }
  }
  return false;
}

// helps compare when sorting
static bool sorter(const std::pair<std::string, std::string> &left,
                   const std::pair<std::string, std::string> &right) {
  return left.first < right.first;
}

// @return string representing edges and weights, "" if vertex not found
// A-3->B, A-5->C should return B(3),C(5)
string Graph::getEdgesAsString(const string &label) const {
  vector<pair<string, string>> toSort;
  string s;
  std::map<string, Vertex *>::const_iterator it =
      vmap.find(label); // access start Vertex from vmap this way to obey const
  vector<Edge *> edges = it->second->neighbors;
  if (isDirectional) {

    for (int i = 0; i < edges.size(); i++) {
      pair<string, string> pair = make_pair(edges[i]->to->label + "(",
                                            to_string(edges[i]->weight) + ")");
      toSort.push_back(pair);
      sort(toSort.begin(), toSort.end(), sorter);
    }

    for (int i = 0; i < toSort.size(); i++) {
      s += toSort[i].first;
      if (i != toSort.size() - 1) {
        toSort[i].second += ",";
      }
      s += toSort[i].second;
    }
    return s;
  }

  for (auto *element : edges) {
    s += element->from->label + "-";
    s += to_string(element->weight);
    s += "->" + element->to->label + ", ";
  }
  return s;
}

// @return true if successfully connected
// Add an edge between two vertices, create new vertices if necessary
// A vertex cannot connect to itself, cannot have P->P
// For digraphs (directed graphs), only one directed edge allowed, P->Q
// Undirected graphs must have P->Q and Q->P with same weight
// @return true if successfully connected
bool Graph::connect(const string &from, const string &to, int weight) {
  if (from == to) {
    return false;
  }

  if (isDirectional) {

    if (isDupeEdge(from, to)) {
      return false;
    }
    Edge *from2to = new Edge(from, to, weight, this);

    vmap[from]->neighbors.push_back(from2to);
    edges.push_back(from2to);
    return true;
  }

  if (isDupeEdge(to, from)) {
    return false;
  }
  Edge *from2to = new Edge(from, to, weight, this);
  // if vmap doesnt contain the vertexs already
  vmap[from]->neighbors.push_back(from2to);
  edges.push_back(from2to);

  Edge *to2from = new Edge(to, from, weight, this);
  vmap[to]->neighbors.push_back(to2from);
  edges.push_back(to2from);

  return true;
}

// checks if the edge alread exists
bool Graph::isDupeEdge(string from, string to) {
  if (vmap.count(from) == 0) {
    return false;
  }
  for (Edge *element : vmap[from]->neighbors) {
    if (to == element->to->label && from == element->from->label) {
      return true;
    }
  }
  return false;
}

// checks if the edge already exists
bool Graph::edgeExists(const string &from, const string &to) {
  for (Edge *element : edges) {
    if (to == element->to->label && from == element->from->label) {
      return true;
    }
  }
  return false;
}

// returns the index of the edge
int Graph::edgeIndex(const string &to) {
  for (int i = 0; i < edges.size(); i++) {
    if (edges[i]->to->label == to) {
      return i;
    }
  }
  return -1;
}

// returns the index of the edge in the provided vertex nieghbor list
int Graph::edgeNeighborIndex(const string &from, const string &to) const {
  for (int i = 0; i < get(from)->neighbors.size(); i++) {
    // find index of desired edge in vmap[from]
    if (get(from)->neighbors[i]->to->label == to) {
      return i;
    }
  }
  return -1;
}

// Disconnects two vertexs
bool Graph::disconnect(const string &from, const string &to) {
  if (isDirectional) {
    // only one edge needs to go
    if (edgeExists(from, to)) {
      Edge *forwardEdge = vmap[from]->neighbors[edgeNeighborIndex(from, to)];
      vector<Edge *>::iterator forwardEdgeIndexInNeighborList =
          vmap[from]->neighbors.begin() + edgeNeighborIndex(from, to);
      vector<Edge *>::iterator forwardEdgeIndexInBigEdgeList =
          edges.begin() + edgeIndex(to);
      edges.erase(forwardEdgeIndexInBigEdgeList);
      vmap[from]->neighbors.erase(forwardEdgeIndexInNeighborList);
      delete forwardEdge;
      return true;
    }
    // edge doesn't exist
    return false;
  }
  if (edgeExists(from, to) || edgeExists(to, from)) {

    assert(edgeNeighborIndex(from, to) != -1);
    assert(edgeNeighborIndex(to, from) != -1);

    Edge *forwardEdge = vmap[from]->neighbors[edgeNeighborIndex(from, to)];
    Edge *backwardEdge = vmap[to]->neighbors[edgeNeighborIndex(to, from)];

    vector<Edge *>::iterator forwardEdgeIndexInNeighborList =
        vmap[from]->neighbors.begin() + edgeNeighborIndex(from, to);
    vector<Edge *>::iterator backwardEdgeIndexInNeighborList =
        vmap[to]->neighbors.begin() + edgeNeighborIndex(to, from);
    vector<Edge *>::iterator forwardEdgeIndexInBigEdgeList =
        edges.begin() + edgeIndex(to);
    vector<Edge *>::iterator backwardEdgeIndexInBigEdgeList =
        edges.begin() + edgeIndex(from);

    edges.erase(backwardEdgeIndexInBigEdgeList);
    edges.erase(forwardEdgeIndexInBigEdgeList);
    vmap[from]->neighbors.erase(forwardEdgeIndexInNeighborList);
    vmap[to]->neighbors.erase(backwardEdgeIndexInNeighborList);
    assert(edgeNeighborIndex(from, to) == -1);
    assert(edgeNeighborIndex(to, from) == -1);

    delete forwardEdge;
    delete backwardEdge;
    return true;
  }
  return false;
}

// depth-first traversal starting from given startLabel
void Graph::dfs(const string &startLabel, void visit(const string &label)) {
  unordered_set<Vertex *> visited;
  stack<Vertex *> stack;
  if (get(startLabel) == nullptr) {
    return;
  }
  stack.push(vmap[startLabel]);

  while (!stack.empty()) {
    stack = sortStack(stack);
    Vertex *current = stack.top();
    stack.pop();
    if (current == nullptr) {
      return;
      assert(1 == 0);
    }

    if (visited.count(current) == 0) {
      visit(current->label);
      visited.insert(current);
    }

    for (Edge *neighbor : current->neighbors) {
      if (visited.count(neighbor->to) == 0) {
        stack.push(neighbor->to);
      }
    }
  }
}

// sorts a stack
stack<Graph::Vertex *> Graph::sortStack(std::stack<Vertex *> inputStack) {
  std::stack<Vertex *> tempStack;

  while (!inputStack.empty()) {
    Vertex *temp = inputStack.top();
    inputStack.pop();

    while (!tempStack.empty() && tempStack.top()->label < temp->label) {
      inputStack.push(tempStack.top());
      tempStack.pop();
    }

    tempStack.push(temp);
  }

  return tempStack;
}

// breadth-first traversal starting from startLabel
void Graph::bfs(const string &startLabel, void visit(const string &label)) {
  unordered_set<Vertex *> visited;
  queue<Vertex *> queue;
  queue.push(vmap[startLabel]);
  visited.insert(vmap[startLabel]);

  while (!queue.empty()) {
    Vertex *current = queue.front();
    if (current == nullptr) {
      return;
      assert(1 == 0);
    }
    visit(current->label);
    queue.pop();

    vector<Edge *> tempV = current->neighbors;

    std::sort(tempV.begin(), tempV.end(), [](const Edge *a, const Edge *b) {
      return a->to->label > b->to->label;
    });

    for (int i = tempV.size() - 1; i >= 0; i--) {

      if (visited.count(tempV[i]->to) == 0) {
        visited.insert(tempV[i]->to);
        queue.push(tempV[i]->to);
      }
    }
  }
}

// gets the vertex that matches input
Graph::Vertex *Graph::get(string input) const {
  for (auto element : vmap) {
    if (element.first == input) {
      return element.second;
    }
  }
  return nullptr;
}

// store the weights in a map
// store the previous label in a map
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &startLabel) const {
  priority_queue<pair<int, Vertex *>, vector<pair<int, Vertex *>>,
                 greater<pair<int, Vertex *>>>
      pq;
  map<string, int> weights;
  map<string, string> edges;
  if (get(startLabel) == nullptr) {
    return make_pair(weights, edges);
  }
  pq.push({0, get(startLabel)});

  map<Vertex *, int> dist;

  for (auto v : vmap) {
    dist[v.second] = numeric_limits<int>::max();
  }
  dist[get(startLabel)] = 0;

  map<Vertex *, bool> visited;

  int totalDist = 0;
  while (!pq.empty()) {
    // auto [d, u] = pq.top();
    auto d = pq.top().first;
    auto *u = pq.top().second;
    pq.pop();
    totalDist = d;
    if (visited[u]) {
      continue;
    }
    visited[u] = true;

    for (auto &e : u->neighbors) {
      auto *v = e->to;
      auto w = e->weight;
      if (dist[u] + w < dist[v]) {
        dist[v] = dist[u] + w;
        pq.push({dist[v], v});
        weights[v->label] = w + totalDist;
        edges[v->label] = e->from->label;
      }
    }
  }

  return make_pair(weights, edges);
}

// minimum spanning tree using Prim's algorithm
int Graph::mstPrim(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const {
  if (get(startLabel) == nullptr) {
    return -1;
  }
  vector<Edge *> mst;
  int vectorsVisited = 1;
  vector<Edge *> adj;
  set<string> visited;
  adj = get(startLabel)->neighbors;
  visited.insert(startLabel);
  while (vmap.size() != vectorsVisited) { // until every vertex is visited
    auto *min = getMinEdge(adj);
    mst.push_back(min);
    visited.insert(min->to->label);
    vectorsVisited++;
    adj = adjToMst(mst, visited);
  }

  int minPath = 0;
  for (Edge *element : mst) {
    string from = element->from->label;
    string to = element->to->label;
    int weight = element->weight;
    visit(from, to, weight);
    minPath += element->weight;
  }
  return minPath;
}

// checks if the edge is in the vector
bool Graph::isEdgeInVector(const Edge *edgeToFind,
                           const std::vector<Edge *> &edgeVector) {
  for (const Edge *edge : edgeVector) {
    if (edge == edgeToFind) {
      return true;
    }
  }
  return false;
}

// returns a list of the Edges adjacent to the mst
vector<Graph::Edge *> Graph::adjToMst(vector<Edge *> mst,
                                      set<string> visited) const {
  vector<Edge *> adj;
  for (string element : visited) {
    for (Edge *edge : get(element)->neighbors) {
      if (visited.count(edge->to->label) == 0) {
        adj.push_back(edge);
      }
    }
  }

  for (int i = 0; i < adj.size(); i++) {
    if (isEdgeInVector(adj[1], mst)) {
      adj.erase(adj.begin() + i);
    }
  }
  return adj;
}

// returns the minimum edge adjacent to mst
Graph::Edge *Graph::getMinEdge(vector<Edge *> adjToMst) {
  int min = numeric_limits<int>::max();
  Edge *minKey;
  for (Edge *element : adjToMst) {
    if (element->weight < min) {
      min = element->weight;
      minKey = element;
    }
  }
  return minKey;
}

// read a text file and create the graph
bool Graph::readFile(const string &filename) {
  ifstream myfile(filename);
  // if (isDirectional == false)
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