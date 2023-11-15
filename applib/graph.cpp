#include "graph.h"
#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <utility>
#include <vector>

using namespace std;

// constructor, empty graph
// directionalEdges defaults to true
Graph::Graph(bool directionalEdges) { directional = directionalEdges; }

// destructor
Graph::~Graph() {}

// @return total number of vertices
int Graph::verticesSize() const { return vertices.size(); }

// @return total number of edges
int Graph::edgesSize() const {
  int edges = 0;
  for (auto vertex : vertices) {
    edges += edgeMap.at(vertex).size();
  }
  return edges;
}

// @return number of edges from given vertex, -1 if vertex not found
int Graph::vertexDegree(const string &label) const {
  if (edgeMap.count(label) == 0) {
    return -1;
  }
  return edgeMap.at(label).size();
}

// @return true if vertex added, false if it already is in the graph
bool Graph::add(const string &label) {
  if (contains(label)) {
    return false;
  }
  vertices.insert(label);
  edgeMap[label] = vector<Edge>();
  return true;
}

/** return true if vertex already in graph */
bool Graph::contains(const string &label) const {
  return vertices.find(label) != vertices.end();
}

// @return string representing edges and weights, "" if vertex not found
// A-3->B, A-5->C should return B(3),C(5)
string Graph::getEdgesAsString(const string &label) const {
  if (!contains(label)) {
    return "";
  }
  string edgeString;
  vector<Edge> edges(edgeMap.at(label));
  sort(edges.begin(), edges.end());
  for (auto edge : edges) {
    if (label == edge.end) {
      if (directional) {
        continue;
      }
      edgeString += edge.start;
    } else {
      edgeString += edge.end;
    }
    edgeString += "(" + to_string(edge.weight) + "),";
  }
  if (edgeString.length() > 0) {
    edgeString.erase(edgeString.length() - 1);
  }
  return edgeString;
}

// @return true if successfully connected
bool Graph::connect(const string &from, const string &to, int weight) {
  if (from == to) {
    return false;
  }
  add(from);
  add(to);
  for (auto edge : edgeMap[from]) {
    if (edge.end == to || (!directional && edge.start == to)) {
      return false;
    }
  }
  Edge newEdge(from, to, weight);
  edgeMap[from].push_back(newEdge);
  if (!directional) {
    Edge secondEdge(to, from, weight);
    edgeMap[to].push_back(secondEdge);
  }
  return true;
}

// @return true if edge successfully deleted
bool Graph::disconnect(const string &from, const string &to) {
  if (from == to || !contains(from) || !contains(to)) {
    return false;
  }
  bool removed = false;
  vector<Edge> toRemove;
  for (auto &edge : edgeMap[from]) {
    if (edge.end == to || edge.start == to) {
      toRemove.push_back(edge);
      removed = true;
    }
  }
  for (auto &edge : toRemove) {
    vector<Edge> &edgeMapFrom = edgeMap[from];
    edgeMapFrom.erase(remove(edgeMapFrom.begin(), edgeMapFrom.end(), edge),
                      edgeMapFrom.end());
  }
  toRemove.clear();
  for (auto &edge : edgeMap[to]) {
    if (edge.end == from) {
      toRemove.push_back(edge);
    }
  }
  for (auto &edge : toRemove) {
    vector<Edge> &edgeMapTo = edgeMap[to];
    edgeMapTo.erase(remove(edgeMapTo.begin(), edgeMapTo.end(), edge),
                    edgeMapTo.end());
  }
  return removed;
}

// depth-first traversal starting from given startLabel
void Graph::dfs(const string &startLabel, void visit(const string &label)) {
  if (!contains(startLabel)) {
    return;
  }
  stack<string> toVisit;
  unordered_map<string, bool> visited;
  visited[startLabel] = true;
  toVisit.push(startLabel);
  while (!toVisit.empty()) {
    string label = toVisit.top();
    toVisit.pop();
    visit(label);
    vector<Edge> sorted(edgeMap[label]);
    sort(sorted.begin(), sorted.end(),
         [](Edge a, Edge b) { return a.end > b.end; });
    for (auto edge : sorted) {
      string other = edge.end;
      if (!visited[other]) {
        visited[other] = true;
        toVisit.push(other);
      }
    }
  }
}

// breadth-first traversal starting from startLabel
void Graph::bfs(const string &startLabel, void visit(const string &label)) {
  if (!contains(startLabel)) {
    return;
  }
  queue<string> toVisit;
  unordered_map<string, bool> visited;
  visited[startLabel] = true;
  toVisit.push(startLabel);
  while (!toVisit.empty()) {
    string label = toVisit.front();
    toVisit.pop();
    visit(label);
    vector<Edge> sorted(edgeMap[label]);
    sort(sorted.begin(), sorted.end(),
         [](Edge a, Edge b) { return a.end < b.end; });
    for (auto edge : sorted) {
      string other = edge.end;
      if (!visited[other]) {
        visited[other] = true;
        toVisit.push(other);
      }
    }
  }
}

// used to sort pairs of vertices and weights
struct CompareBySecond {
  bool operator()(const std::pair<std::string, int> &a,
                  const std::pair<std::string, int> &b) const {
    return a.second > b.second;
  }
};

// store the weights in a map
// store the previous label in a map
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &startLabel) const {
  map<string, int> weights;
  map<string, string> previous;
  if (!contains(startLabel)) {
    return make_pair(weights, previous);
  }
  unordered_map<string, bool> visited;
  set<pair<string, int>, CompareBySecond> verticesToVisit;
  string nextVisit = startLabel;
  while (!nextVisit.empty()) {
    visited[nextVisit] = true;
    for (auto edge : edgeMap.at(nextVisit)) {
      if (weights.count(edge.end) == 0 ||
          weights[edge.end] > weights[nextVisit] + edge.weight) {
        weights[edge.end] = weights[nextVisit] + edge.weight;
        previous[edge.end] = nextVisit;
        verticesToVisit.insert(make_pair(edge.end, weights[edge.end]));
      }
    }
    while (!verticesToVisit.empty() &&
           visited[(*verticesToVisit.begin()).first]) {
      verticesToVisit.erase(verticesToVisit.begin());
    }
    if (verticesToVisit.empty()) {
      nextVisit = "";
    } else {
      nextVisit = (*verticesToVisit.begin()).first;
    }
  }
  weights.erase(startLabel);
  return make_pair(weights, previous);
}

// minimum spanning tree using Prim's algorithm
int Graph::mstPrim(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const {
  if (directional || !contains(startLabel)) {
    return -1;
  }
  unordered_map<string, bool> mst;
  mst[startLabel] = true;
  int totalWeight = 0;
  bool finished = false;
  while (!finished) {
    finished = true;
    int cheapestWeight = numeric_limits<int>::max();
    string cheapestTo;
    string cheapestFrom;
    for (auto vertex : mst) {
      for (auto edge : edgeMap.at(vertex.first)) {
        if (mst.count(edge.end) == 0 && edge.weight < cheapestWeight) {
          cheapestWeight = edge.weight;
          cheapestTo = edge.end;
          cheapestFrom = vertex.first;
        }
      }
    }
    if (!cheapestTo.empty()) {
      finished = false;
      mst[cheapestTo] = true;
      totalWeight += cheapestWeight;
      visit(cheapestFrom, cheapestTo, cheapestWeight);
    }
  }
  return totalWeight;
}

// minimum spanning tree using Kruskal's algorithm
int Graph::mstKruskal(void visit(const string &from, const string &to,
                                 int weight)) const {
  vector<Edge> allEdges;
  for (const auto &vertex : edgeMap) {
    for (const auto &edge : vertex.second) {
      allEdges.push_back(edge);
    }
  }
  sort(allEdges.begin(), allEdges.end(),
       [](const Edge &a, const Edge &b) { return a.weight < b.weight; });
  unordered_map<string, string> parent;
  int totalWeight = 0;
  function<string(const string &)> find = [&](const string &vertex) {
    if (parent.find(vertex) == parent.end()) {
      return vertex;
    }
    return find(parent[vertex]);
  };
  for (const Edge &edge : allEdges) {
    string startParent = find(edge.start);
    string endParent = find(edge.end);
    if (startParent != endParent) {
      parent[startParent] = endParent;
      totalWeight += edge.weight;
      visit(edge.start, edge.end, edge.weight);
    }
  }
  return totalWeight;
}

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