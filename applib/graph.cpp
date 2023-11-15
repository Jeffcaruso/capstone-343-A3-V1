#include "graph.h"
#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <utility>
#include <vector>

using namespace std;

// constructor, empty graph
// directionalEdges defaults to true
Graph::Graph(bool directionalEdges) { directionalEdges_ = directionalEdges; }

// destructor
Graph::~Graph() {}

// @return total number of vertices
int Graph::verticesSize() const { return vertices_.size(); }

// @return total number of edges
int Graph::edgesSize() const {
  int count = 0;
  for (const auto &vertex : vertices_) {
    count += vertex.second.edges.size();
  }
  if (!directionalEdges_) {
    count /= 2;
  }
  return count;
}

// @return number of edges from given vertex, -1 if vertex not found
int Graph::vertexDegree(const string &label) const {
  auto it = vertices_.find(label);
  if (it == vertices_.end()) {
    return -1;
  }
  int count = it->second.edges.size();
  if (!directionalEdges_) {
    for (const auto &vertex : vertices_) {
      if (vertex.first != label) {
        const auto &edges = vertex.second.edges;
        auto itEdge =
            find_if(edges.begin(), edges.end(),
                    [&](const Edge &edge) { return edge.to == label; });
        if (itEdge != edges.end()) {
          count++;
        }
      }
    }
    count /= 2;
  }
  return count;
}

// @return true if vertex added, false if it already is in the graph
bool Graph::add(const string &label) {
  if (contains(label)) {
    return false;
  }
  vertices_[label] = Vertex();
  return true;
}

/** return true if vertex already in graph */
bool Graph::contains(const string &label) const {
  return vertices_.find(label) != vertices_.end();
}

// @return string representing edges and weights, "" if vertex not found
// A-3->B, A-5->C should return B(3),C(5)
string Graph::getEdgesAsString(const string &label) const {
  auto it = vertices_.find(label);
  if (it == vertices_.end()) {
    return "";
  }
  string result;
  vector<Edge> sortedEdges = it->second.edges;
  sort(sortedEdges.begin(), sortedEdges.end(),
       [](const Edge &a, const Edge &b) { return a.to < b.to; });
  for (auto &edge : sortedEdges) {
    result += edge.to + "(" + to_string(edge.weight) + "),";
  }
  if (!result.empty()) {
    result.pop_back();
  }
  return result;
}

// @return true if successfully connected
bool Graph::connect(const string &from, const string &to, int weight) {
  if (from == to) {
    return false;
  }

  auto itFrom = vertices_.find(from);
  auto itTo = vertices_.find(to);
  if (itFrom == vertices_.end() || itTo == vertices_.end()) {
    return false;
  }
  auto &edges = itFrom->second.edges;
  auto itEdge = find_if(edges.begin(), edges.end(),
                        [&](const Edge &edge) { return edge.to == to; });
  if (itEdge != edges.end()) {
    return false;
  }
  edges.push_back(Edge(to, weight));
  if (!directionalEdges_) {
    itTo->second.edges.push_back(Edge(from, weight));
  }
  return true;
}

// disconnects two vertices, returns true if operations are successful
bool Graph::disconnect(const string &from, const string &to) {
  auto itFrom = vertices_.find(from);
  auto itTo = vertices_.find(to);
  if (itFrom == vertices_.end() || itTo == vertices_.end()) {
    return false;
  }
  auto &edges = itFrom->second.edges;
  auto itEdge = find_if(edges.begin(), edges.end(),
                        [&](const Edge &edge) { return edge.to == to; });
  if (itEdge == edges.end()) {
    return false;
  }
  edges.erase(itEdge);
  if (!directionalEdges_) {
    auto &edgesTo = itTo->second.edges;
    auto itEdgeTo = find_if(edgesTo.begin(), edgesTo.end(),
                            [&](const Edge &edge) { return edge.to == from; });
    if (itEdgeTo != edgesTo.end()) {
      return false;
    }
    edges.erase(itEdge);
  }
  return true;
}

// depth-first traversal starting from given startLabel
void Graph::dfs(const string &startLabel, void visit(const string &label)) {
  if (!contains(startLabel)) {
    return;
  }
  set<string> visited;
  dfsHelper(startLabel, visited, visit);
}

// Iterates over each edge of the given vertex and recursively calls itself for
// each connected vertex. Helper function called by dfs
void Graph::dfsHelper(const string &label, set<string> &visited,
                      void visit(const string &label)) {
  if (visited.count(label) > 0) {
    return;
  }
  visited.insert(label);
  visit(label);
  auto it = vertices_.find(label);
  if (it == vertices_.end()) {
    return;
  }
  for (auto &edge : it->second.edges) {
    dfsHelper(edge.to, visited, visit);
  }
}

// breadth-first traversal starting from startLabel
void Graph::bfs(const string &startLabel, void visit(const string &label)) {
  if (!contains(startLabel)) {
    return;
  }
  queue<string> q;
  set<string> visited;
  q.push(startLabel);
  visited.insert(startLabel);
  while (!q.empty()) {
    string label = q.front();
    q.pop();
    visit(label);
    auto it = vertices_.find(label);
    if (it == vertices_.end()) {
      continue;
    }
    for (auto &edge : it->second.edges) {
      if (visited.count(edge.to) == 0) {
        q.push(edge.to);
        visited.insert(edge.to);
      }
    }
  }
}

// store the weights in a map
// store the previous label in a map
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &startLabel) const {
  map<string, int> weights;
  map<string, string> previous;

  for (const auto &pair : vertices_) {
    weights[pair.first] = INT_MAX;
  }
  weights[startLabel] = 0;

  // Priority queue to store the vertices sorted by their distance
  using P = pair<int, string>;
  priority_queue<P, vector<P>, greater<P>> queue;
  queue.push({0, startLabel});

  while (!queue.empty()) {
    string current = queue.top().second;
    queue.pop();

    if (vertices_.find(current) != vertices_.end()) {
      for (const auto &edge : vertices_.at(current).edges) {
        int distance = weights[current] + edge.weight;
        if (distance < weights[edge.to]) {
          weights[edge.to] = distance;
          previous[edge.to] = current;
          queue.push({distance, edge.to});
        }
      }
    }
  }

  weights.erase(startLabel);
  for (auto it = weights.begin(); it != weights.end();) {
    if (it->second == INT_MAX) {
      it = weights.erase(it);
    } else {
      ++it;
    }
  }

  return make_pair(weights, previous);
}

// minimum spanning tree using Prim's algorithm
int Graph::mstPrim(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const {
  if (!contains(startLabel)) {
    return -1;
  }

  map<string, int> cost;
  for (const auto &pair : vertices_) {
    cost[pair.first] = INT_MAX;
  }

  cost[startLabel] = 0;

  map<string, string> prev;

  set<string> vertices;
  for (const auto &pair : vertices_) {
    vertices.insert(pair.first);
  }

  int totalWeight = 0;

  while (!vertices.empty()) {
    string vertex;
    int vertexCost = INT_MAX;
    for (const string &v : vertices) {
      if (cost[v] < vertexCost) {
        vertexCost = cost[v];
        vertex = v;
      }
    }

    vertices.erase(vertex);

    if (prev.count(vertex) > 0) {
      visit(prev[vertex], vertex, cost[vertex]);
      totalWeight += cost[vertex];
    }

    for (const Edge &edge : vertices_.at(vertex).edges) {
      if (vertices.count(edge.to) > 0 && edge.weight < cost[edge.to]) {
        cost[edge.to] = edge.weight;
        prev[edge.to] = vertex;
      }
    }
  }

  return totalWeight;
}

// minimum spanning tree using Kruskal's algorithm
int Graph::mstKruskal(void visit(const string &from, const string &to,
                                 int weight)) const {
  // Priority queue to store the edges sorted by their weight
  using EdgeTuple = tuple<int, string, string>;
  priority_queue<EdgeTuple, vector<EdgeTuple>, greater<EdgeTuple>> queue;

  for (const auto &pair : vertices_) {
    const string &from = pair.first;
    for (const Edge &edge : pair.second.edges) {
      if (from < edge.to) {
        queue.push(make_tuple(edge.weight, from, edge.to));
      }
    }
  }

  map<string, set<string>> components;
  for (const auto &pair : vertices_) {
    components[pair.first] = {pair.first};
  }

  int totalWeight = 0;

  while (!queue.empty()) {
    EdgeTuple edgeTuple = queue.top();
    queue.pop();

    int weight = get<0>(edgeTuple);
    const string &from = get<1>(edgeTuple);
    const string &to = get<2>(edgeTuple);

    set<string> *fromComponent = nullptr;
    set<string> *toComponent = nullptr;
    for (auto &pair : components) {
      if (pair.second.count(from) > 0) {
        fromComponent = &pair.second;
      }
      if (pair.second.count(to) > 0) {
        toComponent = &pair.second;
      }
    }

    if (fromComponent != toComponent) {
      visit(from, to, weight);
      totalWeight += weight;
      fromComponent->insert(toComponent->begin(), toComponent->end());
      components.erase(to);
    }
  }

  return totalWeight;
}

// read a text file and create the graph
bool Graph::readFile(const string &filename) {
  ifstream file(filename);
  if (!file) {
    return false;
  }
  int numEdges;
  if (!(file >> numEdges)) {
    return false;
  }
  for (int i = 0; i < numEdges; ++i) {
    string from;
    string to;
    int weight;
    if (!(file >> from >> to >> weight)) {
      return false;
    }
    add(from);
    add(to);
    connect(from, to, weight);
  }
  for (auto &pair : vertices_) {
    sort(pair.second.edges.begin(), pair.second.edges.end(),
         [](const Edge &a, const Edge &b) { return a.to < b.to; });
  }
  return true;
}