#include "graph.h"
#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <utility>
#include <vector>

using namespace std;

// constructor, empty graph
// directionalEdges defaults to true
Graph::Graph(bool directionalEdges) { directed = directionalEdges; }

// destructor
Graph::~Graph() {
  for (auto const &curr : vertices) {
    delete curr.second;
  }
  vertices.clear();
}

// @return total number of vertices
int Graph::verticesSize() const { return vertices.size(); }

// @return total number of edges
int Graph::edgesSize() const {
  map<Vertex *, set<Vertex *>> seen;
  int total = 0;
  for (auto const &i : vertices) {
    if (directed) {
      total += i.second->neighbors.size();
    } else {
      for (auto const &j : i.second->neighbors) {
        if (seen.count(i.second) == 0 || seen[i.second].count(j.first) == 0) {
          seen[i.second].insert(j.first);
          seen[j.first].insert(i.second);
          total++;
        }
      }
    }
  }
  return total;
}

// @return number of edges from given vertex, -1 if vertex not found
int Graph::vertexDegree(const string &label) const {
  return vertices.count(label) == 1 ? vertices.at(label)->neighbors.size() : -1;
}

// @return true if vertex added, false if it already is in the graph
bool Graph::add(const string &label) {
  if (vertices.count(label) == 1) {
    return false;
  }
  vertices[label] = new Vertex{label, {}};
  return true;
}

/** return true if vertex already in graph */
bool Graph::contains(const string &label) const {
  return vertices.count(label) == 1;
}

// @return string representing edges and weights, "" if vertex not found
// A-3->B, A-5->C should return B(3),C(5)
string Graph::getEdgesAsString(const string &label) const {
  string rtn;
  if (vertices.count(label) == 0 || vertices.at(label)->neighbors.empty()) {
    return rtn;
  }
  // compare pairs of vertices with corresponding weights
  auto compVertex = [](pair<Vertex *, int> a, pair<Vertex *, int> b) {
    return a.first->val < b.first->val;
  };
  set<pair<Vertex *, int>, decltype(compVertex)> neighbors(
      vertices.at(label)->neighbors.begin(),
      vertices.at(label)->neighbors.end(), compVertex);
  // the sorted value is printed
  for (const auto &i : neighbors) {
    rtn += i.first->val + "(" + to_string(i.second) + "),";
  }
  rtn.pop_back();
  return rtn;
}

// @return true if successfully connected
bool Graph::connect(const string &from, const string &to, int weight) {
  if (from == to) {
    return false;
  }
  if (vertices.count(from) == 0) {
    vertices[from] = new Vertex{from, {}};
  }
  if (vertices.count(to) == 0) {
    vertices[to] = new Vertex{to, {}};
  }
  if (vertices[from]->neighbors.count(vertices[to]) == 1) {
    return false;
  }
  vertices[from]->neighbors[vertices[to]] = weight;
  if (!directed) {
    vertices[to]->neighbors[vertices[from]] = weight;
  }
  return true;
}

// erases the connection between two vertex's
bool Graph::disconnect(const string &from, const string &to) {
  if (vertices.count(from) == 0 || vertices.count(to) == 0) {
    return false;
  }
  if (vertices[from]->neighbors.count(vertices[to]) == 0) {
    return false;
  }
  vertices[from]->neighbors.erase(vertices[to]);
  if (!directed) {
    vertices[to]->neighbors.erase(vertices[from]);
  }
  return true;
}

// depth-first traversal starting from given startLabel
void Graph::dfs(const string &startLabel, void visit(const string &label)) {
  if (vertices.count(startLabel) == 1) {
    set<Vertex *> seen;
    seen.insert(vertices[startLabel]);
    dfsHelper(seen, vertices[startLabel], visit);
  }
}

// breadth-first traversal starting from startLabel
void Graph::bfs(const string &startLabel, void visit(const string &label)) {
  set<Vertex *> seen;
  queue<Vertex *> q;
  q.push(vertices[startLabel]);
  seen.insert(vertices[startLabel]);
  Vertex *curr;
  auto comp = [](pair<Vertex *, int> a, pair<Vertex *, int> b) {
    return a.first->val < b.first->val;
  };
  while (!q.empty()) {
    curr = q.front();
    q.pop();
    visit(curr->val);
    set<pair<Vertex *, int>, decltype(comp)> neighbors(
        curr->neighbors.begin(), curr->neighbors.end(), comp);
    for (auto const &i : neighbors) {
      if (seen.count(i.first) == 0) {
        q.push(i.first);
        seen.insert(i.first);
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
  // TODO(student) Your code here
  if (vertices.count(startLabel) == 0) {
    return make_pair(weights, previous);
  }
  set<string> seen;
  queue<Edge> edges;
  seen.insert(startLabel);
  for (auto const &j : vertices.at(startLabel)->neighbors) {
    edges.push({vertices.at(startLabel), j.second, j.first});
  }
  int prevWeight;
  int toWeight;
  while (!edges.empty()) {
    Edge edge = edges.front();
    edges.pop();
    prevWeight =
        weights.count(edge.end->val) == 1 ? weights[edge.end->val] : INT_MAX;
    toWeight = weights.count(edge.source->val) == 1
                   ? edge.weight + weights[edge.source->val]
                   : edge.weight;
    if (prevWeight > toWeight) {
      weights[edge.end->val] = toWeight;
      previous[edge.end->val] = edge.source->val;
    }
    if (seen.count(edge.end->val) == 0) {
      seen.insert(edge.end->val);
      for (auto const &i : edge.end->neighbors) {
        if (i.first->val != startLabel) {
          edges.push({edge.end, i.second, i.first});
        }
      }
    }
  }
  return make_pair(weights, previous);
}

// minimum spanning tree using Prim's algorithm
int Graph::mstPrim(const string &startLabel,
                   void visit(const string &source, const string &end,
                              int weight)) const {
  if (directed || vertices.empty() || vertices.count(startLabel) == 0) {
    return -1;
  }
  set<Vertex *> seen;
  auto compEdge = [](Edge a, Edge b) { return a.weight > b.weight; };
  // making pq storing edges
  priority_queue<Edge, vector<Edge>, decltype(compEdge)> edges(compEdge);
  // determine all edges connected to source vertex and add to pq
  seen.insert(vertices.at(startLabel));
  for (auto const &i : vertices.at(startLabel)->neighbors) {
    edges.push({vertices.at(startLabel), i.second, i.first});
  }
  int weight = 0;
  while (!edges.empty()) {
    // edge with lowest weight is chosen and added to pq
    Edge curr = edges.top();
    edges.pop();
    if (seen.count(curr.end) == 0) {
      weight += curr.weight;
      seen.insert(curr.end);
      visit(curr.source->val, curr.end->val, curr.weight);
      // determine all edges from end node of lowest weight edge chosen
      // previously and add to pq
      for (auto const &i : curr.end->neighbors) {
        if (seen.count(i.first) == 0) {
          edges.push({curr.end, i.second, i.first});
        }
      }
    }
  }
  return weight;
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
  string sourceVertex;
  string endVertex;
  myfile >> edges;
  for (int i = 0; i < edges; ++i) {
    myfile >> sourceVertex >> endVertex >> weight;
    connect(sourceVertex, endVertex, weight);
  }
  myfile.close();
  return true;
}

// Helper for dfs that recursively explores the graph
void Graph::dfsHelper(set<Vertex *> &seen, Vertex *curr,
                      void visit(const string &label)) {
  visit(curr->val);
  auto comp = [](pair<Vertex *, int> a, pair<Vertex *, int> b) {
    return a.first->val < b.first->val;
  };
  set<pair<Vertex *, int>, decltype(comp)> neighbors(
      curr->neighbors.begin(), curr->neighbors.end(), comp);
  for (auto const &i : neighbors) {
    if (seen.count(i.first) == 0) {
      seen.insert(i.first);
      dfsHelper(seen, i.first, visit);
    }
  }
}

// Determines if there is a path from one vertex to another and vice versa
// dfs is used to check if any vertex on the path from the source edge
// on the current mst is connected to ending edge. return true if the case
bool Graph::hasCycle(const map<Vertex *, set<Vertex *>> &origin, Edge edge) {
  stack<Vertex *> stk;
  set<Vertex *> cycSeen;
  cycSeen.insert(edge.source);
  if (origin.count(edge.source) == 1) {
    for (auto const &i : edge.source->neighbors) {
      if (origin.at(edge.source).count(i.first) == 1) {
        stk.push(i.first);
      }
    }
  }
  while (!stk.empty()) {
    Vertex *curr = stk.top();
    stk.pop();
    if (origin.at(curr).count(edge.end) == 1) {
      return true;
    }
    for (auto const &i : curr->neighbors) {
      if (origin.at(curr).count(i.first) == 1 && cycSeen.count(i.first) == 0) {
        stk.push(i.first);
        cycSeen.insert(i.first);
      }
    }
  }
  return false;
}
