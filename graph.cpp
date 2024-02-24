/**
 * Graph data structure
 *
 * @author Kyler Tran
 * @date 29 Oct 2023
 */

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
Graph::Graph(bool directionalEdges) {
  this->directionalEdges = directionalEdges;
}

// destructor
Graph::~Graph() {}

// @return total number of vertices
int Graph::verticesSize() const { return vertexMap.size(); }

// @return total number of edges
int Graph::edgesSize() const { return edgeList.size(); }

// @return number of edges from given vertex, -1 if vertex not found
int Graph::neighborsSize(const string &label) const {
  if (!contains(label)) {
    return -1;
  }
  int count = 0;
  for (Edge edge : edgeList) {
    if (edge.fromVertex == label) {
      count++;
    }
  }
  return count;
}

// @return true if vertex added, false if it already is in the graph
bool Graph::add(const string &label) {
  if (contains(label)) {
    return false;
  }
  vertexMap[label] = Vertex{label, {}, {}};
  return true;
}

/** return true if vertex already in graph */
bool Graph::contains(const string &label) const {
  return vertexMap.find(label) != vertexMap.end();
}

// @return string representing edges and weights, "" if vertex not found
// A-3->B, A-5->C should return B(3),C(5)
string Graph::getEdgesAsString(const string &label) const {
  vector<Graph::Edge> relevantEdges;

  // Gather all edges originating from the given vertex
  for (const Graph::Edge &edge : edgeList) {
    if (edge.fromVertex == label) {
      relevantEdges.push_back(edge);
    }
  }

  // Sort the edges based on weight
  sort(relevantEdges.begin(), relevantEdges.end(),
       [](const Graph::Edge &a, const Graph::Edge &b) {
         return a.weight < b.weight;
       });

  string result;
  for (const Graph::Edge &edge : relevantEdges) {
    if (!result.empty()) {
      result += ",";
    }
    result += edge.toVertex + "(" + to_string(edge.weight) + ")";
  }
  return result;
}

// @return true if successfully connected
// creates an edge between two vertices, create new vertices if necessary
bool Graph::connect(const string &from, const string &to, int weight) {
  if (from == to) {
    return false;
  }

  // Check if the edge already exists in edgeList
  for (const Edge &edge : edgeList) {
    if (edge.fromVertex == from && edge.toVertex == to) {
      return false; // Edge already exists
    }
  }

  Edge newEdge = Edge{from, to, weight};
  edgeList.push_back(newEdge);
  vertexMap[from].outgoingEdges.push_back(newEdge);
  vertexMap[to].incomingEdges.push_back(newEdge);

  // If graph is undirected, add the reverse edge as well
  if (!directionalEdges) {
    Edge reverseEdge = Edge{to, from, weight};
    edgeList.push_back(reverseEdge);
    vertexMap[to].outgoingEdges.push_back(reverseEdge);
    vertexMap[from].incomingEdges.push_back(reverseEdge);
  }

  return true;
}

// @return true if edge successfully deleted
// remove edge from graph
bool Graph::disconnect(const string &from, const string &to) {
  for (int i = 0; i < edgeList.size(); i++) {
    if (edgeList[i].fromVertex == from && edgeList[i].toVertex == to) {
      edgeList.erase(edgeList.begin() + i);

      // Also remove from the vertex's incoming and outgoing edges list
      auto &fromVertex = vertexMap[from];
      fromVertex.outgoingEdges.erase(
          remove_if(fromVertex.outgoingEdges.begin(),
                    fromVertex.outgoingEdges.end(),
                    [&](const Edge &e) { return e.toVertex == to; }),
          fromVertex.outgoingEdges.end());

      auto &toVertex = vertexMap[to];
      toVertex.incomingEdges.erase(
          remove_if(toVertex.incomingEdges.begin(),
                    toVertex.incomingEdges.end(),
                    [&](const Edge &e) { return e.fromVertex == from; }),
          toVertex.incomingEdges.end());

      return true;
    }
  }
  return false;
}

// depth-first traversal starting from given startLabel
void Graph::dfs(const string &startLabel, void visit(const string &label)) {
  if (!contains(startLabel)) {
    return;
  }

  stack<string> s;
  set<string> visited;

  s.push(startLabel);

  while (!s.empty()) {
    string current = s.top();
    s.pop();

    // If not visited yet
    if (visited.find(current) == visited.end()) {
      visited.insert(current);
      visit(current);

      Vertex &currentVertex = vertexMap[current];

      // Create a vector of adjacent vertices
      vector<string> adjacentVertices;

      // Push adjacent vertices to the vector using outgoingEdges
      for (const Edge &edge : currentVertex.outgoingEdges) {
        if (visited.find(edge.toVertex) == visited.end()) {
          adjacentVertices.push_back(edge.toVertex);
        }
      }

      // For non-directional graphs, also consider incomingEdges
      if (!directionalEdges) {
        for (const Edge &edge : currentVertex.incomingEdges) {
          if (visited.find(edge.fromVertex) == visited.end()) {
            adjacentVertices.push_back(edge.fromVertex);
          }
        }
      }

      // Sort adjacent vertices in reverse alphabetical order
      sort(adjacentVertices.rbegin(), adjacentVertices.rend());

      // Push the sorted adjacent vertices to the stack
      for (const string &vertex : adjacentVertices) {
        s.push(vertex);
      }
    }
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

  while (!q.empty()) {
    string current = q.front();
    q.pop();

    // If not visited yet
    if (visited.find(current) == visited.end()) {
      visited.insert(current);
      visit(current);

      // Collect adjacent vertices
      vector<Edge> adjacentEdges;
      for (const Edge &edge : edgeList) {
        if (edge.fromVertex == current &&
            visited.find(edge.toVertex) == visited.end()) {
          adjacentEdges.push_back(edge);
        }
      }

      // Sort adjacent edges by weight
      sort(adjacentEdges.begin(), adjacentEdges.end(),
           [](const Edge &a, const Edge &b) { return a.weight < b.weight; });

      // Enqueue adjacent vertices in order of increasing edge weight
      for (const Edge &edge : adjacentEdges) {
        q.push(edge.toVertex);
      }
    }
  }
}

// dijkstra's algorithm to find shortest distance to all other vertices
// @return total weight, -1 if start vertex not found
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &startLabel) const {
  map<string, int> distances;
  map<string, string> previous;

  // Using a priority queue to efficiently get the vertex with the minimum
  // distance
  priority_queue<pair<int, string>, vector<pair<int, string>>,
                 greater<pair<int, string>>>
      pq;

  for (const auto &pair : vertexMap) {
    const string &vertex = pair.first;
    distances[vertex] = INT_MAX;
    previous[vertex] = "";
    pq.push({INT_MAX, vertex});
  }

  distances[startLabel] = 0;
  pq.push({0, startLabel});

  while (!pq.empty()) {
    string current = pq.top().second;
    int currentDist = pq.top().first;
    pq.pop();

    if (vertexMap.find(current) == vertexMap.end()) {
      continue; // Skip if vertex not found
    }

    if (currentDist != distances[current]) {
      continue; // Skip old distance values
    }

    // Relaxing edges
    for (const Edge &edge : vertexMap.at(current).outgoingEdges) {
      if (distances[current] != INT_MAX) {
        string nextVertex = edge.toVertex;
        int newDist = distances[current] + edge.weight;
        if (newDist < distances[nextVertex]) {
          distances[nextVertex] = newDist;
          previous[nextVertex] = current;
          pq.push({newDist, nextVertex});
        }
      }
    }
  }

  distances.erase(startLabel);
  previous.erase(startLabel);

  set<string> verticesToRemove;

  for (const auto &pair : distances) {
    if (pair.second == INT_MAX) {
      verticesToRemove.insert(pair.first);
    }
  }

  for (const string &vertex : verticesToRemove) {
    distances.erase(vertex);
    previous.erase(vertex);
  }

  return make_pair(distances, previous);
}

// runs mstPrim algorithm on graph and calls given visit function on each edge
// @return length of the minimum spanning tree or -1 if start vertex not
//mst prim
int Graph::mst(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const {
  if (!contains(startLabel)) {
    return -1;
  }

  map<string, int>
      key; // Stores weights used to pick minimum weight edge in cut
  map<string, string> parent; // Stores the resultant MST
  set<string> mstSet; // To represent set of vertices not yet included in MST

  // Initialize all keys as INFINITE and mstSet as empty using vertexMap
  for (const auto &entry : vertexMap) {
    const string &vertex = entry.first;
    key[vertex] = INT_MAX;
    mstSet.insert(vertex);
  }

  key[startLabel] = 0;       // Make key 0 so that vertex is picked first
  parent[startLabel] = "-1"; // First node has no parent

  int mstWeight = 0; // Total weight of MST

  while (!mstSet.empty()) {
    // Pick the minimum key vertex from the set of vertices not yet included in
    // MST
    string u = *min_element(
        mstSet.begin(), mstSet.end(),
        [&](const string &a, const string &b) { return key[a] < key[b]; });
    mstSet.erase(u); // Add the picked vertex to the MST set

    // If the parent of this vertex isn't "-1", it means it has a connecting
    // edge, add its weight
    if (parent[u] != "-1") {
      mstWeight += key[u];
    }

    // Update key values of adjacent vertices of the picked vertex
    for (const Edge &edge : vertexMap.at(u).outgoingEdges) {
      string v = edge.toVertex;
      if (mstSet.find(v) != mstSet.end() && edge.weight < key[v]) {
        parent[v] = u;
        key[v] = edge.weight;
      }
    }
  }

  // Call the visit function for the constructed MST using vertexMap
  for (const auto &entry : vertexMap) {
    const string &vertex = entry.first;
    if (parent[vertex] != "-1") {
      visit(parent[vertex], vertex, key[vertex]);
    }
  }

  return mstWeight;
}

// helper function for mstKruskal
// creates a set for a vertex
void Graph::makeSet(UnionFind &uf, const string &vertex) {
  uf.parent[vertex] = vertex;
  uf.rank[vertex] = 0;
}

// helper function for mstKruskal
// finds the set a vertex belongs to
string Graph::find(UnionFind &uf, const string &vertex) {
  if (uf.parent[vertex] != vertex) {
    uf.parent[vertex] = find(uf, uf.parent[vertex]);
  }
  return uf.parent[vertex];
}

// helper function for mstKruskal
// combines two sets into one
bool Graph::unionSets(UnionFind &uf, const string &vertex1,
                      const string &vertex2) {
  string root1 = find(uf, vertex1);
  string root2 = find(uf, vertex2);

  if (root1 == root2) {
    return false; // They are already in the same set.
  }

  if (uf.rank[root1] < uf.rank[root2]) {
    uf.parent[root1] = root2;
  } else if (uf.rank[root1] > uf.rank[root2]) {
    uf.parent[root2] = root1;
  } else {
    uf.parent[root2] = root1;
    uf.rank[root1]++;
  }
  return true;
}

// @return total weight of MST, -1 if graph is disconnected
// calls visit on each edge in the MST in the order they were added to the MST
int Graph::mstKruskal(void visit(const string &from, const string &to,
                                 int weight)) {
  vector<Edge> sortedEdges = edgeList;
  sort(sortedEdges.begin(), sortedEdges.end(),
       [](const Edge &a, const Edge &b) { return a.weight < b.weight; });

  UnionFind uf;
  for (const auto &pair : vertexMap) {
    makeSet(uf, pair.first);
  }

  int mstWeight = 0;
  for (const Edge &edge : sortedEdges) {
    if (unionSets(uf, edge.fromVertex, edge.toVertex)) {
      mstWeight += edge.weight;
      visit(edge.fromVertex, edge.toVertex, edge.weight);
    }
  }

  return mstWeight;
}

// read edges from file
// first line of file is an integer indicating number of edges
// calls connect function for every edge in file with weight
bool Graph::readFile(const string &filename) {
  ifstream myfile(filename);
  if (!myfile.is_open()) {
    cerr << "Failed to open " << filename << endl;
    return false;
  }

  int weight = 0;
  string fromVertex;
  string toVertex;

  string firstLine;
  getline(myfile, firstLine);

  stringstream ss(firstLine);
  int edges;
  if (ss >> edges) {
    // if the first line is a number, read that many lines
    for (int i = 0; i < edges; ++i) {
      myfile >> fromVertex >> toVertex >> weight;
      connect(fromVertex, toVertex, weight);
    }
  } else {
    // if the first line isn't a number, read the entire file
    ss = stringstream(firstLine);
    ss >> fromVertex >> toVertex >> weight;
    connect(fromVertex, toVertex, weight);
    while (myfile >> fromVertex >> toVertex >> weight) {
      connect(fromVertex, toVertex, weight);
    }
  }

  myfile.close();
  return true;
}
