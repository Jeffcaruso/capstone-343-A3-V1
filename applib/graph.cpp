#include "graph.h"
#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <stack>
#include <utility>
#include <vector>

// constructor, empty graph
// directionalEdges defaults to true
Graph::Graph(bool directionalEdges) {
  _vertex_count = 0;
  _isDirectional = directionalEdges;
}

// destructor
Graph::~Graph() {
  // Destructor to delete all vertices and edges
  for (vector<int> &row : _adjacency_list) {
    row.clear();
  }
  _adjacency_list.clear();
  _vertices.clear();
  _vertex_count = 0;
}

// @return total number of vertices
int Graph::verticesSize() const { return _vertex_count; }

// @return total number of edges
int Graph::edgesSize() const {
  int totalEdges = 0;

  for (int i = 0; i < _vertex_count; ++i) {
    for (int j = 0; j < _vertex_count; ++j) {
      totalEdges += _adjacency_list[i][j] != 0 ? 1 : 0;
    }
  }

  if (!_isDirectional) {
    // For undirected graphs, divide by 2 to avoid counting edges twice
    totalEdges /= 2;
  }

  return totalEdges;
}

// @return number of edges from given vertex, -1 if vertex not found
int Graph::vertexDegree(const string &label) const {
  if (!contains(label)) {
    // Vertex not found
    return -1;
  }

  int vertexIndex =
      find(_vertices.begin(), _vertices.end(), label) - _vertices.begin();
  int degree = 0;

  for (int i = 0; i < _vertex_count; ++i) {
    degree += _adjacency_list[vertexIndex][i] != 0 ? 1 : 0;
  }

  return degree;
}

// @return true if vertex added, false if it already is in the graph
bool Graph::add(const string &label) {
  if (contains(label)) {
    // Vertex already exists in the graph
    return false;
  }

  _vertices.push_back(label);
  _adjacency_list.push_back(vector<int>(_vertex_count, 0));

  for (vector<int> &row : _adjacency_list) {
    row.push_back(0);
  }

  ++_vertex_count;
  return true;
}

/** return true if vertex already in graph */
bool Graph::contains(const string &label) const {
  return find(_vertices.begin(), _vertices.end(), label) != _vertices.end();
}

// @return string representing edges and weights, "" if vertex not found
// A-3->B, A-5->C should return B(3),C(5)
string Graph::getEdgesAsString(const string &label) const {
  if (!contains(label)) {
    // Vertex not found
    return "";
  }

  int vertexIndex =
      find(_vertices.begin(), _vertices.end(), label) - _vertices.begin();
  string edgesString;

  // Create vertex list
  vector<pair<string, int>> vertex_list;
  for (int i = 0; i < _vertex_count; ++i) {
    int weight = _adjacency_list[vertexIndex][i];
    if (weight != 0) {
      pair<string, int> neighbor;
      neighbor.first = _vertices[i];
      neighbor.second = weight;
      vertex_list.push_back(neighbor);
    }
  }

  // Sort the vector based on the strings
  sort(vertex_list.begin(), vertex_list.end(),
       [](const pair<string, int> &lhs, const pair<string, int> &rhs) {
         return lhs.first < rhs.first;
       });

  // Save the sorted vector for output
  for (const pair<string, int> &pair : vertex_list) {
    if (!edgesString.empty()) {
      edgesString += ",";
    }
    edgesString += pair.first + "(" + to_string(pair.second) + ")";
  }

  return edgesString;
}

// @return true if successfully connected
bool Graph::connect(const string &from, const string &to, int weight) {
  if (from == to) {
    // Trying to connect a vertex to itself
    return false;
  }

  if (!contains(from)) {
    // add missing vertex
    add(from);
  }

  if (!contains(to)) {
    // add missing vertex
    add(to);
  }

  int fromIndex =
      find(_vertices.begin(), _vertices.end(), from) - _vertices.begin();
  int toIndex =
      find(_vertices.begin(), _vertices.end(), to) - _vertices.begin();

  if (!_adjacency_list[fromIndex][toIndex]) {
    // Update the weight in the adjacency matrix
    _adjacency_list[fromIndex][toIndex] = weight;

    if (!_isDirectional) {
      // For undirected graphs, update the reverse direction as well
      _adjacency_list[toIndex][fromIndex] = weight;
    }
  } else {
    // Edge already exists
    return false;
  }

  return true;
}

bool Graph::disconnect(const string &from, const string &to) {
  if (!contains(from) || !contains(to)) {
    // One or both vertices not found
    return false;
  }

  int fromIndex =
      find(_vertices.begin(), _vertices.end(), from) - _vertices.begin();
  int toIndex =
      find(_vertices.begin(), _vertices.end(), to) - _vertices.begin();

  // Check if the edge exists
  if (_adjacency_list[fromIndex][toIndex]) {
    // Remove the edge by setting the weight to 0
    _adjacency_list[fromIndex][toIndex] = 0;

    if (!_isDirectional) {
      // For undirected graphs, remove the reverse direction as well
      _adjacency_list[toIndex][fromIndex] = 0;
    }
  } else {
    // return false because the edge did not exist.
    return false;
  }

  return true;
}

#if NEW_DFS
void Graph::dfs(const string &startLabel, void visit(const string &label)) {
    if (!contains(startLabel)) {
        // Start vertex not found
        return;
    }

    int startIndex = find(_vertices.begin(), _vertices.end(), startLabel) - _vertices.begin();
    vector<bool> visited(_vertex_count, false);
    stack<int> s;

    while (true) {
        s.push(startIndex);

        while (!s.empty()) {
            int currentVertex = s.top();
            s.pop();

            if (!visited[currentVertex]) {
                visit(_vertices[currentVertex]);
                visited[currentVertex] = true;

                // Push neighbors onto the stack
                for (int neighbor = 0; neighbor < _vertex_count; ++neighbor) {
                    if (_adjacency_list[currentVertex][neighbor] != 0 && !visited[neighbor]) {
                        s.push(neighbor);
                    }
                }
            }
        }
#if DISJOINTED_GRAPHS
        // Find the next unvisited vertex (for disconnected graphs)
        startIndex = -1;
        for (int i = 0; i < _vertex_count; ++i) {
            if (!visited[i]) {
                startIndex = i;
                break;
            }
        }

        if (startIndex == -1) {
            // All vertices visited
            break;
        }
#endif
      break;
    }
}

#else 
// depth-first traversal starting from given startLabel
void Graph::dfs(const string &startLabel, void visit(const string &label)) {
  if (!contains(startLabel)) {
    // Start vertex not found
    return;
  }

  int startIndex =
      find(_vertices.begin(), _vertices.end(), startLabel) - _vertices.begin();
  vector<bool> visited(_vertex_count, false);
  stack<int> s;
  if (_isDirectional) {
    if (!contains(startLabel)) {
      // Start vertex not found
      return;
    }

    int startIndex = find(_vertices.begin(), _vertices.end(), startLabel) -
                     _vertices.begin();
    vector<bool> visited(_vertex_count, false);
    stack<int> s;

    s.push(startIndex);

    while (!s.empty()) {
      int currentVertex = s.top();
      s.pop();

      if (!visited[currentVertex]) {
        visited[currentVertex] = true;
        visit(_vertices[currentVertex]);

        // Push neighbors onto the stack
        for (int neighbor = 0; neighbor < _vertex_count; ++neighbor) {
          if (_adjacency_list[currentVertex][neighbor] != 0 &&
              !visited[neighbor]) {
            s.push(neighbor);
          }
        }
      }
    }
  } else {
    s.push(startIndex);

    while (!s.empty()) {
      int currentVertex = s.top();
      s.pop();

      if (!visited[currentVertex]) {
        visit(_vertices[currentVertex]);
        visited[currentVertex] = true;
      }

      // Iterate through neighbors in reverse order to prioritize lower indices
      for (int neighbor = _vertex_count - 1; neighbor >= 0; --neighbor) {
        if (_adjacency_list[currentVertex][neighbor] != 0 &&
            !visited[neighbor]) {
          s.push(neighbor);
        }
      }
    }
  }
}
#endif

// breadth-first traversal starting from startLabel
void Graph::bfs(const string &startLabel, void visit(const string &label)) {
  if (!contains(startLabel)) {
    // Start vertex not found
    return;
  }

  int startIndex =
      find(_vertices.begin(), _vertices.end(), startLabel) - _vertices.begin();
  vector<bool> visited(_vertex_count, false);
  queue<int> q;

  q.push(startIndex);
  visited[startIndex] = true;

  while (!q.empty()) {
    int currentVertex = q.front();
    q.pop();

    visit(_vertices[currentVertex]);

    // Sort neighbors before enqueuing
    vector<pair<int, int>> neighbors;
    for (int neighbor = 0; neighbor < _vertex_count; ++neighbor) {
      if (_adjacency_list[currentVertex][neighbor] != 0 && !visited[neighbor]) {
        neighbors.push_back(
            {_adjacency_list[currentVertex][neighbor], neighbor});
      }
    }
    sort(neighbors.begin(), neighbors.end());

    for (const auto &neighbor : neighbors) {
      q.push(neighbor.second);
      visited[neighbor.second] = true;
    }
  }
}
// store the weights in a map
// store the previous label in a map
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &startLabel) const {
  map<string, int> weights;
  map<string, string> previous;

  if (!contains(startLabel)) {
    // Start vertex not found
    return make_pair(map<string, int>(), map<string, string>());
  }

  int startIndex =
      find(_vertices.begin(), _vertices.end(), startLabel) - _vertices.begin();

  // Priority queue to store vertices with their current distances
  priority_queue<pair<int, int>, vector<pair<int, int>>,
                 greater<pair<int, int>>>
      pq;

  // Initialize distances and add the start vertex to the priority queue
  for (const string &vertex : _vertices) {
    weights[vertex] = numeric_limits<int>::max();
    previous[vertex] = "";
  }

  weights[startLabel] = 0;
  pq.push({0, startIndex});

  // Dijkstra's algorithm
  while (!pq.empty()) {
    int currentDistance = pq.top().first;
    int currentVertex = pq.top().second;
    pq.pop();

    // Update distances for neighbors
    for (int neighbor = 0; neighbor < _vertex_count; ++neighbor) {
      int weight = _adjacency_list[currentVertex][neighbor];
      if (weight != 0) {
        int newDistance = currentDistance + weight;
        if (newDistance < weights[_vertices[neighbor]]) {
          weights[_vertices[neighbor]] = newDistance;
          previous[_vertices[neighbor]] = _vertices[currentVertex];
          pq.push({newDistance, neighbor});
        }
      }
    }
  }

  int valueToErase = numeric_limits<int>::max();

  // Remove unwanted information from the map
  map<string, int> clean_weights;
  map<string, string> clean_previous;

  auto it = weights.begin();
  for (; it != weights.end(); ++it) {
    if ((it->second != valueToErase) && (it->first != startLabel)){
      clean_weights[it->first] = it->second;
    }
  }

  auto itp = previous.begin();
  for (; itp != previous.end(); ++itp) {
    if ((itp->first != startLabel) && (itp->second != "")){
      clean_previous[itp->first] = itp->second;
    }
  }

  //weights.erase(startLabel);
  //previous.erase(startLabel);

  return make_pair(clean_weights, clean_previous);
}

// minimum spanning tree using Prim's algorithm
int Graph::mstPrim(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const {
  if (!contains(startLabel)) {
    // Start vertex not found
    return -1;
  }

  int startIndex =
      find(_vertices.begin(), _vertices.end(), startLabel) - _vertices.begin();

  // Priority queue to store edges with their weights
  priority_queue<pair<int, pair<int, int>>, vector<pair<int, pair<int, int>>>,
                 greater<pair<int, pair<int, int>>>>
      pq;

  // Set to keep track of visited vertices
  set<int> visitedVertices;

  // Add edges from the start vertex to the priority queue
  for (int neighbor = 0; neighbor < _vertex_count; ++neighbor) {
    if (_adjacency_list[startIndex][neighbor] != 0) {
      pq.push({_adjacency_list[startIndex][neighbor], {startIndex, neighbor}});
    }
  }

  visitedVertices.insert(startIndex);

  int mstLength = 0;

  // Prim's algorithm
  while (!pq.empty() && visitedVertices.size() < _vertex_count) {
    int weight = pq.top().first;
    int fromVertex = pq.top().second.first;
    int toVertex = pq.top().second.second;
    pq.pop();

    if (visitedVertices.count(toVertex) == 0) {
      // Visit the edge
      visit(_vertices[fromVertex], _vertices[toVertex], weight);

      // Update MST length
      mstLength += weight;

      // Mark the "to" vertex as visited
      visitedVertices.insert(toVertex);

      // Add edges from the "to" vertex to the priority queue
      for (int neighbor = 0; neighbor < _vertex_count; ++neighbor) {
        if (_adjacency_list[toVertex][neighbor] != 0 &&
            visitedVertices.count(neighbor) == 0) {
          pq.push({_adjacency_list[toVertex][neighbor], {toVertex, neighbor}});
        }
      }
    }
  }

  return mstLength;
}

// minimum spanning tree using Kruskal's algorithm

int findSet(vector<int> &parent, int i) {
    if (parent[i] == -1)
        return i;
    return findSet(parent, parent[i]);
}

void unionSets(vector<int> &parent, int x, int y) {
    int rootX = findSet(parent, x);
    int rootY = findSet(parent, y);
    parent[rootX] = rootY;
}

int Graph::mstKruskal(void visit(const string &from, const string &to, int weight)) const {
    if (_isDirectional) {
        cout << "Kruskal's algorithm only works for undirected graphs." << endl;
        return -1;
    }

    vector<pair<int, pair<int, int>>> edges; // {weight, {vertex1, vertex2}}

    // Populate the edges vector with all edges in the graph
    for (int i = 0; i < _vertex_count; ++i) {
        for (int j = i + 1; j < _vertex_count; ++j) {
            if (_adjacency_list[i][j] != 0) {
                edges.push_back({ _adjacency_list[i][j], {i, j} });
            }
        }
    }

    // Sort edges in non-decreasing order of weight
    sort(edges.begin(), edges.end());

    // Initialize parent array for Union-Find
    vector<int> parent(_vertex_count, -1);

    int minSpanningTreeLength = 0;

    for (const auto &edge : edges) {
        int weight = edge.first;
        int u = edge.second.first;
        int v = edge.second.second;

        int setU = findSet(parent, u);
        int setV = findSet(parent, v);

        // Check if including this edge forms a cycle
        if (setU != setV) {
            visit(_vertices[u], _vertices[v], weight);
            minSpanningTreeLength += weight;
            unionSets(parent, setU, setV);
        }
    }

    return minSpanningTreeLength;
}

#if 0
int Graph::mstKruskal(void visit(const string &from, const string &to,
                                 int weight)) const {
  visit = visit;

  return -1;
}

   // Helper struct to represent an edge
    struct Edge {
        int weight;
        int fromVertex;
        int toVertex;
    };

    // Priority queue to store edges with their weights
    priority_queue<pair<int, Edge>, vector<pair<int, Edge>>, greater<pair<int, Edge>>> pq;

    // Populate the priority queue with all edges
    for (int i = 0; i < _vertex_count; ++i) {
        for (int j = i + 1; j < _vertex_count; ++j) {
            if (_adjacency_list[i][j] != 0) {
                Edge edge = { _adjacency_list[i][j], i, j };
                pq.push({ _adjacency_list[i][j], edge });
            }
        }
    }

    // Disjoint Set Union (DSU) data structure
    vector<int> parent(_vertex_count);
    iota(parent.begin(), parent.end(), 0);

    // Helper function to find the representative of a set
    auto findSet = [&](int vertex) {
        if (parent[vertex] != vertex) {
            parent[vertex] = findSet(parent[vertex]);
        }
        return parent[vertex];
    };

    int mstLength = 0;

    // Kruskal's algorithm
    while (!pq.empty()) {
        int weight = pq.top().first;
        Edge edge = pq.top().second;
        pq.pop();

        // Check if adding the edge creates a cycle
        int representativeFrom = findSet(edge.fromVertex);
        int representativeTo = findSet(edge.toVertex);

        if (representativeFrom != representativeTo) {
            // Visit the edge
            visit(_vertices[edge.fromVertex], _vertices[edge.toVertex], weight);

            // Update MST length
            mstLength += weight;

            // Merge the two sets
            parent[representativeFrom] = representativeTo;
        }
    }

    return mstLength;
}
#endif

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