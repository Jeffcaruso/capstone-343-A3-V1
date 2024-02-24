//============================================================================
// Name        : Graph
// File Desc.  : implement Graph
// Author(s)   : Yusuf Pisan pisan@uw.edu, Jeffrey Caruso jc12321@uw.edu
// Date    	   : Fall 2023
//============================================================================
#include "graph.h"
#include <algorithm>
#include <climits>
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

// Graph()
//-----------------------------------------------------
// Description: This method creates an instance of the
// Graph class. Sets default bool for directional edges
// to be true.
//
// precondition: None.
//
// Postcondition: An instance of this class is created.
//-----------------------------------------------------
// @param directionalEdges If graph will be direct.
// @return An instance of this class.
//
Graph::Graph(bool directionalEdges) {
  this->directionalEdges = directionalEdges;
}

// destructor()
//-----------------------------------------------------
// Description: This method destroys an instance of the
// Graph class by iterating through vector of vertices.
//
// Precondition: An instance of this class should exist.
//
// Postcondition: None.
//-----------------------------------------------------
//
Graph::~Graph() {
  for (auto iter = vertices.begin(); iter != vertices.end();) {
    delete iter->second;
    iter->second = nullptr;

    iter++;
  }

  vertices.clear();
}

// verticesSize()
//-----------------------------------------------------
// Description: This method returns the size of the
// graph.
//
// Precondition: None.
//
// Postcondition: Returns the number of vertices that
// were in the instance of the Graph.
//-----------------------------------------------------
// @param None
// @return Number of vertices.
//
int Graph::verticesSize() const { return vertices.size(); }

// edgesSize()
//-----------------------------------------------------
// Description: This method returns the total number of
// edges from this instance of the Graph class.
//
// Precondition: None.
//
// Postcondition: The number of edges is returned.
//-----------------------------------------------------
// @return Total number of edges.
//
int Graph::edgesSize() const { return numOfEdges; }

// @return number of edges from given vertex, -1 if vertex not found
// neighborsSize()
//-----------------------------------------------------
// Description: This method returns the number of edges
// from specified vertex.
//
// Precondition: Vertex must exist.
//
// Postcondition: Number of edges of vertex is returned.
//-----------------------------------------------------
// @param label Vertex to get number of edges from.
// @return Number of edges from given vertex.
//
int Graph::neighborsSize(const string &label) const {
  if (!contains(label)) {
    return -1;
  }

  Vertex *currVertex = getVertex(label);
  return currVertex->edges.size();
}

// add()
//-----------------------------------------------------
// Description: This method adds a point to this graph.
// It checks if it exists or not.
//
// Precondition: Label must not already exist for the
// the insertion to work.
//
// Postcondition: Boolean is returned if the insertion
// worked.
//-----------------------------------------------------
// @param label The point to store in graph.
//
// @return Bool if insertion worked.
//
bool Graph::add(const string &label) {
  // If vertex exists, cancel.
  if (contains(label)) {
    return false;
  }

  // New vertex created.
  vertices[label] = new Vertex();
  vertices[label]->label = label;

  return true;
}

// contains()
//-----------------------------------------------------
// Description: This method checks if a lable already
// exists within the instance of the Graph class.
//
// Precondition: None. Label must be a string.
//
// Postcondition: A boolean is returned confirming if
// the label already exists.
//-----------------------------------------------------
// @param label The point to store in graph.
// @return Confirmation if Vertex with label exists.
//
bool Graph::contains(const string &label) const {
  return vertices.find(label) != vertices.end();
}

// getEdgesAsString()
//-----------------------------------------------------
// Description: This returns a string of all vertices
// connected to each other (excludes non-connected) in
// alphabetical order.
//
// Precondition: Connections should be made for string
// to return something.
//
// Postcondition: A string of all edges returned. If
// empty, returns -1.
//-----------------------------------------------------
// @param label All edges for specified vertex's label.
// @return string All edges.
//
string Graph::getEdgesAsString(const string &label) const {
  string edgesAsString;

  if (!contains(label)) {
    return edgesAsString;
  }

  Vertex *currVertex = getVertex(label);
  vector<Edge *> sortedEdges = currVertex->edges;

  sort(sortedEdges.begin(), sortedEdges.end(),
       [](Edge *a, Edge *b) { return a->target->label < b->target->label; });

  for (Edge *edge : sortedEdges) {
    if (!edgesAsString.empty()) {
      edgesAsString += ",";
    }

    edgesAsString += edge->target->label + "(" + to_string(edge->weight) + ")";
  }

  return edgesAsString;
}

// connect()
//-----------------------------------------------------
// Description: This method makes connections between
// two different vertices. If vertices don't exist,
// creates new vertex (or more) to connect.
//
// Precondition: Vertices must not be already connected
// same positions as "from" & "to". Cannot have the
// same weight
//
// Postcondition: Returns boolean if connection was
// successfully made.
//-----------------------------------------------------
// @param from Source of edge.
// @param to Target of edge.
// @param weight The weight of the connection.
// @return confirmation if connection was made.
//
bool Graph::connect(const string &from, const string &to, int weight) {
  if (!contains(from)) {
    add(from);
  }

  if (!contains(to)) {
    add(to);
  }

  if (isAlreadyConnected(from, to) || (from == to)) {
    cout << "ERROR: already connected or connecting itself" << endl;
    return false;
  }

  Vertex *source = getVertex(from);
  Vertex *target = getVertex(to);

  Edge *edge = new Edge();
  edge->source = source;
  edge->target = target;
  edge->weight = weight;

  source->edges.push_back(edge);

  if (!directionalEdges) {
    Edge *newEdge = new Edge();
    newEdge->source = target;
    newEdge->target = source;
    newEdge->weight = weight;

    target->edges.push_back(newEdge);
  }

  numOfEdges++;
  return true;
}

// isAlreadyConnected()
//-----------------------------------------------------
// Description: This method checks if two vertices are
// already connected. It creates an instance of the
// Edge class to do this.
//
// Precondition: Must have valid vertex arguments.
//
// Postcondition: An instance of the Edge class is made
// and connects the source to the target.
//-----------------------------------------------------
// @param from Source of the edge.
// @param t Target connecting Source with.
// @return Confirmation if connection made successfully.
//
bool Graph::isAlreadyConnected(const string &from, const string &t) {
  // Look for source in vertices.
  Vertex *sourceVertex = vertices[from];

  // Check if source already targets label T.
  for (Edge *edge : sourceVertex->edges) {
    if (edge->target->label == t) {
      cout << "isAlreadyConnected: source already connected to target" << endl;
      return true;
    }
  }

  return false;
}

// disconnect()
//-----------------------------------------------------
// Description: This method disconnects two vertices
// and deletes an instance of the Edge class.
//
// Precondition: Must have valid vertex arguments.
//
// Postcondition: An instance of the Edge class that
// connects specified vertices is removed.
//-----------------------------------------------------
// @param from Source of the edge.
// @param to Target we want to disconnect from.
// @return Confirmation if connection removed.
//
bool Graph::disconnect(const string &from, const string &to) {
  if (!contains(from) || !contains(to)) {
    cout << "ERROR: label for source or target doesn't exist in graph." << endl;
    return false;
  }

  Vertex *source = vertices[from];
  bool disconnected = false;

  // Iterate through the edges and mark the ones to remove.
  for (auto iter = source->edges.begin(); iter != source->edges.end();) {
    Edge *edge = *iter;

    string sourceLabel = edge->source->label;
    string targetLabel = edge->target->label;

    if ((sourceLabel == from) && (targetLabel == to)) {
      disconnected = true;

      delete edge;
      source->edges.erase(iter);
      break;
    }

    ++iter;
  }

  if (disconnected) {
    numOfEdges--;
  }

  return disconnected;
}

// dfs()
//-----------------------------------------------------
// Description: This method traverses the graph using
// the depth-first approach. utilizes "visit" method
// to record traversal. Uses startLabel.
//
// Precondition: startLabel must exist.
//
// Postcondition: Traversed the graph using the depth-
// first approach.
//-----------------------------------------------------
// @param startLabel Starting vertex.
// @param visit Method recording traversal.
//
void Graph::dfs(const string &startLabel, void visit(const string &label)) {
  if (!contains(startLabel)) {
    return;
  }

  stack<Vertex *> unvisitedVertices;
  set<Vertex *> seen;
  Vertex *startVertex = vertices[startLabel];

  unvisitedVertices.push(startVertex);
  seen.insert(startVertex);

  while (!unvisitedVertices.empty()) {
    Vertex *currVertex = unvisitedVertices.top();
    unvisitedVertices.pop();
    visit(currVertex->label);

    vector<Edge *> sortedEdges = currVertex->edges;

    sort(sortedEdges.begin(), sortedEdges.end(),
         [](Edge *a, Edge *b) { return a->target->label > b->target->label; });

    for (Edge *edge : sortedEdges) {
      Vertex *neighbor = edge->target;

      if (seen.find(neighbor) == seen.end()) {
        unvisitedVertices.push(neighbor);
        seen.insert(neighbor);
      }
    }
  }
}

// bfs()
//-----------------------------------------------------
// Description: This method traverses the graph using
// the breadth-first approach. utilizes "visit" method
// to record traversal. Uses startLabel.
//
// Precondition: startLabel must exist.
//
// Postcondition: Traversed the graph using the breadth-
// first approach.
//-----------------------------------------------------
// @param startLabel Starting vertex.
// @param visit Method recording traversal.
//
void Graph::bfs(const string &startLabel, void visit(const string &label)) {
  if (!contains(startLabel)) {
    return;
  }

  queue<Vertex *> unvisitedVertices;
  set<Vertex *> seen;

  Vertex *startVertex = vertices[startLabel];

  unvisitedVertices.push(startVertex);
  seen.insert(startVertex);
  visit(startVertex->label);

  while (!unvisitedVertices.empty()) {
    Vertex *currVertex = unvisitedVertices.front();
    unvisitedVertices.pop();

    vector<Edge *> sortedEdges = currVertex->edges;
    sort(sortedEdges.begin(), sortedEdges.end(),
         [](Edge *a, Edge *b) { return a->target->label < b->target->label; });

    for (Edge *edge : sortedEdges) {
      Vertex *neighbor = edge->target;

      if (seen.find(neighbor) == seen.end()) {
        unvisitedVertices.push(neighbor);
        seen.insert(neighbor);
        visit(neighbor->label);
      }
    }
  }
}

// dijsktra()
//-----------------------------------------------------
// Description: Traverses through graph by choosing
// edges that accumulate the least weight.
//
// Precondition: Starting label must be valid.
//
// Postcondition: Returns a Dijkstra traversal.
//-----------------------------------------------------
// @param startLabel Starting vertex.
// @return Pair labeling traversal of vertices & weight.
//
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &startLabel) const {
  map<string, int> weights;
  map<string, string> previous;
  map<Vertex *, int> unvisitedVertices;

  if (!contains(startLabel)) {
    return make_pair(weights, previous);
  }

  // Assign startLabel as best Vertex to use at start.
  weights[startLabel] = 0;

  // Assign the rest of Vertices with "infinity".
  for (const auto &pair : vertices) {
    if (pair.first != startLabel) {
      weights[pair.first] = INT_MAX;
    }

    previous[pair.first] = "";
    unvisitedVertices[pair.second] = weights[pair.first];
  }

  while (!unvisitedVertices.empty()) {
    Vertex *currVertex = extractMin(unvisitedVertices);

    if (currVertex->edges.empty()) {
      break;
    }

    for (Edge *edge : currVertex->edges) {
      Vertex *nextVertex = edge->target;
      int altDistance = weights[currVertex->label] + edge->weight;

      if (altDistance < weights[nextVertex->label]) {
        weights[nextVertex->label] = altDistance;
        previous[nextVertex->label] = currVertex->label;
        unvisitedVertices[nextVertex] = altDistance;
      }
    }
  }

  weights.erase(startLabel);
  previous.erase(startLabel);

  // Remove vertices with distance INT_MAX from the result.
  for (auto iter = weights.begin(); iter != weights.end();) {
    if (iter->second == INT_MAX) {
      iter = weights.erase(iter);
    } else {
      ++iter;
    }
  }

  // Remove unvisited Vertices from the result.
  for (auto iter = previous.begin(); iter != previous.end();) {
    if (iter->second.empty()) {
      iter = previous.erase(iter);
    } else {
      ++iter;
    }
  }

  return make_pair(weights, previous);
}

// extractMin()
//-----------------------------------------------------
// Description: This is a helper method for the method
// "Dijkstra". It gets the vertex with the smallest
// weight from a map.
//
// Precondition: Map must contain vertices.
//
// Postcondition: Returns the best vertex to use.
//-----------------------------------------------------
// @param sVertices Map of vertices to get vertex from
// @return The vertex with the smallest weight.
//
Vertex *Graph::extractMin(map<Vertex *, int> &sVertices) {
  Vertex *minVertex;
  int minWeight = INT_MAX;

  for (const auto &entry : sVertices) {
    if (entry.second < minWeight) {
      minVertex = entry.first;
      minWeight = entry.second;
    }
  }

  if (minVertex != nullptr) {
    sVertices.erase(minVertex);
  }

  return minVertex;
}

// minimum spanning tree using Prim's algorithm
// mstPrim()
//-----------------------------------------------------
// Description: Traverses through graph using Prim's
// Algorithm. Uses outside method to record traversal.
//
// Precondition: Starting label must be valid.
//
// Postcondition: Traveses through graph using Prim's
// Algorithm. Returns the size of MST.
//-----------------------------------------------------
// @param startLabel Starting vertex.
// @param visit Method recording traversal.
// @return Int of the minimum spanning tree size.
//mst prim
int Graph::mst(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const {
  if (!contains(startLabel)) {
    return -1;
  }

  vector<Edge *> minEdges;
  vector<Vertex *> unvisitedVertices;
  set<Edge *> seenEdge;
  set<Vertex *> seenVertex;

  int mstSize = 0;
  int overallSize = 0;

  unvisitedVertices.push_back(getVertex(startLabel));
  string currLabel = startLabel;
  Vertex *currVertex = unvisitedVertices.front();

  while (!unvisitedVertices.empty()) {
    overallSize++;

    // Once we reached the final vertex, break loop.
    if (overallSize == vertices.size()) {
      break;
    }

    seenVertex.insert(currVertex);

    // Adding all edges.
    for (Edge *edge : currVertex->edges) {
      if (seenVertex.count(edge->target) == 0) {
        minEdges.push_back(edge);
        unvisitedVertices.push_back(edge->target);
      }
    }

    int lowestWeight = minEdges.front()->weight;
    Edge *minEdge = minEdges.front();

    // Getting edge with lowest weight.
    for (Edge *edge : minEdges) {
      if (edge->weight < lowestWeight) {
        minEdge = edge;
      }
    }

    if (seenEdge.count(minEdge) == 0) {
      visit(minEdge->source->label, minEdge->target->label, minEdge->weight);
      currVertex = minEdge->target;
      mstSize += minEdge->weight;
      seenEdge.insert(minEdge);

      auto t = find(minEdges.begin(), minEdges.end(), minEdge);
      minEdges.erase(t);
    }
  }

  return mstSize;
}

// getVertex()
//-----------------------------------------------------
// Description: This method retrieves a vertex from the
// vertices map from this instance of the graph class.
//
// Precondition: Label should exist.
//
// Postcondition: Retrieves a pointer of the vertex
// from the vertices map.
//-----------------------------------------------------
// @param Label Target label containing vertex.
// @return Pointer to vertex containing target label.
//
Vertex *Graph::getVertex(const string &label) const {
  for (const auto &pair : vertices) {
    if (pair.first == label) {
      return pair.second;
    }
  }

  return nullptr;
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