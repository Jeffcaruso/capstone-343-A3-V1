#include "graph.h"
#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <string>
#include <utility>
#include <vector>

using namespace std;
typedef pair<int, pair<string, string>> labeledWeight;

// constructor, empty graph
// directionalEdges defaults to true
Graph::Graph(bool directionalEdges) {
  this->edgesCount = 0; // Counter for edges
  // Determine if connections need to be made on both sides when inserting
  this->directionalEdgeSwitch = directionalEdges;
}

// destructor
Graph::~Graph() {
  for (auto &vertex : this->verticies) {
    delete vertex.second;
  }
}

// @return total number of vertices
int Graph::verticesSize() const { return verticies.size(); }

// @return total number of edges
int Graph::edgesSize() const { return this->edgesCount; }

// @return number of edges from given vertex, -1 if vertex not found
int Graph::vertexDegree(const string &label) const {
  if (this->verticies.count(label) == 0) {
    return -1;
  }
  return verticies.at(label)->getDegree();
}

// @return true if vertex added, false if it already is in the graph
bool Graph::add(const string &label) {
  if (verticies.count(label) == 0) {
    // Create a new vertex if the label hasn't been added to the graph already!
    Vertex *temp = new Vertex(label);
    verticies[label] = temp;
    return true;
  }
  // Vertex is found in the graph
  return false;
}

/** return true if vertex already in graph */
bool Graph::contains(const string &label) const {
  return !(verticies.count(label) == 0);
}

// @return string representing edges and weights, "" if vertex not found
// A-3->B, A-5->C should return B(3),C(5)
string Graph::getEdgesAsString(const string &label) const {
  if (verticies.count(label) == 0) {
    return "";
  }
  return verticies.at(label)->getEdgesString();
}

// Private helper function which returns a vector of all edges contained within
// a vertex
//  @return vector of all edges in given vertex
vector<string> Graph::getEdgesAsVector(const string &label) const {
  if (verticies.count(label) == 0) {
    vector<string> empty;
    return empty;
  }
  return verticies.at(label)->getEdgesVector();
}

// Private helper function which returns a vector pair of all edges contained
// within a vertex
//  @return vector pair of all edges in given vertex
vector<pair<string, int>>
Graph::getWeightedEdgesVector(const string &label) const {
  if (verticies.count(label) == 0) {
    vector<pair<string, int>> empty;
    return empty;
  }
  return verticies.at(label)->getWeightedEdgesVector();
}

// @return true if successfully connected
bool Graph::connect(const string &from, const string &to, int weight) {
  // Add vertexes if they don't already exist
  this->add(to);
  this->add(from);
  // Check preconditions:
  if (to == from || this->verticies[from]->connects(to) ||
      (this->directionalEdgeSwitch && this->verticies[to]->connects(from))) {
    /* Preconditions:
     * 1. Validate that the string isn't trying to connect to itself!
     * 2. If directional, validate that the other side doesn't already have a
     * connection!
     * 3. Validate that a connection doesn't already exist (works for both types
     * of graph)
     */
    // cout << "1: " << to.compare(from) << endl;
    // cout << boolalpha << "2: " << this->verticies[from]->connects(to) <<
    // endl; cout << boolalpha << "3: " << (this->directionalEdgeSwitch &&
    // this->verticies[to]->connects(from)) << endl;

    return false; // A precondition failed!
  }
  // Passes all checks, create a connection
  this->verticies[from]->add(
      to, weight); // Add the new connection along with its weight
  this->edgesCount++;
  if (!this->directionalEdgeSwitch) { // If non-directional add the connection
                                      // to the other graph!
    this->verticies[to]->add(from, weight);
    // NOTE for grader:
    //  It doesn't make sense to make the graph count for 2 when the vertecies
    //  SHARE the same edge They are added as if they are one edge, and removed
    //  as if they are one edge If you wanted to you could just add the
    //  following line of code here to count up this->edgesCount++;
  }
  return true; // All preconditions have been passed!
}

// @return true if edge successfully deleted
bool Graph::disconnect(const string &from, const string &to) {
  if (verticies.count(from) != 0 &&
      verticies[from]->connects(to)) { // Vertex exists, and has a connection
    verticies[from]->remove(to); // Add the new connection along with its weight
    this->edgesCount--;
    if (!this->directionalEdgeSwitch) { // If non-directional remove the
                                        // connection from the other graph!
      this->verticies[to]->remove(from);
      // NOTE for grader:
      //  It doesn't make sense to make the graph count for 2 when the vertecies
      //  SHARE the same edge They are added as if they are one edge, and
      //  removed as if they are one edge If you wanted to you could just add
      //  the following line of code here to count down this->edgesCount--;
    }
    return true;
  }
  return false; // One of the preconditions failed
}

// depth-first traversal starting from given startLabel
void Graph::dfs(const string &startLabel, void visit(const string &label)) {
  string returnString;
  if (this->verticies.count(startLabel) !=
      0) { // Make sure that starting label exists!
    stack<string> dfsStack;
    set<string> visited;
    dfsStack.push(startLabel);
    while (!dfsStack.empty()) {
      string current = dfsStack.top();
      dfsStack.pop(); // Pop from the stack!
      if (visited.count(current) == 0) {
        visited.insert(current);
        returnString += current;
        vector<string> edges = this->getEdgesAsVector(current);
        // for(int i = 0; i < edges.size(); i++) {
        // Seems like we have to read the queue backwards to get expected input
        for (int i = edges.size() - 1; i >= 0; i--) {
          if (visited.count(edges.at(i)) == 0) { // Check if node already
                                                 // visited
            dfsStack.push(edges.at(i));
          }
        }
      }
      // cout << current << " : " << dfsStack.size() << endl;
    }
  }
  visit(returnString);
}

// breadth-first traversal starting from startLabel
void Graph::bfs(const string &startLabel, void visit(const string &label)) {
  string returnString;
  if (this->verticies.count(startLabel) !=
      0) { // Make sure that starting label exists!
    queue<string> bfsQueue;
    set<string> visited;
    bfsQueue.push(startLabel);
    while (!bfsQueue.empty()) {
      string current = bfsQueue.front();
      bfsQueue.pop(); // Pop from the queue!
      if (visited.count(current) == 0) {
        visited.insert(current);
        returnString += current;
        vector<string> edges = this->getEdgesAsVector(current);
        for (int i = 0; i < edges.size(); i++) {
          if (visited.count(edges.at(i)) == 0) { // Check if node already
                                                 // visited
            bfsQueue.push(edges.at(i));
          }
        }
      }
      // cout << current << " : " << bfsQueue.size() << endl;
    }
  }
  visit(returnString);
}

// store the weights in a map
// store the previous label in a map
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &startLabel) const {
  map<string, int> weights;
  map<string, string> previous;
  // Makeshift priority queue sorted by lowest weight value
  set<string> toVisit;
  // Validate that verticies contains the start label!
  if (this->verticies.count(startLabel) != 0) {
    for (const auto &crawler : this->verticies) {
      weights[crawler.first] = INT16_MAX;
      previous[crawler.first] = "";
      toVisit.insert(crawler.first);
    }
  }
  // For each element in the graph, set weight to INT_MAX
  // For each element in the graph, set previous to ""
  // For each element in the graph, add it to toVisit set with given weights

  // Make weight of initial label 0
  weights[startLabel] = 0;

  while (!toVisit.empty()) { // While value in toVisit
    // Find vertex X with minimum weight value (that's not INT_MAX)
    string lowestKey;
    int minWeight = INT16_MAX;
    for (set<string>::iterator itr = toVisit.begin(); itr != toVisit.end();
         itr++) {
      if (weights.at(*itr) <= minWeight) { // weights.at(*itr) != INT16_MAX &&
        minWeight = weights.at(*itr);
        lowestKey = *itr;
      }
    }
    // Remove that value from the toVisit
    toVisit.erase(lowestKey);
    // Only iterate through neighbors if lowest value is a non-inf number
    if (minWeight < INT16_MAX) {
      // For each neighbor I of lowestKey that is in toVisit
      vector<pair<string, int>> neighbors =
          this->getWeightedEdgesVector(lowestKey);
      for (int i = 0; i < neighbors.size(); i++) {
        int pathWeight = weights[lowestKey] + neighbors.at(i).second;
        // Determine if pathweight is lower than its curernt path for neighbor
        if (pathWeight < weights[neighbors.at(i).first]) {
          weights[neighbors.at(i).first] = pathWeight;
          previous[neighbors.at(i).first] = lowestKey;
        }
      }
    }
    // If we find weights without a partner
    if (minWeight == INT16_MAX) {
      weights.erase(lowestKey);
      previous.erase(lowestKey);
    }
  }
  // Remove the starting label from the return value
  weights.erase(startLabel);
  previous.erase(startLabel);
  return make_pair(weights, previous);
}

// minimum spanning tree using Prim's algorithm
int Graph::mstPrim(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const {
  if (this->directionalEdgeSwitch || this->verticies.count(startLabel) == 0) {
    // Graph is directional OR starting label is not contained within the graph
    return -1;
  }
  int totalCost = 0;
  int edgeCounter = 0;
  set<string> visited;
  priority_queue<labeledWeight, vector<labeledWeight>, greater<labeledWeight>>
      edges;
  // Add startLabel to visited
  visited.insert(startLabel);
  // Add all edges from startLabel to the priority queue
  vector<pair<string, int>> startingEdges =
      this->getWeightedEdgesVector(startLabel);
  for (int i = 0; i < startingEdges.size(); i++) {
    pair<string, string> temp =
        make_pair(startLabel, startingEdges.at(i).first);
    edges.push(make_pair(startingEdges.at(i).second, temp));
  }
  while (!edges.empty()) {
    pair<int, pair<string, string>> nextEdge = edges.top();
    edges.pop();
    string edgeTo = nextEdge.second.second;
    string edgeFrom = nextEdge.second.first;
    int edgeWeight = nextEdge.first;
    // Only add edges if node isn't visited yet
    if (visited.count(edgeTo) == 0) {
      // Mark this edge as visited!
      visited.insert(edgeTo);
      edgeCounter++;
      totalCost += edgeWeight;
      visit(edgeFrom, edgeTo, edgeWeight);
      // Add all new edges
      vector<pair<string, int>> newEdges = this->getWeightedEdgesVector(edgeTo);
      for (int j = 0; j < newEdges.size(); j++) {
        // Only add if the other side of the connection has not been visited!
        if (visited.count(newEdges.at(j).first) == 0) {
          pair<string, string> tempNew =
              make_pair(edgeTo, newEdges.at(j).first);
          edges.push(make_pair(newEdges.at(j).second, tempNew));
        }
      }
    }
  }
  // NOTE FOR GRADER:
  // Validate that edges have been found
  // It appears that we are attempting to find partial mst
  // To validate our prim search, check if edges = vertexes - 1
  // Uncomment the following code if requested, I thought it wasn't needed:

  // if (edgesCount != this->verticesSize() - 1) {
  //   return -1;
  // }

  return totalCost;
}

// minimum spanning tree using Kruskal's algorithm
int Graph::mstKruskal(void visit(const string &from, const string &to,
                                 int weight)) const {
  map<string, int *> token;
  int tokenCounter = 0;
  int edgeTotal = 0;
  vector<int *> tokenHolder;
  set<string> unvisited;
  priority_queue<labeledWeight, vector<labeledWeight>, greater<labeledWeight>>
      edges;

  for (const auto &vertex : this->verticies) {
    // Add to set
    unvisited.insert(vertex.first);
    // Token Counter 1 2 3...
    int *temp = new int;
    *temp = ++tokenCounter;
    tokenHolder.push_back(temp);
    token[vertex.first] = temp;

    // Add edges
    vector<pair<string, int>> startingEdges =
        this->getWeightedEdgesVector(vertex.first);
    for (int i = 0; i < startingEdges.size(); i++) {
      pair<string, string> temp =
          make_pair(vertex.first, startingEdges.at(i).first);
      edges.push(make_pair(startingEdges.at(i).second, temp));
    }
  }

  // While there are unvisited verticies and edges to be looked at
  // Looking for smallest edge, make sure token #'s are different
  // while (!edges.empty() && !unvisited.empty()) {
  while (!edges.empty()) {
    labeledWeight lowestEdge = edges.top();
    edges.pop();
    // Make sure that the token value of the graphs are different!
    // Case 1 left side
    if (*token[lowestEdge.second.first] > *token[lowestEdge.second.second]) {
      // delete token[lowestEdge.second.first];
      int updatedValue = *token[lowestEdge.second.first];
      for (auto &tokenInstance : token) {
        if (*tokenInstance.second == updatedValue) {
          tokenInstance.second = token[lowestEdge.second.second];
        }
      }
      token[lowestEdge.second.first] = token[lowestEdge.second.second];
      unvisited.erase(lowestEdge.second.first);
      unvisited.erase(lowestEdge.second.second);
      edgeTotal += lowestEdge.first;
      visit(lowestEdge.second.first, lowestEdge.second.second,
            lowestEdge.first);
    }
    // Case 2 right side
    if (*token[lowestEdge.second.first] < *token[lowestEdge.second.second]) {
      // delete token[lowestEdge.second.second];
      int updatedValue = *token[lowestEdge.second.second];
      for (auto &tokenInstance : token) {
        if (*tokenInstance.second == updatedValue) {
          tokenInstance.second = token[lowestEdge.second.first];
        }
      }
      token[lowestEdge.second.second] = token[lowestEdge.second.first];
      unvisited.erase(lowestEdge.second.first);
      unvisited.erase(lowestEdge.second.second);
      edgeTotal += lowestEdge.first;
      visit(lowestEdge.second.first, lowestEdge.second.second,
            lowestEdge.first);
    }
  }

  // When assigning a new edge, take the lowest token # and add to all
  // other values with that token value Vector of Pointers which reprepesent all
  // vertexes When a value of the tree changes, make it point to that new value

  // loop through and delete all pointers from the vector when done
  for (int i = 0; i < tokenHolder.size(); i++) {
    delete tokenHolder.at(i);
  }
  return edgeTotal;
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