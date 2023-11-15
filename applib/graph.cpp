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
  directionalEdgeCheck = directionalEdges;
}

// destructor
Graph::~Graph() {
  //Built in map to remove all values in map
  allVertexes.clear();
}

// @return total number of vertices
int Graph::verticesSize() const {
  return allVertexes.size();
}

// @return total number of edges
int Graph::edgesSize() const {
  int mapCount = 0;
  int innerMapSize;

  //Loops and counts all edges in master map
  for(auto& i:allVertexes){
    //innerMapSize = allVertexes[i].first.size();
    if(innerMapSize > 1){
      mapCount = innerMapSize;
    }
    else{
      mapCount++;
    }
  }
  return mapCount;
}

// @return number of edges from given vertex, -1 if vertex not found
int Graph::vertexDegree(const string &label) const { return 0; }

// @return true if vertex added, false if it already is in the graph
bool Graph::add(const string &label) { 
  //New vertex, will add
  if(contains(label) == false){
    //

    return true;
  }
  //Already in vertex, will not add
  else{
    return false;
  }
  }

/** return true if vertex already in graph */
bool Graph::contains(const string &label) const {
  //Goes through allVertex to see any instances of target label
  if(allVertexes.count(label) == 1){
    return true;
  }
  //Not in vertex
  else{
    return false;
  }
}

// @return string representing edges and weights, "" if vertex not found
// A-3->B, A-5->C should return B(3),C(5)
string Graph::getEdgesAsString(const string &label) const { 

  //Target vertex for label not found
  if(contains(label) == false){
    return "";
  }

  //Will build up our edge string
  string buildingEdgeString;
  //Range based loop, looking for all edges of target label
  for(auto &i : allVertexes){
    buildingEdgeString = buildingEdgeString + i.first + "(" + ")";
  }
  //Returns fully built string
  return buildingEdgeString; 
}

// @return true if successfully connected
bool Graph::connect(const string &from, const string &to, int weight) {

  //Cannot connect a vertex on itself
  if(from == to){
    return false;
  }
  //We found another connection
  if(contains(from) == true){

    //Comparing if we have a repeat value
    //string tempCompare = allVertexes.at(from).at(to);
    //if(tempCompare.compare(to) == 0){
    //  return false;
    //}
    //Checks inner map for if we already have connection
    allVertexes.at(from).at(to);

    //Check if the vertex is 
    allVertexes.at(from);
  }

  //Will connect
  if(weight==1){
    //Map temp used to help insert into allVertexes
    map<string, int> temp;
    temp.insert(pair<string, int>(to, weight));

    //Insert into the master map, with alternative emplace
    allVertexes.emplace(pair<string, map<string,int>>(from, temp));
  }
  //Successfully connected
  return true;
}

bool Graph::disconnect(const string &from, const string &to) { return true; }

// depth-first traversal starting from given startLabel
void Graph::dfs(const string &startLabel, void visit(const string &label)) {}

// breadth-first traversal starting from startLabel
void Graph::bfs(const string &startLabel, void visit(const string &label)) {}

// store the weights in a map
// store the previous label in a map
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &startLabel) const {
  map<string, int> weights;
  map<string, string> previous;
  // TODO(student) Your code here
  return make_pair(weights, previous);
}

// minimum spanning tree using Prim's algorithm
int Graph::mstPrim(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const {
  return -1;
}

// minimum spanning tree using Kruskal's algorithm
int Graph::mstKruskal(void visit(const string &from, const string &to,
                                 int weight)) const {
  return -1;
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