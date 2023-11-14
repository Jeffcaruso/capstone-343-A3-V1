#include "graph.h"
#include <algorithm>
#include <cassert>
#include <map>
#include <utility>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

Graph:bool DirectionalEdges = false;

Graph::Graph(bool DirectionalEdges) {}

Graph::~Graph() {}

bool Graph::readFile(const string &Filename) { return true; }

int Graph::verticesSize() const { return 0; }

int Graph::edgesSize() const { return 0; }

int Graph::neighborsSize(const string &Label) const { return 0; }

bool Graph::add(const string &Label) { return false; }

/** return true if vertex already in graph */
bool Graph::contains(const std::string &Label) const { return false; }

string Graph::getEdgesAsString(const string &Label) const { return ""; }

bool Graph::connect(const string &From, const string &To, int Weight) {
  return true;
}

bool Graph::disconnect(const string &From, const string &To) { return true; }

void Graph::dfs(const string &StartLabel, void Visit(const string &Label)) {}

void Graph::bfs(const string &StartLabel, void Visit(const string &Label)) {}

// store the weights in a map
// store the previous label in a map
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &StartLabel) const {
  map<string, int> Weights;
  map<string, string> Previous;
  return make_pair(Weights, Previous);
}

/**
 * minimum spanning tree
 * @param function to be called on each edge
 * @return length of the minimum spanning tree or -1 if start vertex not found
 */
int Graph::mst(const string &StartLabel,
               void Visit(const string &From, const string &To,
                          int Weight)) {
  // TEST(Test5, testMST)
  // {
  //   EXPECT_FALSE(DirectionalEdges);
  // }
  //accessing DirectionalEdges was the issue !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // if(! DirectionalEdges)
  // {
  //   return -100; // invalid case... Could then use this to do what is needed...
  // }
  //assert(!DirectionalEdges);
  return 0;
}
//maybe need to remove const at end of this !!!!!!!!!!!!!!!!!!!!!!!!!
//int Graph::mst(const string &StartLabel,
//               void Visit(const string &From, const string &To,
//                          int Weight)) const {
// to w/o that last const...?

// maybe issue here is using assert and not Gtest...?