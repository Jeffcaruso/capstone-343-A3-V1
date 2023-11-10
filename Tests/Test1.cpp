//============================================================================
// Name        : Test1
// Test Desc.  : Test == (basic)
//				 	(coverage for old test 1 method)
// Author      : Jeffrey Caruso, Yusuf Pisan
// Date    	   : Fall 2023
//============================================================================

#include <gtest/gtest.h>
#include "applib/graph.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

/**
 * Trying to avoid global variables,
 * by creating a singleton class with our visitor functions
 * stringstream SS contains the output from visitor
 */
class Tester {
public:
  Tester() = delete;
  // insert output to SS rather than cout, so we can test it
  static stringstream SS;
  static string getSs() { return SS.str(); }
  static void resetSs() { SS.str(string()); }
  // visitor function used for DFS and BFS
  static void labelVisitor(const string &Label) { SS << Label; }
  // visitor function used for edges for minimum spanning tree
  static void edgeVisitor(const string &From, const string &To, int Weight) {
    SS << "[" << From << To << " " << Weight << "]";
  }
};

// initialize the static variable
// NOLINTNEXTLINE
stringstream Tester::SS;

// convert a map to a string so we can compare it
template <typename K, typename L>
static string map2string(const map<K, L> &Mp) {
  stringstream Out;
  for (auto &P : Mp)
    Out << "[" << P.first << ":" << P.second << "]";
  return Out.str();
}



TEST(Test2, BasicGraphTesting)
{
	Graph G();

	EXPECT_EQ(G.add("a") , "add vertex a")
	EXPECT_EQ(G.add("b") , "add vertex b")
	EXPECT_EQ(G.add("c") , "add vertex c")
	EXPECT_EQ(G.add("d") , "add vertex d")
	EXPECT_EQ(G.add("e") , "add vertex e")

//   Graph G;
//   assert(G.add("a") && "add vertex a");
//   assert(G.add("b") && "add vertex b");
//   assert(G.add("c") && "add vertex c");
//   assert(G.add("d") && "add vertex d");
//   assert(G.add("e") && "add vertex e");



//   assert(!G.add("b") && "b added twice");
//   assert(G.connect("a", "b", 10) && "connect a b");
//   assert(!G.connect("a", "b", 50) && "duplicate connect a b");
//   assert(!G.connect("a", "a", 1) && "connect a to itself");
//   G.connect("a", "d", 40);
//   G.connect("a", "c", 20);
//   assert((G.verticesSize() == 5) && "graph number of vertices");
//   assert((G.edgesSize() == 3) && "graph number of edges");
//   assert((G.neighborsSize("a") == 3) && "vertex number of edges");
//   assert((G.neighborsSize("c") == 0) && "no outgoing edges c");
//   assert((G.neighborsSize("xxx") == -1) && "no edges for xxx");
//   assert(!G.contains("xxx") && "xxx not in graph");
//   assert(G.contains("a") && "a in graph");

//   // check that they are sorted based on edge end label
//   assert(G.getEdgesAsString("a") == "b(10),c(20),d(40)");
//   // disconnect non-existent edge/vertex
//   assert(!G.disconnect("a", "e") && "disconnecting non-existent vertex");
//   assert((G.edgesSize() == 3) && "disconnected nonexisting");
//   assert(G.disconnect("a", "c") && "a-c disconnect");
//   assert((G.edgesSize() == 2) && "number of edges after disconnect");
//   assert((G.neighborsSize("a") == 2) && "a has 2 edges");
//   assert(G.getEdgesAsString("a") == "b(10),d(40)" && "removing middle edge");
}
