//============================================================================
// Name        : Test1
// Test Desc.  : Basic Graph Testing (not using any of the graph files, using fn calls to add...)
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
class Tester
{
public:
	Tester() = delete;
	// insert output to SS rather than cout, so we can test it
	static stringstream SS;
	static string getSs() { return SS.str(); }
	static void resetSs() { SS.str(string()); }
	// visitor function used for DFS and BFS
	static void labelVisitor(const string &Label) { SS << Label; }
	// visitor function used for edges for minimum spanning tree
	static void edgeVisitor(const string &From, const string &To, int Weight)
	{
		SS << "[" << From << To << " " << Weight << "]";
	}
};

// initialize the static variable
// NOLINTNEXTLINE
stringstream Tester::SS;

// convert a map to a string so we can compare it
template <typename K, typename L>
static string map2string(const map<K, L> &Mp)
{
	stringstream Out;
	for (auto &P : Mp)
		Out << "[" << P.first << ":" << P.second << "]";
	return Out.str();
}


TEST(Test1, BasicGraphTesting)
{
	Graph G(true);

	//add edges a-e to Graph G
	//all should pass (returning true)
	EXPECT_TRUE(G.add("a")); // add vertext a
	EXPECT_TRUE(G.add("b")); // add vertext b
	EXPECT_TRUE(G.add("c")); // add vertext c
	EXPECT_TRUE(G.add("d")); // add vertext d
	EXPECT_TRUE(G.add("e")); // add vertext e

	//duplicate adds, connections being made, etc...
	EXPECT_FALSE(G.add("b")); // b added twice
	EXPECT_TRUE(G.connect("a", "b", 10)); // connect a to b
	EXPECT_FALSE(G.connect("a", "b", 50)); // duplicate connect a to b
	EXPECT_FALSE(G.connect("a", "a", 1)); // connect a to itself

	//connect w/o testing, just get the rest done.
	G.connect("a", "d", 40); // connect a to d, cost 40
	G.connect("a", "c", 20); // connect a to c, cost 20

	//Testing methods for Graph data (and metadata) access
	EXPECT_EQ(G.verticesSize(), 5); // graph number of vertices
	EXPECT_EQ(G.edgesSize(), 3); // graph number of edges
	//test neighbors size
	EXPECT_EQ(G.neighborsSize("a"), 3); // vertex number of edges
	EXPECT_EQ(G.neighborsSize("c"), 0); // no outgoing edges for c
	EXPECT_EQ(G.neighborsSize("xxx"), -1); // no edges for xxx
	//test contains
	EXPECT_FALSE(G.contains("xxx")); // xxx not in graph
	EXPECT_TRUE(G.contains("a")); // a in graph
	// check that they are sorted based on edge end label	
	string expectedAns = "b(10),c(20),d(40)";
	EXPECT_EQ(G.getEdgesAsString("a"), expectedAns);

	// disconnect existent (and non-existent) edge/vertex
	EXPECT_FALSE(G.disconnect("a", "e")); // disconnecting non-existent vertex
	EXPECT_EQ(G.edgesSize(), 3); // post disconnected non-existing
	EXPECT_TRUE(G.disconnect("a", "c")); // a-c disconnect (existing)
	EXPECT_EQ(G.edgesSize(), 2); // number of edges after disconnect
	EXPECT_EQ(G.neighborsSize("a"), 2); // a has 2 edges
	expectedAns = "b(10),d(40)";
	EXPECT_EQ(G.getEdgesAsString("a"), expectedAns); // removing middle edge
}
