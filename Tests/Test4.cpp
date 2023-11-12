//============================================================================
// Name        : Test4
// Test Desc.  : testing Dijkstra on graph 0
//				 	(coverage for old testGraph0Dijkstra(). was #4 test method...)
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

/*
 * Note, if testing is not working as expected, there is a decent chance of some degree of misconfigurations.
 * I.E.   assert(G.contains("A") && "a in graph"); -> EXPECT_TRUE(G.contains("A")); // a in graph
 * 				May not be as correct of an interpretation as I am expecting...
 *
 */



// detailed == testing
TEST(Test4, Graph0Dijkstra)
{
	// prep graph, read in graph0.txt
	Graph G;
	if (!G.readFile("graph0.txt"))
		return;

	// maps for weights (current) + previous
	map<string, int> Weights;
	map<string, string> Previous;

	// T1.1
	tie(Weights, Previous) = G.dijkstra("A");
	// cout << "Dijkstra(A) weights is " << map2string(weights) << endl;
	string ansStr = "[B:1][C:4]";
	EXPECT_EQ(map2string(Weights), ansStr); // Dijkstra(A) weights
	// assert(map2string(Weights) == "[B:1][C:4]" && "Dijkstra(A) weights");
	ansStr = "[B:A][C:B]";
	EXPECT_EQ(map2string(Previous), ansStr); // Dijkstra(A) previous
	// cout << "Dijkstra(A) previous is " << map2string(previous) << endl;
	// assert(map2string(Previous) == "[B:A][C:B]" && "Dijkstra(A) previous");

	// T1.2
	tie(Weights, Previous) = G.dijkstra("B");
	ansStr = "[C:3]";
	EXPECT_EQ(map2string(Weights), ansStr); // Dijkstra(B) weights
	// assert(map2string(Weights) == "[C:3]" && "Dijkstra(B) weights");
	ansStr = "[C:B]";
	EXPECT_EQ(map2string(Previous), ansStr); // Dijkstra(B) previous
	// assert(map2string(Previous) == "[C:B]" && "Dijkstra(B) previous");

	// T1.3
	tie(Weights, Previous) = G.dijkstra("X");

	EXPECT_TRUE(map2string(Weights).empty()); // Dijkstra(C) weights
	// assert(map2string(Weights).empty() && "Dijkstra(C) weights");
	EXPECT_TRUE(map2string(Previous).empty()); // Dijkstra(C) previous
	// assert(map2string(Previous).empty() && "Dijkstra(C) previous");
}
