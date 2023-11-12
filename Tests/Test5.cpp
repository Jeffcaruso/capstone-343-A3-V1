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



//Testing Graph0 with non-directed links
TEST(Test5, testGraph0NotDirected)
{
	//trivial 'test' to see if the gTest connection is setup...

//   cout << "testGraph0NotDirected" << endl;
//   bool IsDirectional = false;
//   Graph G(IsDirectional);
//   if (!G.readFile("graph0.txt"))
//     return;

//   Tester::resetSs();
//   G.bfs("A", Tester::labelVisitor);
//   assert(Tester::getSs() == "ABC" && "starting from A");

//   Tester::resetSs();
//   G.dfs("B", Tester::labelVisitor);
//   assert(Tester::getSs() == "BAC" && "starting from B");

//   Tester::resetSs();
//   G.dfs("C", Tester::labelVisitor);
//   assert(Tester::getSs() == "CAB" && "starting from C");

//   Tester::resetSs();
//   G.dfs("X", Tester::labelVisitor);
//   assert(Tester::getSs().empty() && "starting from X");

//   map<string, int> Weights;
//   map<string, string> Previous;
//   tie(Weights, Previous) = G.dijkstra("A");
//   // cout << "Dijkstra(A) weights is " << map2string(weights) << endl;
//   assert(map2string(Weights) == "[B:1][C:4]" && "Dijkstra(A) weights");
//   // cout << "Dijkstra(A) previous is " << map2string(previous) << endl;
//   assert(map2string(Previous) == "[B:A][C:B]" && "Dijkstra(A) previous");

//   tie(Weights, Previous) = G.dijkstra("B");
//   assert(map2string(Weights) == "[A:1][C:3]" && "Dijkstra(B) weights");
//   assert(map2string(Previous) == "[A:B][C:B]" && "Dijkstra(B) previous");

//   tie(Weights, Previous) = G.dijkstra("X");
//   assert(map2string(Weights).empty() && "Dijkstra(C) weights");
//   assert(map2string(Previous).empty() && "Dijkstra(C) previous");

//   Tester::resetSs();
//   int MstLength = G.mst("A", Tester::edgeVisitor);
//   assert(MstLength == 4 && "mst A is 4");
//   assert(Tester::getSs() == "[AB 1][BC 3]" && "mst A is [AB 1][BC 3]");

//   Tester::resetSs();
//   MstLength = G.mst("B", Tester::edgeVisitor);
//   assert(MstLength == 4 && "mst 4 is 4");
//   assert(Tester::getSs() == "[BA 1][BC 3]");

//   Tester::resetSs();
//   MstLength = G.mst("C", Tester::edgeVisitor);
//   assert(MstLength == 4 && "mst C is 4");
//   assert(Tester::getSs() == "[CB 3][BA 1]");

//   Tester::resetSs();
//   MstLength = G.mst("X", Tester::edgeVisitor);
//   assert(MstLength == -1 && "mst X is -1");
//   assert(Tester::getSs().empty() && "mst for vertex not found");


}
