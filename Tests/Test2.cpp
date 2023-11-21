//============================================================================
// Name        : Test2
// Test Desc.  : testing Depth First Search (DFS) on graph 0
//				 	(coverage for old testGraph0DFS(). was #2 test method...)
// Author      : Jeffrey Caruso, Yusuf Pisan
// Date    	   : Fall 2023
//============================================================================

#include <gtest/gtest.h>
#include "applib/graph.h"
//#include "tools/cpp/runfiles.h"
//using bazel::tools::cpp::runfiles::runfiles.h;


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
* NOTE: The only detail to notice here is reviewing if the error output is what I want...
*/



TEST(Test2, Graph0DFS)
{
	Graph G;
	// if (!G.readFile("graph0.txt"))
	// 	return;
	//EXPECT_TRUE(G.readFile("applib/graph0.txt"));
	//EXPECT_TRUE(G.readFile(path.c_str()));
	EXPECT_TRUE(G.readFile("data/graph0.txt"));



	EXPECT_TRUE(G.contains("A")); // a in graph
	EXPECT_TRUE(G.contains("B")); // b in graph
	EXPECT_TRUE(G.contains("C")); // c in graph
	//made it to here w/o error. So, file IO is now working!!
	string ansStr = "B(1),C(8)";
	EXPECT_EQ(G.getEdgesAsString("A"), ansStr);
	ansStr = "C(3)";
	EXPECT_EQ(G.getEdgesAsString("B"), ansStr);
	EXPECT_TRUE(G.getEdgesAsString("C").empty());

	Tester::resetSs();
	ansStr = "A";
	G.dfs(ansStr, Tester::labelVisitor);
	ansStr = "ABC";
	EXPECT_EQ(Tester::getSs(), ansStr); // starting from A

	Tester::resetSs();
	ansStr = "B";
	G.dfs(ansStr, Tester::labelVisitor);
	ansStr = "BC";
	EXPECT_EQ(Tester::getSs(), ansStr); // starting from B

	Tester::resetSs();
	ansStr = "C";
	G.dfs(ansStr, Tester::labelVisitor);
	ansStr = "C";
	EXPECT_EQ(Tester::getSs(), ansStr); // starting from C

	Tester::resetSs();
	ansStr = "X";
	G.dfs(ansStr, Tester::labelVisitor);
	ansStr = "BC";
	EXPECT_TRUE(Tester::getSs().empty()); // starting from X
}
