//============================================================================
// Name        : Test2
// Test Desc.  : testing Depth First Search (DFS) on graph 0
//				 	(coverage for old testGraph0DFS(). was #2 test method...)
// Author      : Jeffrey Caruso, Yusuf Pisan
// Date    	   : Fall 2023
//============================================================================

#include <gtest/gtest.h>
#include "graph.h"

#include "tester.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;


// convert a map to a string so we can compare it
template <typename K, typename L>
string map2string(const map<K, L> &Mp)
{
    stringstream Out;
    for (auto &P : Mp)
        Out << "[" << P.first << ":" << P.second << "]";
    return Out.str();
}



/// @brief GTest method for this file
TEST(Test2, Graph0DFS)
{
	Graph G;
	//read in Graph0
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
