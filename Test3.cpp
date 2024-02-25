//============================================================================
// Name        : Test3
// Test Desc.  : testing Breadth First Search (BFS) on graph 0
//				 	(coverage for old testGraph0BFS(). was #3 test method...)
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
TEST(Test3, Graph0BFS)
{
	//prep graph, read in graph0.txt
	Graph G;
	//read in graph0
	EXPECT_TRUE(G.readFile("data/graph0.txt"));

	//A
	Tester::resetSs();
	string ansStr = "A";
	G.bfs(ansStr, Tester::labelVisitor);
	ansStr = "ABC";
	EXPECT_EQ(Tester::getSs(), ansStr); // starting from A

	//B
	Tester::resetSs();
	ansStr = "B";
	G.dfs(ansStr, Tester::labelVisitor);
	ansStr = "BC";
	EXPECT_EQ(Tester::getSs(), ansStr); // starting from B

	//C
	Tester::resetSs();
	ansStr = "C";
	G.dfs(ansStr, Tester::labelVisitor);
	ansStr = "C";
	EXPECT_EQ(Tester::getSs(), ansStr); // starting from C

	//X
	Tester::resetSs();
	ansStr = "X";
	G.dfs(ansStr, Tester::labelVisitor);
	EXPECT_TRUE(Tester::getSs().empty()); // starting from X
}
