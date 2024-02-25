//============================================================================
// Name        : Test6
// Test Desc.  : testing graph 1
//				 	(coverage for old testGraph1(). was #6 test method...)
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
TEST(Test6, testGraph1)
{
    Graph G;
    //read in graph1
    EXPECT_TRUE(G.readFile("data/graph1.txt"));

    //5.1 - BFS A
    Tester::resetSs();
    string ansStr = "A";
    G.dfs(ansStr, Tester::labelVisitor);
    ansStr = "ABCDEFGH";
    EXPECT_EQ(ansStr, Tester::getSs()); // "dfs starting from A"

    //5.2 - DFS B
    Tester::resetSs();
    ansStr = "B";
    G.dfs(ansStr, Tester::labelVisitor);
    ansStr = "BCDEFG";
    EXPECT_EQ(ansStr, Tester::getSs()); // dfs starting from B

    //5.3 - BFS B
    Tester::resetSs();
    ansStr = "B";
    G.bfs(ansStr, Tester::labelVisitor);
    ansStr = "BCDEFG";
    EXPECT_EQ(ansStr, Tester::getSs()); // dfs starting from B


    //5.4 - Dijkstra A
    map<string, int> Weights;
    map<string, string> Previous;
    ansStr = "A";
    auto P = G.dijkstra(ansStr);
    Weights = P.first;
    Previous = P.second;
    //5.4.1
    ansStr = "[B:1][C:2][D:3][E:4][F:5][G:4][H:3]";
    EXPECT_EQ(ansStr, map2string(Weights)); // Dijkstra(B) weights

    //5.4.2
    ansStr = "[B:A][C:B][D:C][E:D][F:E][G:H][H:A]";
    EXPECT_EQ(ansStr, map2string(Previous));
}
