//============================================================================
// Name        : Test5
// Test Desc.  : testing undirected links on graph0
//				 	(coverage for old testGraph0NotDirected(). was #5 test method...)
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


/// @brief GTest method for this file
TEST(Test5, testGraph0NotDirected)
{
    //prep non-directional graph
    bool IsDirectional = false;
    Graph G(IsDirectional);
    //read in graph 0
    EXPECT_TRUE(G.readFile("data/graph0.txt"));

    //5.1.1
    Tester::resetSs();
    string ansStr = "A";
    G.bfs(ansStr, Tester::labelVisitor);
    ansStr = "ABC";
    EXPECT_EQ(Tester::getSs(), ansStr); // starting from A

    //5.1.2
    Tester::resetSs();
    ansStr = "B";
    G.dfs(ansStr, Tester::labelVisitor);
    ansStr = "BAC";
    EXPECT_EQ(Tester::getSs(), ansStr); // starting from B

    //5.1.3
    Tester::resetSs();
    ansStr = "C";
    G.dfs(ansStr, Tester::labelVisitor);
    ansStr = "CAB";
    EXPECT_EQ(Tester::getSs(), ansStr); // starting from C

    //5.1.4
    Tester::resetSs();
    ansStr = "X";
    G.dfs(ansStr, Tester::labelVisitor);
    EXPECT_TRUE(Tester::getSs().empty()); // starting from X


    //general focus shifting to the second sub-category of testing in this method

    // 5.2.1
    map<string, int> Weights;
    map<string, string> Previous;
    ansStr = "A";
    tie(Weights, Previous) = G.dijkstra(ansStr);
    ansStr = "[B:1][C:4]";
    EXPECT_EQ(map2string(Weights), ansStr); // Dijkstra(A) weights
    ansStr = "[B:A][C:B]";
    EXPECT_EQ(map2string(Previous), ansStr); // Dijkstra(A) previous

    // 5.2.2
    ansStr = "B";
    tie(Weights, Previous) = G.dijkstra(ansStr);
    ansStr = "[A:1][C:3]";
    EXPECT_EQ(map2string(Weights), ansStr); // Dijkstra(B) weights
    ansStr = "[A:B][C:B]";
    EXPECT_EQ(map2string(Previous), ansStr); // Dijkstra(B) previous

    // 5.2.3
    ansStr = "X";
    tie(Weights, Previous) = G.dijkstra(ansStr);
    EXPECT_TRUE(map2string(Weights).empty()); // Dijkstra(C) weights
    EXPECT_TRUE(map2string(Previous).empty()); // Dijkstra(C) previous

    // 5.2.4   !!! mst seems to be the cause of valgrund errors, but not too sure what I can do at this point...
    Tester::resetSs();
    ansStr = "A";
    int MstLength = G.mst(ansStr, Tester::edgeVisitor);
    EXPECT_TRUE(MstLength == 4); // mst A is 4
    ansStr = "[AB 1][BC 3]";
    EXPECT_EQ(Tester::getSs(), ansStr); // mst A is [AB 1][BC 3]

    // 5.2.5
    Tester::resetSs();
    ansStr = "B";
    MstLength = G.mst(ansStr, Tester::edgeVisitor);
    EXPECT_TRUE(MstLength == 4); // mst 4 is 4
    ansStr = "[BA 1][BC 3]";
    EXPECT_EQ(Tester::getSs(), ansStr);

    // 5.2.6
    Tester::resetSs();
    MstLength = G.mst("C", Tester::edgeVisitor);
    EXPECT_TRUE(MstLength == 4); // mst C is 4
    ansStr = "[CB 3][BA 1]";
    EXPECT_EQ(Tester::getSs(), ansStr);

    // 5.2.7
    Tester::resetSs();
    MstLength = G.mst("X", Tester::edgeVisitor);
    EXPECT_TRUE(MstLength == -1); // mst X is -1
    EXPECT_TRUE(Tester::getSs().empty()); // mst for vertex not found
}
