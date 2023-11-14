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

/*
 * Note, if testing is not working as expected, there is a decent chance of some degree of misconfigurations.
 * I.E.   assert(G.contains("A") && "a in graph"); -> EXPECT_TRUE(G.contains("A")); // a in graph
 * 				May not be as correct of an interpretation as I am expecting...
 *
 */


// Testing Graph0 with non-directed links
TEST(Test5, testGraph1)
{
    //trivial test to validate gTest setup

    Graph G;
    if (!G.readFile("graph1.txt"))
        return;

    //5.1 - bfs
    Tester::resetSs();
    string ansStr = "A";
    G.dfs(ansStr, Tester::labelVisitor);
    ansStr = "ABCDEFGH";
    EXPECT_EQ(ansStr, Tester::getSs()); // "dfs starting from A"
    //assert(Tester::getSs() == "ABCDEFGH" && "dfs starting from A");

    //5.2 - dfs
    Tester::resetSs();
    ansStr = "B";
    G.dfs(ansStr, Tester::labelVisitor);
    ansStr = "BCDEFG";
    EXPECT_EQ(ansStr, Tester::getSs()); // dfs starting from B
    //assert(Tester::getSs() == "BCDEFG" && "dfs starting from B");

    //5.3 - bfs
    Tester::resetSs();
    ansStr = "B";
    G.bfs(ansStr, Tester::labelVisitor);
    ansStr = "BCDEFG";
    EXPECT_EQ(ansStr, Tester::getSs()); // dfs starting from B
    //assert(Tester::getSs() == "BCDEFG" && "dfs starting from B");

    //5.4 - dijkstra
    map<string, int> Weights;
    map<string, string> Previous;
    ansStr = "A";
    auto P = G.dijkstra(ansStr);
    Weights = P.first;
    Previous = P.second;
    //5.4.1
    ansStr = "[B:1][C:2][D:3][E:4][F:5][G:4][H:3]";
    EXPECT_EQ(ansStr, map2string(Weights)); // Dijkstra(B) weights
    //assert(map2string(Weights) == "[B:1][C:2][D:3][E:4][F:5][G:4][H:3]" && "Dijkstra(B) weights");

    //5.4.2
    ansStr = "[B:A][C:B][D:C][E:D][F:E][G:H][H:A]";
    EXPECT_EQ(ansStr, map2string(Previous));
    //assert(map2string(Previous) == "[B:A][C:B][D:C][E:D][F:E][G:H][H:A]" && "Dijkstra(B) previous");
}

