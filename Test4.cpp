//============================================================================
// Name        : Test4
// Test Desc.  : testing Dijkstra on graph 0
//				 	(coverage for old testGraph0Dijkstra(). was #4 test method...)
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
TEST(Test4, Graph0Dijkstra)
{
	// prep graph, read in graph0.txt
	Graph G;
	//read in graph 0
	EXPECT_TRUE(G.readFile("data/graph0.txt"));

	// maps for weights (current) + previous
	map<string, int> Weights;
	map<string, string> Previous;

	// T1.1
	tie(Weights, Previous) = G.dijkstra("A");
	string ansStr = "[B:1][C:4]";
	EXPECT_EQ(map2string(Weights), ansStr); // Dijkstra(A) weights
	ansStr = "[B:A][C:B]";
	EXPECT_EQ(map2string(Previous), ansStr); // Dijkstra(A) previous

	// T1.2
	tie(Weights, Previous) = G.dijkstra("B");
	ansStr = "[C:3]";
	EXPECT_EQ(map2string(Weights), ansStr); // Dijkstra(B) weights
	ansStr = "[C:B]";
	EXPECT_EQ(map2string(Previous), ansStr); // Dijkstra(B) previous

	// T1.3
	tie(Weights, Previous) = G.dijkstra("X");

	EXPECT_TRUE(map2string(Weights).empty()); // Dijkstra(C) weights
	EXPECT_TRUE(map2string(Previous).empty()); // Dijkstra(C) previous
}
