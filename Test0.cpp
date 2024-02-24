//============================================================================
// Name        : Test1
// Test Desc.  : Basic Graph Testing (not using any of the graph files, using fn calls to add...)
//				 	(coverage for old test 1 method)
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

// /**
//  * Trying to avoid global variables,
//  * by creating a singleton class with our visitor functions
//  * stringstream SS contains the output from visitor
//  */
// class Tester
// {
// public:
// 	Tester() = delete;
// 	// insert output to SS rather than cout, so we can test it
// 	static stringstream SS;
// 	static string getSs() { return SS.str(); }
// 	static void resetSs() { SS.str(string()); }
// 	// visitor function used for DFS and BFS
// 	static void labelVisitor(const string &Label) { SS << Label; }
// 	// visitor function used for edges for minimum spanning tree
// 	static void edgeVisitor(const string &From, const string &To, int Weight)
// 	{
// 		SS << "[" << From << To << " " << Weight << "]";
// 	}
// };

// // initialize the static variable
// // NOLINTNEXTLINE
// stringstream Tester::SS;

// // convert a map to a string so we can compare it
// template <typename K, typename L>
// static string map2string(const map<K, L> &Mp)
// {
// 	stringstream Out;
// 	for (auto &P : Mp)
// 		Out << "[" << P.first << ":" << P.second << "]";
// 	return Out.str();
// }


// convert a map to a string so we can compare it
template <typename K, typename L>
string map2string(const map<K, L> &Mp)
{
    stringstream Out;
    for (auto &P : Mp)
        Out << "[" << P.first << ":" << P.second << "]";
    return Out.str();
}

// initialize the static variable
// NOLINTNEXTLINE
//stringstream Tester::SS;


/// @brief GTest method for this file
TEST(Test0, TrivialTest)
{
	Graph G(true);

    EXPECT_EQ(1,1);
}
