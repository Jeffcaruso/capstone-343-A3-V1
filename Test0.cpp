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
TEST(Test0, TrivialTest)
{
	Graph G(true);

    EXPECT_EQ(1,1);
}
