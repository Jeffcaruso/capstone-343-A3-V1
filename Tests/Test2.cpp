//============================================================================
// Name        : Test2
// Test Desc.  : Testing == (detailed) testing and operations
//				 	(coverage for old test 2 method)
// Author      : Jeffrey Caruso, Yusuf Pisan
// Date    	   : Fall 2023
//============================================================================

#include <gtest/gtest.h>
#include "applib/bst.h"

#include <iostream>
#include <sstream>

using namespace std;

//note / reminder:
// use EXPECT_EQ (or _NE) for std::string
// for c strings, use EXPECT_STREQ (or STRNE)
// http://google.github.io/googletest/reference/assertions.html


/**
 * Testing BST - Binary Search Tree functions
 *
 * This file has series of tests for BST
 * Each test is independent and uses assert statements
 * Test functions are of the form
 *
 *      test_netidXX()
 *
 * where netid is UW netid and XX is the test number starting from 01
 *
 * Test functions can only use the public functions from BST
 * testBSTAll() is called from main in main.cpp
 * testBSTAll calls all other functions
 * @author Multiple
 * @date ongoing
 */

/**
 * Trying to avoid global variables,
 * by creating a singleton class with our visitor functions
 * stringstream SS contains the output from visitor
 */
class TreeVisitor {
public:
  // never create an instance of TreeVisitor object
  // we'll just use the static functions
  TreeVisitor() = delete;

  // insert output to SS rather than cout, so we can test it
  static stringstream SS;

  // get SS as a string
  static string getSS() { return SS.str(); }

  // set SS to be empty string
  static void resetSS() { SS.str(string()); }

  // instead of cout, insert item into SS, a stringstream object
  static void visitor(const string &Item) { SS << Item; }

  // instead of cout, insert item into SS, a stringstream object
  static void visitor(const int &Item) { SS << Item; }
};

// initialize the static variable
//  warning: initialization of 'SS' with static storage duration
//  may throw an exception that cannot be caught [cert-err58-cpp]
//  Not sure how to do it without making code harder to read
//  NOLINTNEXTLINE
stringstream TreeVisitor::SS;

template <class T> void visitorSimple(const T &Item) {
  cout << "visitorSimple: " << Item;
}





//detailed == testing
TEST(Test2one, DetailedEqualityTesting)
{
	BST<string> B1;
	BST<string> B2;
	BST<string> B3;

	//add the following to B1
	for (auto &S : vector<string>{"c", "a", "f", "g", "x"})
	{
		B1.add(S);
	}

	//add the following to B2
	for (auto &S : vector<string>{"c", "f", "a", "g", "x"})
	{
		B2.add(S);
	}

	//add "b" to B3
	B3.add("b");

	// == for the 5 node trees
	EXPECT_TRUE(B1 == B2 && (!(B1 != B2)));

	//copy constructor usage
	BST<string> B4(B3);
	// copy constructor for 1-Node trees B3, B4
	EXPECT_TRUE(B3 == B4 && (!(B3 != B4)));

	//copy constructor usage
	BST<string> B5(B1);
	// copy constructor for 5-Node trees B1, B5
	EXPECT_TRUE(B1 == B5 && (!(B5 != B1)));

	//copy constructor usage
	BST<string> B7("b");
	// 1-param constructor for 1-Node trees B3, B7
	EXPECT_TRUE(B3 == B7 && (!(B3 != B7)));
}
