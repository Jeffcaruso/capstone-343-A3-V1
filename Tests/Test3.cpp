//============================================================================
// Name        : Test3
// Test Desc.  : Testing traversal methods on BSTs
//				 	(coverage for old test 3 method)
// Author      : Jeffrey Caruso, Yusuf Pisan
// Date    	   : Fall 2023
//============================================================================

#include <gtest/gtest.h>
#include "applib/bst.h"

#include <iostream>
#include <sstream>

using namespace std;


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



// Testing traversal
TEST(Test3, testBSTTraversal)
{
	BST<string> B1;
	BST<string> B2;
	BST<string> B3;
	
	//add to B1
    for (auto &S : vector<string>{"c", "a", "f", "g", "x"})
    	B1.add(S);

	//add to B2
    for (auto &S : vector<string>{"c", "f", "a", "g", "x"})
    	B2.add(S);

	//add to B3
    B3.add("b");

	//reset StringStream to ""
	TreeVisitor::resetSS();
	B1.inOrderTraverse(TreeVisitor::visitor);
	string expectedResult = "acfgx";
	// check that SS matches expected Result
	EXPECT_EQ(TreeVisitor::getSS(), expectedResult);

	// testing out simpleVisitor to demonstrate
	// any function that has a matching signature can be called
	B1.inOrderTraverse(visitorSimple);

	//reset SS
	TreeVisitor::resetSS();
	//do a preorder traversal
	B1.preOrderTraverse(TreeVisitor::visitor);
	//set expected result to new expected result...
	expectedResult = "cafgx";
	// check that SS matches expected Result
	EXPECT_EQ(TreeVisitor::getSS(), expectedResult);

	TreeVisitor::resetSS();
	B1.postOrderTraverse(TreeVisitor::visitor);
	expectedResult = "axgfc";
	// check that SS matches expected Result
	EXPECT_EQ(TreeVisitor::getSS(), expectedResult);

	// visual check of B1
	cout << "Visual check B1:" << endl;
	cout << B1 << endl;
}
