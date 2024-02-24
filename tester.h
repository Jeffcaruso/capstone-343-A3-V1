//============================================================================
// Name        : Tester 
// File Desc.  : define testers
// Author(s)   : Yusuf Pisan pisan@uw.edu, Jeffrey Caruso jc12321@uw.edu
// Date    	   : Fall 2023
//============================================================================
#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
/**
 * Trying to avoid global variables,
 * by creating a singleton class with our visitor functions
 * stringstream SS contains the output from visitor
 */
using namespace std;

class Tester
{
public:
	Tester() = delete;
	// insert output to SS rather than cout, so we can test it
	static stringstream SS;
	static string getSs();
	static void resetSs();
	// visitor function used for DFS and BFS
	static void labelVisitor(const string &Label);
	// visitor function used for edges for minimum spanning tree
	static void edgeVisitor(const string &From, const string &To, int Weight);

};

// // initialize the static variable
// // NOLINTNEXTLINE
// stringstream Tester::SS;

