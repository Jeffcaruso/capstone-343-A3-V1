//============================================================================
// Name        : Tester 
// File Desc.  : Support Google Test files. implement tester.
// Author(s)   : Yusuf Pisan pisan@uw.edu, Jeffrey Caruso jc12321@uw.edu
// Date    	   : Fall 2023
//============================================================================
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>

#include "tester.h"

/**
 * Trying to avoid global variables,
 * by creating a singleton class with our visitor functions
 * stringstream SS contains the output from visitor
 */
using namespace std;

//Tester() = delete;
// insert output to SS rather than cout, so we can test it
static stringstream SS;
string Tester::getSs() { return SS.str(); }
void Tester::resetSs() { SS.str(string()); }
// visitor function used for DFS and BFS
void Tester::labelVisitor(const string &Label) { SS << Label; }
// visitor function used for edges for minimum spanning tree
void Tester::edgeVisitor(const string &From, const string &To, int Weight)
{
    SS << "[" << From << To << " " << Weight << "]";
}

// initialize the static variable
// NOLINTNEXTLINE
stringstream Tester::SS;