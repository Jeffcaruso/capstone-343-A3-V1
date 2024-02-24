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

    // // convert a map to a string so we can compare it
    // template <typename K, typename L>
    // static string map2string(const map<K, L> &Mp);
};

// // initialize the static variable
// // NOLINTNEXTLINE
// stringstream Tester::SS;

