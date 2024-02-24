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

// // convert a map to a string so we can compare it
// template <typename K, typename L>
// string Tester::map2string(const map<K, L> &Mp)
// {
//     stringstream Out;
//     for (auto &P : Mp)
//         Out << "[" << P.first << ":" << P.second << "]";
//     return Out.str();
// }

// // initialize the static variable
// // NOLINTNEXTLINE
stringstream Tester::SS;