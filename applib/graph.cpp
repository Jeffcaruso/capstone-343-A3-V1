#include "graph.h"
#include <algorithm>
#include <cassert>
#include <map>
#include <utility>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <queue>
#include <stack>
#include <set>
#include <limits>

using namespace std;

Graph::Graph(bool directedEdges) { DirectionalEdges = directedEdges; }

Graph::~Graph() {}

// bool Graph::readFile(const string &Filename) { return true; }

// @return total number of vertices
int Graph::verticesSize() const { return vertices.size(); }

// @return total number of edges
int Graph::edgesSize() const
{
	int edges = 0;
	for (auto vertex : vertices)
	{
		edges += edgeMap.at(vertex).size();
	}
	return edges;
}

// @return number of edges from given vertex, -1 if vertex not found
int Graph::neighborsSize(const string &Label) const
{
	if (edgeMap.count(Label) == 0)
	{
		return -1;
	}
	return edgeMap.at(Label).size();
}

// @return true if vertex added, false if it already is in the graph
bool Graph::add(const string &Label)
{
	if (contains(Label))
	{
		return false;
	}
	vertices.insert(Label);
	edgeMap[Label] = vector<Edge>();
	return true;
}

/** return true if vertex already in graph */
bool Graph::contains(const std::string &Label) const { return vertices.find(Label) != vertices.end(); }


//alt option to actually sort...?
bool cmp(Edge &v1, Edge &v2)
{
    if (strcmp(v1.getStart(v1).c_str(), v2.getStart(v2).c_str()) == 0)
    {
        return false;
    }
    else
    {
        //I think these are correct, but jist in case I got them flipped, lets see...
        // v#.end.c_str() ?
        if(strcmp(v1.getStart(v1).c_str(), v2.getStart(v2).c_str()) < 0)
        {
            return true;  //expected
            //return false;
        }
        else
        {
            return false; // expected
            //return true;
        }
    }
}

// @return string representing edges and weights, "" if vertex not found
// A-3->B, A-5->C should return B(3),C(5)
string Graph::getEdgesAsString(const string &Label) const
{
	if (!contains(Label))
	{
		return "";
	}
	string edgeString;
	vector<Edge> edges(edgeMap.at(Label));
	sort(edges.begin(), edges.end(), cmp);
	for (auto edge : edges)
	{
		if (Label == edge.end)
		{
			if (DirectionalEdges)
			{
				continue;
			}
			edgeString += edge.start;
		}
		else
		{
			edgeString += edge.end;
		}
		edgeString += "(" + to_string(edge.weight) + "),";
	}
	if (edgeString.length() > 0)
	{
		edgeString.erase(edgeString.length() - 1);
	}
	return edgeString;
}

// @return true if successfully connected, false if not connected.
bool Graph::connect(const string &From, const string &To, int Weight)
{
	if (From == To)
	{
		return false;
	}
	add(From);
	add(To);
	for (auto edge : edgeMap[From])
	{
		if (edge.end == To || (!DirectionalEdges && edge.start == To))
		{
			return false;
		}
	}
	// Vertex fr(From);
	// Vertex t(To);
	// Edge newEdge(*fr, *t, Weight);
	Vertex fr(From);
	Vertex t(To);
	Edge newEdge(&fr, &t, Weight);

	edgeMap[From].push_back(newEdge);
	if (!DirectionalEdges)
	{
		Edge secondEdge(&t, &fr, Weight);
		edgeMap[To].push_back(secondEdge);
	}
	return true;
}

// @return true if edge successfully deleted
bool Graph::disconnect(const string &From, const string &To)
{
	if (From == To || !contains(From) || !contains(To))
	{
		return false;
	}
	bool removed = false;
	vector<Edge> toRemove;
	for (auto &edge : edgeMap[From])
	{
		if (edge.end == To || edge.start == To)
		{
			toRemove.push_back(edge);
			removed = true;
		}
	}
	for (auto &edge : toRemove)
	{
		vector<Edge> &edgeMapFrom = edgeMap[From];
		edgeMapFrom.erase(remove(edgeMapFrom.begin(), edgeMapFrom.end(), edge),
						  edgeMapFrom.end());
	}
	toRemove.clear();
	for (auto &edge : edgeMap[To])
	{
		if (edge.end == From)
		{
			toRemove.push_back(edge);
		}
	}
	for (auto &edge : toRemove)
	{
		vector<Edge> &edgeMapTo = edgeMap[To];
		edgeMapTo.erase(remove(edgeMapTo.begin(), edgeMapTo.end(), edge),
						edgeMapTo.end());
	}
	return removed;
}

// depth-first traversal starting from given startLabel
void Graph::dfs(const string &StartLabel, void Visit(const string &Label))
{
	if (!contains(StartLabel))
	{
		return;
	}
	stack<string> toVisit;
	unordered_map<string, bool> visited;
	visited[StartLabel] = true;
	toVisit.push(StartLabel);
	while (!toVisit.empty())
	{
		string label = toVisit.top();
		toVisit.pop();
		Visit(label);
		vector<Edge> sorted(edgeMap[label]);
		sort(sorted.begin(), sorted.end(),
			 [](Edge a, Edge b)
			 { return a.end > b.end; });
		for (auto edge : sorted)
		{
			string other = edge.end;
			if (!visited[other])
			{
				visited[other] = true;
				toVisit.push(other);
			}
		}
	}
}

// breadth-first traversal starting from startLabel
void Graph::bfs(const string &StartLabel, void Visit(const string &Label))
{
	if (!contains(StartLabel))
	{
		return;
	}
	queue<string> toVisit;
	unordered_map<string, bool> visited;
	visited[StartLabel] = true;
	toVisit.push(StartLabel);
	while (!toVisit.empty())
	{
		string label = toVisit.front();
		toVisit.pop();
		Visit(label);
		vector<Edge> sorted(edgeMap[label]);
		sort(sorted.begin(), sorted.end(),
			 [](Edge a, Edge b)
			 { return a.end < b.end; });
		for (auto edge : sorted)
		{
			string other = edge.end;
			if (!visited[other])
			{
				visited[other] = true;
				toVisit.push(other);
			}
		}
	}
}
// used to sort pairs of vertices and weights
struct CompareBySecond
{
	bool operator()(const std::pair<std::string, int> &a,
					const std::pair<std::string, int> &b) const
	{
		return a.second > b.second;
	}
};

// store the weights in a map
// store the previous label in a map
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &StartLabel) const
{
	map<string, int> weights;
	map<string, string> previous;
	if (!contains(StartLabel))
	{
		return make_pair(weights, previous);
	}
	unordered_map<string, bool> visited;
	set<pair<string, int>, CompareBySecond> verticesToVisit;
	string nextVisit = StartLabel;
	while (!nextVisit.empty())
	{
		visited[nextVisit] = true;
		for (auto edge : edgeMap.at(nextVisit))
		{
			if (weights.count(edge.end) == 0 ||
				weights[edge.end] > weights[nextVisit] + edge.weight)
			{
				weights[edge.end] = weights[nextVisit] + edge.weight;
				previous[edge.end] = nextVisit;
				verticesToVisit.insert(make_pair(edge.end, weights[edge.end]));
			}
		}
		while (!verticesToVisit.empty() &&
			   visited[(*verticesToVisit.begin()).first])
		{
			verticesToVisit.erase(verticesToVisit.begin());
		}
		if (verticesToVisit.empty())
		{
			nextVisit = "";
		}
		else
		{
			nextVisit = (*verticesToVisit.begin()).first;
		}
	}
	weights.erase(StartLabel);
	return make_pair(weights, previous);
}

/**
 * minimum spanning tree
 * @param function to be called on each edge
 * @return length of the minimum spanning tree or -1 if start vertex not found
 */
int Graph::mst(const string &StartLabel,
			   void Visit(const string &From, const string &To,
						  int Weight))
{
	// Prim's Algo
	if (DirectionalEdges || !contains(StartLabel))
	{
		return -1;
	}
	unordered_map<string, bool> mst;
	mst[StartLabel] = true;
	int totalWeight = 0;
	bool finished = false;
	while (!finished)
	{
		finished = true;
		int cheapestWeight = numeric_limits<int>::max();
		string cheapestTo;
		string cheapestFrom;
		for (auto vertex : mst)
		{
			for (auto edge : edgeMap.at(vertex.first))
			{
				if (mst.count(edge.end) == 0 && edge.weight < cheapestWeight)
				{
					cheapestWeight = edge.weight;
					cheapestTo = edge.end;
					cheapestFrom = vertex.first;
				}
			}
		}
		if (!cheapestTo.empty())
		{
			finished = false;
			mst[cheapestTo] = true;
			totalWeight += cheapestWeight;
			Visit(cheapestFrom, cheapestTo, cheapestWeight);
		}
	}
	return totalWeight;
}
// maybe need to remove const at end of this !!!!!!!!!!!!!!!!!!!!!!!!!
// int Graph::mst(const string &StartLabel,
//                void Visit(const string &From, const string &To,
//                           int Weight)) const {
//  to w/o that last const...?

// read a text file and create the graph
bool Graph::readFile(const char *filename)
{
	// TCHAR NPath[MAX_PATH];
	// GetCurrentDirectory(MAX_PATH, NPath);
	// std::cout << NPath << std::endl;
	ifstream myfile(filename);
	if (!myfile.is_open())
	{
		cerr << "Failed to open " << filename << endl;
		return false;
	}
	int edges = 0;
	int weight = 0;
	string fromVertex;
	string toVertex;
	myfile >> edges;
	for (int i = 0; i < edges; ++i)
	{
		myfile >> fromVertex >> toVertex >> weight;
		connect(fromVertex, toVertex, weight);
	}
	myfile.close();
	return true;
}
