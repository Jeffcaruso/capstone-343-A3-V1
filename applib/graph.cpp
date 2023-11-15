#include "graph.h"
#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <utility>
#include <vector>

using namespace std;

// constructor, empty graph
Graph::Graph(bool directional) : isDirectional(directional) {}

/** destructor, delete all vertices and edges */
Graph::~Graph() {}

// @return true if vertex added, false if it already is in the graph
bool Graph::add(const std::string &label) {
    if (vertices.find(label) != vertices.end()) {
        return false; 
    }
    vertices[label] = {};
    return true;
}

// Add an edge between two vertices, create new vertices if necessary
  // A vertex cannot connect to itself, cannot have P->P
  // For digraphs (directed graphs), only one directed edge allowed, P->Q
  // Undirected graphs must have P->Q and Q->P with same weight
  // @return true if successfully connected
bool Graph::connect(const std::string &from, const std::string &to, int weight) {
    // Check for self-connection and return false if detected
    if (from == to) {
        return false;
    }

    if (vertices.find(from) == vertices.end()) {
        vertices[from] = std::unordered_map<std::string, int>();
    }
    if (vertices.find(to) == vertices.end()) {
        vertices[to] = std::unordered_map<std::string, int>();
    }

    if (vertices[from].find(to) != vertices[from].end()) {
        return false; 
    }

    vertices[from][to] = weight;
    if (!isDirectional) {
        vertices[to][from] = weight;
    }
    return true;
}

// @return total number of vertices
int Graph::verticesSize() const {
    return vertices.size();
}

// @return total number of edges
int Graph::edgesSize() const {
    int size = 0;
    for (const auto &vertex : vertices) {
        size += vertex.second.size();
    }
    if (!isDirectional) {
        size /= 2; 
    }
    return size;
}

// @return number of edges from given vertex, -1 if vertex not found
int Graph::vertexDegree(const std::string &label) const {
    auto it = vertices.find(label);
    if (it == vertices.end()) {
        return -1; 
    }
    return it->second.size();
}

// @return true if vertex is in the graph
bool Graph::contains(const std::string &label) const {
    return vertices.find(label) != vertices.end();
}

std::string Graph::getEdgesAsString(const std::string &label) const {
    auto it = vertices.find(label);
    if (it == vertices.end()) {
        return ""; 
    }

    // Convert the edges (adjacent vertices) of the found vertex into a vector
    std::vector<std::pair<std::string, int>> edges(it->second.begin(), it->second.end());

    // Sort the edges lexicographically by vertex label for consistent output
    std::sort(edges.begin(), edges.end(), [](const std::pair<std::string, int> &a, const std::pair<std::string, int> &b) {
        return a.first < b.first;
    });

    std::string result;

    for (const auto &edge : edges) {
        // Append the vertex label and its associated weight to the result string
        result += edge.first + "(" + std::to_string(edge.second) + "),";
    }

    // Remove the trailing comma
    if (!result.empty()) {
        result.pop_back(); 
    }

    return result;
}

// Remove edge from graph
  // @return true if edge successfully deleted
bool Graph::disconnect(const std::string &from, const std::string &to) {
    if (vertices[from].erase(to) > 0) {
        if (!isDirectional) {
            vertices[to].erase(from);
        }
        return true;
    }
    return false;
}

// depth-first traversal starting from given startLabel
void Graph::dfs(const std::string &start, void (*func)(const std::string &)) {
    if (vertices.find(start) == vertices.end()) {
        return;  
    }
    std::set<std::string> visited;
    dfsHelper(start, visited, func);
}

void Graph::dfsHelper(const std::string &vertex, std::set<std::string> &visited, void (*func)(const std::string &)) {
    if (vertices.find(vertex) == vertices.end()) {
        return;
    }
    
    if (visited.find(vertex) == visited.end()) {
        visited.insert(vertex);
        func(vertex);
        
        std::vector<std::string> sortedNeighbors;
        for (const auto &neighbour : vertices[vertex]) {
            sortedNeighbors.push_back(neighbour.first);
        }
        std::sort(sortedNeighbors.begin(), sortedNeighbors.end());
        
        for (const std::string &neighbour : sortedNeighbors) {
            dfsHelper(neighbour, visited, func);
        }
    }
}

// breadth-first traversal starting from startLabel
// call the function visit on each vertex label */
void Graph::bfs(const std::string &start, void (*func)(const std::string &)) {
    if (vertices.find(start) == vertices.end()) { 
        return;
    }

    std::set<std::string> visited;
    std::queue<std::string> q;
    
    visited.insert(start);
    q.push(start);

    while (!q.empty()) {
        std::string current = q.front();
        q.pop();
        func(current);

        std::vector<std::string> neighbors;
        for (const auto &neighbour : vertices[current]) {
            neighbors.push_back(neighbour.first);
        }
        std::sort(neighbors.begin(), neighbors.end());

        for (const auto &neighbor : neighbors) {
            if (visited.find(neighbor) == visited.end()) {
                visited.insert(neighbor);
                q.push(neighbor);
            }
        }
    }
}


// store the weights in a map
// store the previous label in a map
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &startLabel) const {
    map<string, int> weights;
    map<string, string> previous;

    if (vertices.find(startLabel) == vertices.end()) 
        return make_pair(weights, previous);

    set<string> unvisited;
    priority_queue<pair<int, string>, vector<pair<int, string>>, greater<pair<int, string>>> pq;

    // Initialize weights and previous maps, and populate the unvisited set
    for (const auto &vertexPair : vertices) {
        const string &vertex = vertexPair.first;
        weights[vertex] = INT_MAX;
        previous[vertex] = "";
        unvisited.insert(vertex);
    }

    // Distance from start vertex to itself is always 0
    weights[startLabel] = 0;
    pq.push({0, startLabel});

    while (!pq.empty()) {
        string currentVertex = pq.top().second;
        pq.pop();

        // If the current vertex hasn't been visited yet
        if (unvisited.find(currentVertex) != unvisited.end()) {
            unvisited.erase(currentVertex);

            for (const auto &neighborPair : vertices.at(currentVertex)) {
                const string &neighbor = neighborPair.first;
                int edgeWeight = neighborPair.second;

                // Relaxation step
                if (weights[currentVertex] + edgeWeight < weights[neighbor]) {
                    weights[neighbor] = weights[currentVertex] + edgeWeight;
                    previous[neighbor] = currentVertex;
                    pq.push({weights[neighbor], neighbor});
                }
            }
        }
    }

    weights.erase(startLabel);
    previous.erase(startLabel);
    for (auto it = weights.begin(); it != weights.end();) {
        if (it->second == INT_MAX) {
            it = weights.erase(it);
        } else {
            ++it;
        }
    }

    for (auto it = previous.begin(); it != previous.end();) {
        if (weights.find(it->first) == weights.end()) {
            it = previous.erase(it);
        } else {
            ++it;
        }
    }

    return make_pair(weights, previous);
}

// minimum spanning tree using Prim's algorithm
int Graph::mstPrim(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const {
    // Initialize all vertices as not visited
    if (vertices.find(startLabel) == vertices.end()) {
        return -1;
    }

    unordered_map<string, bool> visited;
    
    for (const auto &pair : vertices) {
        visited[pair.first] = false;
    }

    // Set the start vertex as visited
    visited[startLabel] = true;

    int mstWeight = 0;
    // The number of edges in MST will be vertices.size() - 1
    for (size_t i = 1; i < vertices.size(); i++) {
        int minWeight = INT_MAX;
        string u, v;

        // Find the minimum weight edge from the set of visited vertices
        // Find the minimum weight edge from the set of visited vertices
    for (const auto &src : vertices) {
        for (const auto &dest : src.second) {
            if (visited[src.first] && !visited[dest.first] && dest.second < minWeight) {
                u = src.first;
                v = dest.first;
                minWeight = dest.second;
            }
        }
    }


        // Mark the vertex as visited and add the edge weight to the MST weight
        visited[v] = true;
        mstWeight += minWeight;
        visit(u, v, minWeight);
    }

    return mstWeight;
}

// minimum spanning tree using Kruskal's algorithm
int Graph::mstKruskal(void visit(const string &from, const string &to, int weight)) const {
    // Initialize the parent and rank structures.
    for (const auto &vertex : vertices) {
        parent[vertex.first] = vertex.first;
        rank[vertex.first] = 0;
    }

    // Get all edges from the graph and sort them.
    vector<pair<int, pair<string, string>>> edges;
    for (const auto &vertex : vertices) {
        for (const auto &neighbor : vertex.second) {
            if (vertex.first < neighbor.first) {
                edges.push_back({neighbor.second, {vertex.first, neighbor.first}});
            }
        }
    }
    sort(edges.begin(), edges.end());

    int mstWeight = 0;
    for (const auto &edge : edges) {
        string root1 = findSet(edge.second.first);
        string root2 = findSet(edge.second.second);
        
        // Check if the edge causes a cycle
        if (root1 != root2) {
            mstWeight += edge.first;
            visit(edge.second.first, edge.second.second, edge.first);
            unionSets(root1, root2);
        }
    }

    return mstWeight;
}

// Read edges from file
  // first line of file is an integer, indicating number of edges
  // each line represents an edge in the form of "string string int"
  // vertex labels cannot contain spaces
  // @return true if file successfully read
bool Graph::readFile(const string &filename) {
  ifstream myfile(filename);
  if (!myfile.is_open()) {
    cerr << "Failed to open " << filename << endl;
    return false;
  }
  int edges = 0;
  int weight = 0;
  string fromVertex;
  string toVertex;
  myfile >> edges;
  for (int i = 0; i < edges; ++i) {
    myfile >> fromVertex >> toVertex >> weight;
    connect(fromVertex, toVertex, weight);
  }
  myfile.close();
  return true;
}

string Graph::findSet(const string &vertex) const {
    // If the vertex is not its own representative (i.e., not the root of its set)
    // Recursively find the representative of the set and path compress.
    if (vertex != parent[vertex])
        parent[vertex] = findSet(parent[vertex]);

    // Return the representative (root) of the set the vertex belongs to.
    return parent[vertex];
}

bool Graph::unionSets(const string &u, const string &v) const {
    // Find the representatives (roots) of the sets u and v belong to.
    string pu = findSet(u);
    string pv = findSet(v);

    // If u and v are in the same set, return false to indicate no union performed.
    if (pu == pv) return false;

    // Make the root of the smaller rank tree point to the root of the larger rank tree.
    if (rank[pu] < rank[pv]) std::swap(pu, pv);
    parent[pv] = pu;

    // If both trees have the same rank, increase the rank of the resulting tree.
    if (rank[pu] == rank[pv]) rank[pu]++;

    // Return true to indicate that a union was performed.
    return true;
}


const std::unordered_map<std::string, std::unordered_map<std::string, int>>& Graph::getVertices() const {
    return vertices;
}
