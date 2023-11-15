#include "graph.h"
#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <stack>
#include <utility>
#include <vector>

using namespace std;

// constructor, empty graph
// directionalEdges defaults to true
Graph::Graph(bool directionalEdges) {
  directional = directionalEdges;
  verts.clear();
  numV = 0;
}

// destructor
Graph::~Graph() {
  for (auto *v : verts) {
    deleteEdges(v->next);
    delete v;
  }
  verts.clear();
}

void Graph::deleteEdges(Vertex::Edge *cur) {
  if (cur != nullptr) {
    deleteEdges(cur->next);
    delete cur;
  }
}

// @return total number of vertices
int Graph::verticesSize() const { 
  cout << "number of vertices: " << numV << endl;
  return numV; 
}

// @return total number of edges
int Graph::edgesSize() const { 
  int edges {0};
  Vertex::Edge *cur;
  for (auto *v : verts) {
    cur = v->next;
    while (cur != nullptr) {
      edges++;
      cur = cur->next;
    }
  }
  cout << "number of edges: " << edges << endl;
  return edges; 
}

// @return number of edges from given vertex, -1 if vertex not found
int Graph::vertexDegree(const string &label) const { 
  for (auto *v : verts) {
    if (v->label == label) {
      int edges {0};
      Vertex::Edge *cur = v->next;
      while (cur !=nullptr) {
        edges++;
        cur = cur->next;
      }
      return edges;
    }
  }
  return -1; 
}

// @return true if vertex added, false if it already is in the graph
bool Graph::add(const string &label) { 
  if (contains(label)) {
    return false;
  }
  Vertex *add = new Vertex;
  add->label = label;
  verts.push_back(add);
  numV++;
  

  std::cout << "current list of vertices: ";
  for (auto *v :verts) {
    std::cout << v->label << " ";
  }
  std::cout << endl << "numV= " << numV << endl;

  return true; 
}

/** return true if vertex already in graph */
bool Graph::contains(const string &label) const { 
  for (auto *v : verts) {
    if (v->label == label) {
      return true;
    }
  }
  return false; 
}

// @return string representing edges and weights, "" if vertex not found
// A-3->B, A-5->C should return B(3),C(5)
string Graph::getEdgesAsString(const string &label) const { 
  for (auto *v : verts) {
    if (v->label == label) {
      Vertex::Edge *cur = v->next;
      stringstream ss;
      while (cur !=nullptr) {
        ss << cur->toVert->label << "(" << cur->weight << ")";
        if (cur->next != nullptr) {
          ss << ",";
        }
        cur = cur->next;
      }
      cout << ss.str() << endl;
      return ss.str();
    }
  }
  return ""; 
}

// @return true if successfully connected
bool Graph::connect(const string &from, const string &to, int weight) {
  if (!contains(from)) {
    add(from);
  }
  if (!contains(to)) {
    add(to);
  }
  if (from == to) {
    return false;
  }

  Vertex *connectTo;
  for (auto *v : verts) {
    if (v-> label == to) {
      connectTo = v;
    }
  }

  for (auto *v : verts) {
    if (v->label == from) {
      if (v->next == nullptr) {
        v->next = new Vertex::Edge;
        v->next->toVert = connectTo;
        v->next->weight = weight;
        cout << "connected " << v->label << " and " << v->next->toVert->label << " with weight: " << v->next->weight << endl;
        
        if (!directional) {
          cout << "graph is not directional; making reverse connection" << endl;
          connect(to, from, weight);
        }
        return true;
      }
      if (v->next != nullptr && v->next->toVert->label.compare(to) > 0) {
        Vertex::Edge *insert = new Vertex::Edge;
        insert->toVert = connectTo;
        insert->weight = weight;
        insert->next = v->next;
        v->next = insert;
        cout << "connected " << v->label << " and " << v->next->toVert->label << " with weight: " << v->next->weight << endl;
        //cout << "inserted " << to << " before " << cur->next->next->toVert->label << endl;
        if (!directional) {
          cout << "graph is not directional; making reverse connection" << endl;
         connect(to, from, weight);
        }
        return true;
      }

      Vertex::Edge *cur = v->next;
      while (cur != nullptr) {
        //cout << "traversing: found edge between " << v->label << " and " << cur->toVert->label << endl;
        if (cur->toVert == connectTo) {
          std::cout << from << " is already connected to " << to << endl;
          return false;
        }
        if (cur->next != nullptr && cur->next->toVert->label.compare(to) > 0) {
          Vertex::Edge *insert = new Vertex::Edge;
          insert->toVert = connectTo;
          insert->weight = weight;
          insert->next = cur->next;
          cur->next = insert;
          cout << "connected " << v->label << " and " << cur->next->toVert->label << " with weight: " << cur->next->weight << endl;
          //cout << "inserted " << to << " before " << cur->next->next->toVert->label << endl;
          if (!directional) {
            cout << "graph is not directional; making reverse connection" << endl;
            connect(to, from, weight);
          }
          return true;
        }
        if (cur->next == nullptr) {
          cur->next = new Vertex::Edge;
          cur->next->toVert = connectTo;
          cur->next->weight = weight;
          cout << "connected " << v->label << " and " << cur->next->toVert->label << " with weight: " << cur->next->weight << endl;
          if (!directional) {
          cout << "graph is not directional; making reverse connection" << endl;
          connect(to, from, weight);
          }
          return true;
        }
        cur = cur->next;
      }
    }
  }
  
  
  return false;
}

bool Graph::disconnect(const string &from, const string &to) { 
  if (!contains(from) || !contains(to)) {
    return false;
  }
  for (auto *v : verts) {
    if (v->label == from) {
      Vertex::Edge *cur = v->next;
      Vertex::Edge *prev;
      while (cur != nullptr) {
        if (cur->toVert->label == to) {
          //if it's the first and only edge from vertex v, just delete edge
          if (v->next == cur && cur->next == nullptr) {
            delete cur;
            if (!directional) {
              disconnect(to, from);
            }
            return true;
          }
          if (v->next != cur && cur->next != nullptr ) {
            prev->next = cur->next;
            delete cur;
            if (!directional) {
              disconnect(to, from);
            }
            return true;
          }
        }
        else {
          prev = cur;
          cur = cur->next;
        }
      }
      std::cout << "there is no edge connecting " << from << " and " << to << endl;
      return false;
    }
  }
  return false; 
}

// depth-first traversal starting from given startLabel
void Graph::dfs(const string &startLabel, void visit(const string &label)) {
  if (!contains(startLabel)) {
    std::cout << "Does not contain " << startLabel << endl;
    return;
  }
  stack<Vertex*> s;
  Vertex::Edge *cur;
  for (auto *v : verts) {
    if (v->label == startLabel) {
      s.push(v);
      visit(s.top()->label);
      v->visited = true;
    }
  }
  while (!s.empty()) {
    cur = s.top()->next;
    while (cur != nullptr) {
      if (!cur->toVert->visited) { 
        break;
      }
      cur = cur->next;
    }
    if(cur == nullptr) {
      
      s.pop();
    }
    else {
      s.push(cur->toVert);
      visit(s.top()->label);
      s.top()->visited = true;
    }
  }
  //reset visited fields to false. there is definitely a better way to do this.
  for (auto *v : verts) {
    v->visited = false;
  }
}

// breadth-first traversal starting from startLabel
void Graph::bfs(const string &startLabel, void visit(const string &label)) {
  if (!contains(startLabel)) {
    std::cout << "Does not contain " << startLabel << endl;
    return;
  }

  queue<Vertex*> vq;
  Vertex::Edge *cur;
  for (auto *v : verts) {
    if (v->label == startLabel) {
      vq.push(v);
      v->visited = true;
    }
  }
  while (!vq.empty()) {
    //visit then dequeue
    visit(vq.front()->label);
    cur = vq.front()->next;
    vq.pop();

    //iterate list, finding all unvisited adjacent vertices and enqueueing them
    while (cur != nullptr) {
      if (!cur->toVert->visited) { 
        cur->toVert->visited = true;
        vq.push(cur->toVert);
      }
      cur = cur->next;
    }
  }
  //reset visited fields to false. there is definitely a better way to do this.
  for (auto *v : verts) {
    v->visited = false;
  }
}

// store the weights in a map
// store the previous label in a map
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &startLabel) const {
  map<string, int> weights;

  // <label of current edge, label of previous>
  map<string, string> previous;
  set<Vertex*>vSet;
  set<Vertex*>::iterator it;

  Vertex *vStart;
  Vertex::Edge *cur;

  int iterations {0}; //temporary

  if (!contains(startLabel)) {
    std::cout << "Does not contain " << startLabel << endl;
    return make_pair(weights, previous);
  }
  //setting initial vertex set
  for (auto *v : verts) {
    if (v->label == startLabel) {
      vSet.insert(v);
      cout << "Inserting startLabel '" << startLabel << "' into vSet" << endl;
      vStart = v;
    }
  }
  //setting initial weights
  for ( Vertex *v : vSet) {
    cur = v->next;
    while (cur != nullptr) {
      weights.insert(pair<string, int>(cur->toVert->label, cur->weight));
      previous.insert(pair<string, string>(cur->toVert->label, v->label));
      cur = cur->next;
    }
  }
  cout << "initial weights: ";
  for (auto a : weights) {
    cout << "(" <<  a.first << "," << a.second << ")" ;
  }
  cout << endl;

  //completes(?) when all vertices have been added to the set
  while (iterations <= verts.size()) {
    Vertex *vAdd;
    int minWeight INT16_MAX;
    
     //start from the top each iteration
    
    cout << "starting from the top, vertex: " << vStart->label << ". (should equal: " << startLabel << ")" << endl;
    //finding vertex with lowest weight NOT in vSet and adding it
    for (auto *v : vSet) {
      cur = v->next;
      while (cur != nullptr) {
      // vSet.find(cur->toVert) finds a vertex in the set and returns pointer. 
        if (cur->toVert == vStart ) {
          cur = cur->next;
          continue;
        }
        if (weights[cur->toVert->label] < minWeight && *vSet.find(cur->toVert) != cur->toVert) {
          minWeight = cur->weight;
        
          vAdd = cur->toVert;
        }
        cur = cur->next;
      }
    }
    
    //add lowest weight vertex not in vAdd to the set
    vSet.insert(vAdd);
    iterations ++;

    cout << "current vSet: ";
    for (auto *a : vSet) {
      cout << a->label << " ";
    }
    cout << endl;

    //starting from the added vertex, search for adjacent ones
    cur = vAdd->next;
    while (cur != nullptr) {
      cout << "comparing path lengths..." << endl;
      if (cur->toVert == vStart ) {
        cur = cur->next;
        continue;
      }
      // using adjacency list, so vertices not adjacent to start will not be present in weights until a path is found.
      if (weights.count(cur->toVert->label) == 0 ) {
        cout << " vertex: " << cur->toVert->label << " not connected to start, adding to weights";
        weights.insert(pair<string, int> (cur->toVert->label, cur->weight + weights[vAdd->label]));
        //connected to added vertex, not to starting vertex
        cout << "and connecting to " << vAdd->label << " in previous" << endl;
        previous.insert(pair<string, string>(cur->toVert->label, vAdd->label));
      }
      // going via cur->toVert is a shorter path than in weights and isn't in vSet
      else if (cur->weight + weights.at(vAdd->label) < weights.at(cur->toVert->label) && *vSet.find(cur->toVert) != cur->toVert) {
        weights[cur->toVert->label] = cur->weight + weights[vAdd->label];
        previous[cur->toVert->label] = vAdd->label;
      }
      //int pathLength = cur->weight + weights[vAdd->label];
      
      cur = cur->next;
    }
    cout << "minWeight is now: " << minWeight << endl;
  }
  
  return make_pair(weights, previous);
}

// minimum spanning tree using Prim's algorithm
int Graph::mstPrim(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const {
  if (!contains(startLabel)) {
    std::cout << "Does not contain " << startLabel << endl;
    return -1;
  }
  if (directional) {
    cout << "Prim's algorithm does not work with directed graphs" << endl;
    return -1;
  }
  vector<Vertex*> primv;
  Vertex *fromv;
  //pointer for edge to add to mst
  
  //pointer to edge for iteration
  Vertex::Edge *cur {nullptr};
 
  int minPath = 0;
  
  for (auto *v : verts) {
    if (v->label == startLabel) {
      v->visited = true;
      primv.push_back(v);
    }
  }
  while (primv.size() < verts.size()) {
    Vertex::Edge *edge;
    int minEdge = 0;
    for (auto *v : primv) {
      cur = v->next;
      cout << "checking vertex " << v->label << endl;
      while (cur != nullptr) {
        cout << "checking edge to " << cur->toVert->label << "with weight " << cur->weight << endl;
        if ((cur->weight < minEdge || minEdge == 0) && !cur->toVert->visited) {
          minEdge = cur->weight;
          edge = cur;
          fromv = v;
          cout << "new min edge is " << minEdge << " from " << v->label << " to " << edge->toVert->label << endl;
        }
        cur = cur->next;
      }
    }
    if (!edge->toVert->visited) {
      edge->toVert->visited = true;
      visit(fromv->label, edge->toVert->label, edge->weight);
      for (auto *v : verts) {
        if (v->label == fromv->label) {
           primv.push_back(edge->toVert);
        }
      }
      cout << primv.size() << endl;
      minPath += edge->weight;
      cout << "min path length: " << minPath << endl;
    }
    else {
      break;
    }
  }
  

  //reset visited fields to false. there is definitely a better way to do this.
  for (auto *v : verts) {
    v->visited = false;
  }
  return minPath;
}

// minimum spanning tree using Kruskal's algorithm
//int Graph::mstKruskal(void visit(const string &from, const string &to,
//                                 int weight)) const {
//  return -1;
//}

// read a text file and create the graph
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