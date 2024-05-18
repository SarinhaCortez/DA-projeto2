#ifndef WORLD_GRAPH
#define WORLD_GRAPH

#include <iostream>
#include <vector>
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include "../data_structures/MutablePriorityQueue.h"

#define INF std::numeric_limits<double>::max()

/************************* Forward Declarations **************************/

class Edge;
class Vertex;

/************************* Vertex  **************************/

class Vertex {
public:
    explicit Vertex(int in);
    bool operator<(const Vertex & vertex) const; // required by MutablePriorityQueue
    int getInfo() const;
    const std::unordered_set<Edge*>& getAdj() const;
    bool isVisited() const;
    double getDist() const;
    Edge* getPath() const;

    void setVisited(bool visited);
    void setDist(double dist);
    void setPath(Edge *path);
    Edge* addEdgeV(Vertex *dest, double w);
    void setLat(double lat);
    void setLong(double longit);
    double getLat() const;
    double getLong() const;

    friend class MutablePriorityQueue<Vertex>;

protected:
    int info;
    std::unordered_set<Edge*> adj;
    bool visited = false; // used by DFS, BFS, Prim ...
    double dist = 0;
    Edge *path = nullptr;
    int queueIndex = 0; // required by MutablePriorityQueue and UFDS
    double latitude = 0.0, longitude = 0.0; // for non-toy graphs
};

/********************** Edge  ****************************/

class Edge {
public:
    Edge(Vertex *orig, Vertex *dest, double w);
    Vertex* getDest() const;
    double getWeight() const;

protected:
    Vertex *orig; // origin vertex
    Vertex *dest; // destination vertex
    double weight; // edge weight, can also be used for capacity
};

/********************** Graph  ****************************/

class Graph {
public:
    Vertex* findVertex(int in) const;
    bool addVertex(int in);
    Edge* findEdge(int s, int d) const;
    bool addEdge(int sourc, int dest, double w);
    const std::unordered_map<int, Vertex*>& getVertexSet() const;

protected:
protected:
    //std::vector<Vertex*> vertexSet;
    std::unordered_map<int, Vertex*> vertexSet;
    double** distMatrix = nullptr; // dist matrix for Floyd-Warshall
    int** pathMatrix = nullptr; // path matrix for Floyd-Warshall

    int findVertexIdx(int in) const;
};

#endif // WORLD_GRAPH
