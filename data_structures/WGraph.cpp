#include "WGraph.h"


Vertex::Vertex(int in) : info(in) {}

Edge* Vertex::addEdgeV(Vertex *d, double w) {
    auto newEdge = new Edge(this, d, w);
    adj.insert(newEdge);
    return newEdge;
}

bool Vertex::operator<(const Vertex & vertex) const {
    return this->dist < vertex.dist;
}

int Vertex::getInfo() const {
    return this->info;
}

void Vertex::setLat(double lat) {
    this->latitude = lat;
}

void Vertex::setLong(double longit) {
    this->longitude = longit;
}

double Vertex::getLat() const {
    return latitude;
}

double Vertex::getLong() const {
    return longitude;
}

const std::unordered_set<Edge*>& Vertex::getAdj() const {
    return this->adj;
}

bool Vertex::isVisited() const {
    return this->visited;
}

double Vertex::getDist() const {
    return this->dist;
}

Edge* Vertex::getPath() const {
    return this->path;
}

void Vertex::setVisited(bool visited) {
    this->visited = visited;
}

void Vertex::setDist(double dist) {
    this->dist = dist;
}

void Vertex::setPath(Edge *path) {
    this->path = path;
}

/********************** Edge  ****************************/

Edge::Edge(Vertex *orig, Vertex *dest, double w) : orig(orig), dest(dest), weight(w) {}

Vertex* Edge::getDest() const {
    return this->dest;
}

double Edge::getWeight() const {
    return this->weight;
}

/********************** Graph  ****************************/

const std::unordered_map<int, Vertex*>& Graph::getVertexSet() const {
    return vertexSet;
}

Vertex* Graph::findVertex(int in) const {
    auto it = vertexSet.find(in);
    if(it != vertexSet.end())
        return it->second;
    return nullptr;
}

bool Graph::addVertex(int in) {
    auto it = vertexSet.find(in);
    if (it != vertexSet.end())
        return false;
    vertexSet.emplace(in, new Vertex(in));
    return true;
}

bool Graph::addEdge(int sourc, int dest, double w) {
    auto s = vertexSet.find(sourc);
    auto d = vertexSet.find(dest);

    if (s == vertexSet.end() || d == vertexSet.end())
        return false;
    auto src = s->second;
    auto dst =d->second;
    src->addEdgeV(dst, w);
    return true;
}

