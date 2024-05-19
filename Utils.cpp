#include "Utils.h"
#include "data_structures/MatrixGraph.h"
#include <vector>
#include <iostream>
#include <random>
#include <climits>
#include <algorithm>
#include <unordered_set>

using namespace std;

using namespace std;


bool BacktrakingTSP(MGraph &g, int &dist, int &minDist, std::vector<int> &tsp, std::vector<int> &final, int &numVertex, int vertex, bool&found, std::set<int> &visited){

    bool final_found = false;
    if(vertex==0 && numVertex==0 && visited.find(vertex)!=visited.end()){

        if(dist<minDist){
            minDist=dist;
            final.clear();
            for(auto element: tsp){
                final.push_back(element);
            }


            found=true;
            return found;
        }

        return false;
    }

    else if((numVertex>0 && vertex!=0) || (visited.find(vertex)==visited.end() && vertex==0 && numVertex>0)) {

        for (auto dest: g.getAdj(vertex)) {

            if ((visited.find(dest)==visited.end() && dest!=0) || (visited.find(dest)==visited.end() && dest==0 && numVertex==1)) {
                visited.insert(dest);
                tsp.push_back(dest);

                int newNumVertex=numVertex - 1;
                int newDist=dist + g.getWeight(vertex, dest);

                if (BacktrakingTSP(g, newDist, minDist, tsp, final, newNumVertex, dest, found, visited)){

                    visited.erase(dest);
                    tsp.pop_back();
                    final_found=true;
                    /*dist = newDist-edge->getWeight();
                    numVertex++;*/
                }
                else{

                    visited.erase(dest);
                    tsp.pop_back();
                    /*dist = newDist-edge->getWeight();
                    numVertex++;*/
                    return found;}
            }
        }
    }
    else{

        return false;
    }
    visited.erase(vertex);

    return final_found;
}

//Nearest neighbour strategy

bool notPresent(int i, std::vector<int> &tsp){
    for(auto el: tsp){
        if(i == el){
            return false;
        }
    }
    return true;
}


pair<int, int> minDist(std::vector<double> currentRow, std::vector<int> &tsp){
    int min=INT_MAX;
    int minVertex;
    pair<int, int> res;

//se o indicw do minimo tiver no tsp nao da

    for(int i=0; i<currentRow.size(); i++){
        //cout << currentRow[i] <<"---->" << i <<endl;
        if(currentRow[i] >=0 && currentRow[i] <min && notPresent(i, tsp)){/* nao esta em tsp*/
            //cout <<"ENTREI"<<endl;
            min = currentRow[i];
            minVertex=i;
            //cout<<min<<"---->aa"<<minVertex<<endl;
        }
    }

    res.first = min;
    res.second=minVertex;

    return res;
}

bool nearestNeighbour(MGraph &g,  std::vector<int> &tsp, std::vector<std::vector<double>>* matrix){
    int initialVertex = 0;
    std::vector<double> currentRow;
    int count=0;
    double totalDist=0;

    while(count<matrix->size()-1){
        currentRow = (*matrix)[initialVertex];

        pair<int, int> minDistance = minDist(currentRow, tsp);
        tsp.push_back(minDistance.second);
        totalDist+=minDistance.first;

        if(minDistance.first == INT_MAX){
            cout <<"Path not found"<<endl;
            return false;
        }

        //cout<< "minDist: "<<minDistance.first <<" ->> totalDist: "<<totalDist<<endl;

        count++;
        initialVertex=minDistance.second;
    }

    tsp.push_back(0);
    totalDist+=(*matrix)[0][initialVertex];
    cout<<"Path: ";
    for(auto v: tsp){
        cout<<v<<" ";
    }
    cout<<endl;
    cout<<"Total min Dist: "<<totalDist<<endl;
    return true;


}



void preOrder(unordered_map<int, Vertex *> vertexSet, Vertex* v0, vector<Vertex *>& tour){
    v0->setVisited(true);
    tour.push_back(v0);
    for(auto e: v0->getAdj()){
        auto v2 = e->getDest();
        if (!v2->isVisited() and v2->getPath() == e){
            preOrder(vertexSet, v2, tour);
            tour.push_back(v2);
        }
    }
    return;
}


unordered_map<int, Vertex *> prim(Graph * g) {
    if (g->getVertexSet().empty()) {
        return g->getVertexSet();
    }

    for(auto v : g->getVertexSet()) {
        v.second->setDist(INF);
        v.second->setPath(nullptr);
        v.second->setVisited(false);
    }

    Vertex* s = g->getVertexSet().find(0)->second;
    s->setDist(0);
    MutablePriorityQueue<Vertex> q;
    q.insert(s);

    while( ! q.empty() ) {
        auto v = q.extractMin();
        v->setVisited(true);
        for(auto &e : v->getAdj()) {
            Vertex* w = e->getDest();
            if (!w->isVisited()) {
                auto oldDist = w->getDist();
                if(e->getWeight() < oldDist) {
                    w->setDist(e->getWeight());
                    w->setPath(e);
                    if (oldDist == INF) {
                        q.insert(w);
                    }
                    else {
                        q.decreaseKey(w);
                    }
                }
            }
        }
    }

    return g->getVertexSet();
}

double calculateTourDistance(const vector<Vertex*>& tour) {
    double totalDistance = 0.0;
    Vertex* last;

    for (int x = 0; x < tour.size() - 1; ++x) {
        for (auto &e: tour[x]->getAdj()) {
            if (e->getDest() == tour[x + 1]) {
                last = tour[x+1];
                totalDistance += e->getWeight();
                break;
            }
        }
    }
    double tozero;
    for(auto x: last->getAdj()){
        if(x->getDest()->getInfo() == 0){
            tozero = x->getWeight();
        }
    }
    totalDistance += tozero;
    return totalDistance;
}

vector<Vertex*> removeDup(vector<Vertex*> tour){
    unordered_set<Vertex *> seen;
    vector<Vertex *> result;
    for (const auto& elem : tour) {
        if (seen.find(elem) == seen.end()) {
            seen.insert(elem);
            result.push_back(elem);
        }
    }
    return result;
}



void triangularApproximation(Graph  &g) {
    auto mst = prim(&g);

    vector<Vertex *> tour;
    for(auto x: mst){
        x.second->setVisited(false);
    }
    preOrder(mst, mst[0], tour);

    //tour.push_back(g.findVertex(0));

    tour = removeDup(tour);cout << endl;
    double tourDistance = calculateTourDistance(tour);
    cout << "Minimum distance following this algorithm: " << tourDistance << endl;

}

double convert_to_radians(double coord){
    return coord *M_PI/180;
}

double harversineDistance(Vertex * v1, Vertex * v2){

    double lon1, lat1, lon2, lat2;
    double earthradius = 6371000; // meters

    lon1 = v1->getLong();
    lat1 = v1->getLat();
    lon2=v2->getLong();
    lat2= v2->getLat();

    double rad_lat1 = convert_to_radians(lat1);
    double rad_lon1 = convert_to_radians(lon1);
    double rad_lat2 = convert_to_radians(lat2);
    double rad_lon2 = convert_to_radians(lon2);

    double delta_lat = rad_lat2 - rad_lat1;
    double delta_lon = rad_lon2 - rad_lon1;

    double a = sin(delta_lat / 2) * sin(delta_lat / 2) +
               cos(rad_lat1) * cos(rad_lat2) *
               sin(delta_lon / 2) * sin(delta_lon / 2);
    double c = 2.0 * atan2(sqrt(a), std::sqrt(1.0 - a));

    double distance = earthradius * c;

    return distance;
}


void RealWorldFullyConnected(Graph &g) {
    int nEdges = g.getVertexSet().size() - 1;

    std::unordered_map<Vertex*, std::unordered_set<Vertex*>> adjacencyMap;
    for (auto &v : g.getVertexSet()) {
        for (auto e : v.second->getAdj()) {
            adjacencyMap[v.second].insert(e->getDest());
        }
    }

    for (auto &v1 : g.getVertexSet()) {
        auto vertex1 = v1.second;
        if (adjacencyMap[vertex1].size() == nEdges) {
            continue;
        }

        for (auto &v2 : g.getVertexSet()) {
            if (v1 == v2) {
                continue;
            }
            auto vertex2 = v2.second;
            if (adjacencyMap[vertex1].find(vertex2) == adjacencyMap[vertex1].end()) {
                double dist = harversineDistance(vertex1, vertex2);
                g.addEdge(vertex1->getInfo(), vertex2->getInfo(), dist);
                g.addEdge(vertex2->getInfo(), vertex1->getInfo(), dist);

                adjacencyMap[vertex1].insert(vertex2);
                adjacencyMap[vertex2].insert(vertex1);
            }
        }
    }
}