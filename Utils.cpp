#include "Utils.h"
#include "data_structures/MatrixGraph.h"
#include <vector>
#include <iostream>
#include <random>
#include <climits>
#include <algorithm>

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
    int totalDist=0;

    while(count<matrix->size()-1){
        currentRow = (*matrix)[initialVertex];

        pair<int, int> minDistance = minDist(currentRow, tsp);
        tsp.push_back(minDistance.second);
        totalDist+=minDistance.first;

        if(minDistance.first == INT_MAX){
            cout <<"nao existe lol"<<endl;
            return false;
        }

        //cout<< "minDist: "<<minDistance.first <<" ->> totalDist: "<<totalDist<<endl;

        count++;
        initialVertex=minDistance.second;
    }

    tsp.push_back(0);
    totalDist+=(*matrix)[0][initialVertex];
    for(auto v: tsp){
        cout<<v<<endl;
    }
    cout<<"Total min Dist: "<<totalDist<<endl;
    return true;


}

void preOrder(vector<Vertex<int> *> vertexSet, Vertex<int>* v0, vector<Vertex<int>*>& tour){
v0->setVisited(true);
tour.push_back(v0);
for(auto e: v0->getAdj()){
auto v2 = e->getDest();
if (!v2->isVisited()){
preOrder(vertexSet, v2, tour);
tour.push_back(v2);
}
}
return;
}



vector<Vertex<int> *> prim(Graph<int>& g) {
    // Check if the graph is empty
    if (g.getVertexSet().empty()) {
        return g.getVertexSet();
    }

    for(auto v : g.getVertexSet()) {
        v->setDist(INF);
        v->setPath(nullptr);
        v->setVisited(false);
    }

    Vertex<int>* s = g.findVertex(0);
    s->setDist(0);

    MutablePriorityQueue<Vertex<int>> q;
    q.insert(s);

    while( ! q.empty() ) {

        auto v = q.extractMin();
        v->setVisited(true);

        for(auto &e : v->getAdj()) {
            Vertex<int>* w = e->getDest();
            if (!w->isVisited()) {
                auto oldDist = w->getDist();
                cout<<"dist:"<<w->getDist()<<endl;
                cout<<"weight"<<e->getWeight()<<endl;
                if(e->getWeight() < oldDist) {
                    cout<<"entrei"<<endl;
                    w->setDist(e->getWeight());
                    cout<<"new dist"<<w->getDist()<<endl;
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
    vector<Vertex<int> *> res;
    Vertex<int> * initial;
    for(auto v:g.getVertexSet()){
        if(v->getInfo()==0){
            initial=v;
        }
    }
    res.push_back(initial);
    while(res.size()!=g.getNumVertex()) {
        for (auto v: g.getVertexSet()) {
            if ( v->getInfo()!=0 && v->getPath()->getOrig()->getInfo() == res.back()->getInfo() ) {
                res.push_back(v);
            }
        }
    }
    res.push_back(initial);
    return res;
}

double calculateTourDistance(const vector<Vertex<int>*>& tour) {
double totalDistance = 0.0;

for (int x = 0; x < tour.size() - 1; ++x) {

for (auto& e : tour[x]->getAdj()) {
if (e->getDest() == tour[x + 1]) {
totalDistance += e->getWeight();
break;
}
}
}
totalDistance += tour.back()->getAdj()[0]->getWeight();
return totalDistance;
}


void triangularApproximation(Graph<int> &g) {

    auto mst = prim(g);
    vector<Vertex<int>*> tour;
    for(auto x: mst){
        cout<<x->getInfo()<<endl;
    }
    for(auto x: mst){
        x->setVisited(false);
    }
    preOrder(mst, mst[0], tour);

    double tourDistance = calculateTourDistance(tour);
    cout << "2 -Approximated distance: " << tourDistance << endl;

}