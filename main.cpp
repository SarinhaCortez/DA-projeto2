//#include "DataParser.h"
#include "DataParserMatrix.h"
#include "Utils.h"
#include <iostream>

int main(){

    MGraph g;

    Parser("stadiums.csv", g);
    for(int i = 0; i <= g.getNumVertex(); i++){
        for(auto el : g.getAdj(i)){
            cout << el << " ";
        }
        cout << endl;
    }
/*
    for(auto v : (*g.getDistMatrix())){
        for(auto d : v){
            cout << d <<" ";
        }cout << endl;
    }*/


    /*
    Vertex<int> * initial;
    for(auto v: g.getVertexSet()){
        v->setVisited(false);
        if(v->getInfo()==0){
            initial=v;
            initial->setVisited(true);
        }
    }

    /*for (auto v:g.getVertexSet()){
        for(auto edge: v->getAdj() ){
            cout<<edge->getOrig()->getInfo()<<"----->"<<edge->getDest()->getInfo()<<endl;
        }
    }
    std::vector<Vertex<int> *> tsp;
    std::vector<Vertex<int> *> final;
    int numVertex = g.getNumVertex();

    tsp.push_back(initial);
    int dist = 0;
    int minDist = INT_MAX;
    bool found=false;
    if(BacktrakingTSP(g, dist, minDist, tsp, final, numVertex, initial, found)){
        for(auto vertex: final){
            cout<<vertex->getInfo()<<endl;
        }
    }
    else{
        cout<<"Ta mal lol"<<endl;
    }

        //Parser("edges_25.csv", g);
    //Parser("graph1", g);
    /*
    cout <<"Edges"<<endl;

    for(auto v : g.getVertexSet()){
        for(auto e : v->getAdj()){
            cout << e->getWeight()<<endl;
        }
    }

    cout <<"Nodes" <<endl;
    for(auto v : g.getVertexSet()){
        cout << v->getInfo()<<endl;
    }
    */
}