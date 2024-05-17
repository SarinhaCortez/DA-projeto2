#include "DataParser.h"
#include "Graph.h"
//#include "DataParserMatrix.h"
#include "Utils.h"
#include "menu.h"
#include <iostream>
#include <set>

int main(){
    //Menu menu;
    //menu.openMenu();
    Graph<int> g;
    //ToyGraphParser("tourism.csv", g);
    //ExtraMSGraphParser("edges_900.csv", g);
    RealWorldGraphParser("graph3",g);
    cout << "hello" << endl;
    RealWorldFullyConnected(g);
    cout << "its me" << endl;
    triangularApproximation(g);

      /* Menu menu;
        menu.openMenu();*/

    /*MGraph g;

    Parser("tourism.csv", g);
   /* for(int i = 0; i < g.getNumVertex(); i++){
        for(auto el : g.getAdj(i)){
            cout << el << " ";
        }
        cout << endl;
    }*/

    /*std::vector<std::vector<double>>* matrix = g.getDistMatrix();
    std::vector<int> tsp;
    tsp.push_back(0);
    nearestNeighbour(g, tsp, matrix);

   for(int i=0; i<g.getNumVertex(); i++){
        for(auto j=0; j<g.getNumVertex(); j++){
            cout << (*matrix)[i][j]<< "  ";
        }
        cout << endl;
    }


    /*for(auto v : (*g.getDistMatrix())){
        for(auto d : v){
            cout << d <<" ";
        }cout << endl;
    }*/

    //Parser("tourism.csv", g);
    /*for(int i = 0; i < g.getNumVertex(); i++){
        for(auto el : g.getAdj(i)){
            cout << el << "  ";
        }
        cout << endl;
    }*/
/*
    int initial =0;
    std::vector<int> tspB;
    std::vector<int> final;
    int numVertex=g.getNumVertex();
    tspB.push_back(initial);
    int dist=0;
    int minDist=INT_MAX;
    bool found=false;

    std::set<int>visited;

    if(BacktrakingTSP(g, dist, minDist, tspB, final, numVertex, initial, found, visited)){
        for(auto vertex: final){
            cout<<vertex<<endl;
        }
    }
    else{
        cout<<"nao existe lol"<<endl;
    }

    for(auto v: g.getVertexSet()){
        cout<<v.first<<"--->"<<v.second.first<<"------->"<<v.second.second<<endl;
    }

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

    //new heutistic

 /*   cout<<g.getVertexSet().size()<<endl;

    g.addVertex(0,-43.19448871895534,-22.983571669284053);
    g.addVertex(1,-43.30181022730023,-23.011972156360795);
    g.addVertex(2,-43.21636554138819,-22.920250516077687);
    g.addVertex(3,-43.32304806953678,-23.006841073845603);
    g.addVertex(4,-43.33148035581998,-22.946701029022186);
    g.addVertex(5,-43.35081315905275,-22.802312213714195);
    g.addVertex(6,-43.28851640479031,-22.668590460956885);
    g.addVertex(7,-43.354511013073946,-22.884016303007538);
    g.addVertex(8,-43.43929968461572,-22.878464383431382);
    g.addVertex(9,-43.01808331006573,-22.809200330634177);
    g.addVertex(10,-43.33518713997461,-22.93484993621862);
    g.addVertex(11,-43.4263483168707,-22.711041021168743);

    for(int i = 0; i < g.getNumVertex(); i++){
        for(auto el : g.getAdj(i)){
            cout << el << " ";
        }
        cout << endl;
    }

    cout<<g.getVertexSet().size()<<endl;
    vector<pair<int, std::pair<double, double>>> data;
    for(auto v : g.getVertexSet()){
        //cout<<v.first<<"---->"<<v.second.first<<"----->"<<v.second.second<<endl;

        data.push_back(make_pair(v.first, make_pair(v.second.first, v.second.second)));
    }

    int k = 4; // Número de clusters
    int maxIterations = 100; */

    //kmeans(data, k, maxIterations);



}
