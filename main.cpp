#include "DataParser.h"

int main(){

    Graph<int> g;
    Parser("tourism.csv", g);
    //Parser("edges_25.csv", g);
    //Parser("graph1", g);
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
}