#ifndef DA_PROJETO2_MATRIXGRAPH_H
#define DA_PROJETO2_MATRIXGRAPH_H

#include <vector>
#include <unordered_map>
#include <utility>

/*
class Vertex{
    int index, lat, longi;
public:
    Vertex(){index = -1;}
    Vertex(int n) {index = n;}
    int getlat() {return lat;}
    int getLongi() {return longi;}
};*/

class Graph {
    std::vector<std::vector<double>> distMatrix;
    std::unordered_map<int, std::pair<double, double>> vertexSet; //first is latitude, second is longitude
public:
    Graph(): distMatrix(0, std::vector<double>(0, -1)) {}
    Graph(int n) : distMatrix(n, std::vector<double>(n, -1)) {}
    std::vector<std::vector<double>> getDistMatrix(){
        return distMatrix;
    }
    void updateMatrixDim(int n){
        distMatrix.clear();
        distMatrix = std::vector<std::vector<double>>(n, std::vector<double>(n, -1));
    }
    void addVertex(int a,double b, double c){
        vertexSet.emplace(a, std::make_pair(b,c));
    }

};

#endif //DA_PROJETO2_MATRIXGRAPH_H
//trying to change this matrix thing