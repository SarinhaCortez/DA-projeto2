#ifndef DA_PROJETO2_MATRIXGRAPH_H
#define DA_PROJETO2_MATRIXGRAPH_H

#include <vector>
#include <unordered_map>
#include <utility>

class Vertex{
    int latitude = 0;
    int longitude = 0;
    bool visited = false;

public:
    double getLatitude(){
        return latitude;
    }

    double getLongitude(){
        return longitude;
    }
    bool isVisited(){
        return visited;
    }
    void setVisited(bool v){
        visited = v;
    }

    Vertex(){}
    Vertex(int lat, int lon): latitude(lat), longitude(lon){}
};


class MGraph {
    std::vector<std::vector<double>>* distMatrix; // Change to pointer
    std::unordered_map<int, Vertex> vertexSet; // First is latitude, second is longitude
    int numVertex;
public:
    MGraph(): distMatrix(new std::vector<std::vector<double>>()), numVertex(0) {} // Initialize with empty vector
    MGraph(int n) : distMatrix(new std::vector<std::vector<double>>(n + 1, std::vector<double>(n + 1, -1))), numVertex(n + 1) {}

    // Destructor to free memory
    ~MGraph() {
        delete distMatrix;
    }

    int getNumVertex(){
        return numVertex;
    }

    // Return a reference to the distMatrix to make it mutable
    std::vector<std::vector<double>>* getDistMatrix() {
        return distMatrix;
    }

    // Update the dimensions of the distMatrix
    void updateMatrixDim(int n) {
        distMatrix->resize(n+1);
        for (auto& row : *distMatrix) {
            row.resize(n+1, -1);
        }
        numVertex = n + 1;
    }

    void addVertex(int a,double b, double c){
        vertexSet[a] = Vertex(b,c);
    }

    std::unordered_map<int, Vertex> getVertexSet(){
        return vertexSet;
    }

    double getLatitude(int n){
        return vertexSet[n].getLatitude();
    }

    double getLongitude(int n){
        return vertexSet[n].getLongitude();
    }
    std::vector<int> getAdj(int v){
        std::vector<int> res;
        for(int i = 0; i < numVertex; i++){
            if((*distMatrix)[v][i] < 0) continue;
            else res.push_back(i);
        }
        return res;
    }
};

#endif //DA_PROJETO2_MATRIXGRAPH_H
