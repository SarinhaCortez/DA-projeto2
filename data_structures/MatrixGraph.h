#ifndef DA_PROJETO2_MATRIXGRAPH_H
#define DA_PROJETO2_MATRIXGRAPH_H

#include <vector>
#include <unordered_map>
#include <utility>

class MGraph {
    std::vector<std::vector<double>>* distMatrix; // Change to pointer
    std::unordered_map<int, std::pair<double, double>> vertexSet; // First is latitude, second is longitude
public:
    MGraph(): distMatrix(new std::vector<std::vector<double>>()) {} // Initialize with empty vector
    MGraph(int n) : distMatrix(new std::vector<std::vector<double>>(n, std::vector<double>(n, -1))) {}

    // Destructor to free memory
    ~MGraph() {
        delete distMatrix;
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
    }

    void addVertex(int a,double b, double c){
        vertexSet.emplace(a, std::make_pair(b,c));
    }

    std::unordered_map<int, std::pair<double, double>> getVertexSet(){
        return vertexSet;
    }

    double getLatitude(int n){
        return vertexSet[n].first;
    }

    double getLongitude(int n){
        return vertexSet[n].second;
    }
};

#endif //DA_PROJETO2_MATRIXGRAPH_H
