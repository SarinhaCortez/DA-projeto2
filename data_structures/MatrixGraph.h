#ifndef DA_PROJETO2_MATRIXGRAPH_H
#define DA_PROJETO2_MATRIXGRAPH_H

#include <vector>
#include <unordered_map>
#include <utility>
class MGraph {
    std::vector<std::vector<double>>* distMatrix; // Change to pointer
    std::unordered_map<int, std::pair<double, double>> vertexSet; // First is latitude, second is longitude
    std::unordered_map<int, bool> visitedInfo; // First is latitude, second is longitude
    int numVertex;
public:
    MGraph(): distMatrix(new std::vector<std::vector<double>>()), numVertex(0) {} // Initialize with empty vector
    MGraph(int n) : distMatrix(new std::vector<std::vector<double>>(n + 1, std::vector<double>(n + 1, -1))), numVertex(n + 1) {}



    int getNumVertex(){
        return numVertex;
    }

    void setVisited(int n, bool v){
        visitedInfo[n] = v;
    }

    bool isVisited(int n){
        return visitedInfo[n];
    }
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
            vertexSet[a] = std::make_pair(b,c);
        }


    int getWeight(int origin, int dest){
        return (*distMatrix)[origin][dest];
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