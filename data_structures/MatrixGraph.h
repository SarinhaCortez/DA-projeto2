#ifndef DA_PROJETO2_MATRIXGRAPH_H
#define DA_PROJETO2_MATRIXGRAPH_H

#include <vector>
#include <unordered_map>
#include <utility>

/**
 * @brief A class representing a graph using an adjacency matrix.
 *
 * This class represents a graph using an adjacency matrix to store edge distances between vertices.
 * It also stores vertex information, such as latitude and longitude, and whether vertices have been visited.
 */
class MGraph {
    /**< Pointer to the adjacency matrix storing distances */
    std::vector<std::vector<double>>* distMatrix;
     /**< Map storing vertex information: (vertex ID, (latitude, longitude)) */
    std::unordered_map<int, std::pair<double, double>> vertexSet; // First is latitude, second is longitude
    /**< Map storing information about visited vertices */
    std::unordered_map<int, bool> visitedInfo; // First is latitude, second is longitude
    int numVertex;
public:
    /**
     * @brief Default constructor for the MGraph class.
     *
     * Initializes the adjacency matrix with an empty vector.
     */
 MGraph(): distMatrix(new std::vector<std::vector<double>>()), numVertex(0) {} // Initialize with empty vector

 /**
* @brief Constructor for the MGraph class.
*
* Initializes the adjacency matrix with dimensions based on the number of vertices.
*
* @param n The number of vertices in the graph.
*/
 MGraph(int n) : distMatrix(new std::vector<std::vector<double>>(n + 1, std::vector<double>(n + 1, -1))), numVertex(n + 1) {}


 /**
* @brief Getter method for the number of vertices in the graph.
*
* @return The number of vertices in the graph.
*/
 int getNumVertex(){
     return numVertex;
 }

 /**
* @brief Setter method to the visited field.
*
* @param n The ID of the vertex.
* @param v Bool value indicating whether the vertex is visited or not.
*/
 void setVisited(int n, bool v){
     visitedInfo[n] = v;
 }

/**
* @brief Getter method to check if a vertex is visited.
*
* @param n The ID of the vertex.
* @return True if the vertex is visited, false otherwise.
*/
 bool isVisited(int n){
     return visitedInfo[n];
 }

 /**
* @brief Getter method for the adjacency matrix.
*
* @return A pointer to the adjacency matrix.
*/
 std::vector<std::vector<double>>* getDistMatrix() {
         return distMatrix;
     }

    /**
    * @brief Updates the dimensions of the adjacency matrix.
    *
    * Resizes the adjacency matrix to the number os vertexs.
    *
    * @param n The new number of vertexs in the graph.
    */
     void updateMatrixDim(int n) {
         distMatrix->resize(n+1);
         for (auto& row : *distMatrix) {
             row.resize(n+1, -1);
         }
         numVertex = n + 1;
     }

    /**
     * @brief Adds a vertex to the graph with specified latitude and longitude.
     *
     * @param a The ID of the vertex.
     * @param b The latitude of the vertex.
     * @param c The longitude of the vertex.
     */
     void addVertex(int a,double b, double c){
         vertexSet[a] = std::make_pair(b,c);
     }

/**
     * @brief Gets the weight of the edge between two vertices.
     *
     * @param origin The ID of the origin vertex.
     * @param dest The ID of the destination vertex.
     * @return The weight of the edge between the specified vertices.
     */
 int getWeight(int origin, int dest){
     return (*distMatrix)[origin][dest];
 }



    /**
     * @brief Getter method for the vertex set.
     *
     * @return The vertexSet.
     */
     std::unordered_map<int, std::pair<double, double>> getVertexSet(){
         return vertexSet;
     }

    /**
        * @brief Getter for the latitude of a vertex.
        *
        * @param n The ID of the vertex.
        * @return The latitude of the vertex.
        */
     double getLatitude(int n){
         return vertexSet[n].first;
     }

    /**
    * @brief Getter for the longitude of a vertex.
    *
    * @param n The ID of the vertex.
    * @return The longitude of the vertex.
    */
     double getLongitude(int n){
         return vertexSet[n].second;
     }

     /**
    * @brief Getter for the adjacent vertices of a given vertex.
    *
    * @param v The ID of the vertex.
    * @return A vector with adjacent vertices ID.
    */
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