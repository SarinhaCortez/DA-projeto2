#ifndef DA_PROJETO2_UTILS_H
#define DA_PROJETO2_UTILS_H

#include "data_structures/MatrixGraph.h"
#include "data_structures/WGraph.h"
#include <vector>
#include <set>
#include <iostream>
using namespace std;

/**
 * @brief Backtracking algorithm to solve the Traveling Salesman Problem (TSP).
 *
 * This function recursively searches for the optimal Hamiltonian cycle (or tour) in a given graph using backtracking.
 * The goal is to find the shortest possible tour that visits each vertex exactly once and returns to the starting city,
 * thereby minimizing the total distance traveled by a salesman.
 *
 * @param g The graph representing the TSP instance.
 * @param dist The current distance of the tour being explored.
 * @param minDist The minimum distance found so far.
 * @param tsp The current partial tour being constructed.
 * @param final The final optimal tour found.
 * @param numVertex The number of remaining vertices to visit.
 * @param vertex The current vertex being visited.
 * @param found A boolean flag indicating if the optimal tour has been found.
 * @param visited A set containing the vertices already visited.
 * @details This function has a time complexity of O(n⋅(n−1)!), where N is the number of nodes
     *          and (n-1)! is the number of time this function calls itself.
 * @return True if the optimal tour is found, otherwise false.
 */
bool BacktrakingTSP(MGraph &g, int &dist, int &minDist, std::vector<int> &tsp, std::vector<int> &final, int &numVertex, int vertex, bool&found, std::set<int> &visited);

/**
 * @brief Checks if id of the Vertex is not present in a given vector.
 *
 * This function iterates through the elements of the vector to determine if the specified integer
 * is present in it.
 *
 * @param i The parameter to check for presence in the vector.
 * @param tsp The vector in which to search.
 * @details This function has a time complexity of O(n), where N is the number of elements in tsp.
 * @return True if the integer is not present in the vector, otherwise false.
 */
bool notPresent(int i, std::vector<int> &tsp);

/**
 * @brief Finds the minimum distance and its corresponding vertex index from a given row in a distance matrix.
 *
 * This function iterates through the elements of the row to find the minimum distance that is not already included
 * in the specified vector representing visited vertices.
 *
 * @param currentRow The row of distances from a specific vertex to all other vertices.
 * @param tsp The vector representing the visited vertices.
 * @details This function has a time complexity of O(n⋅m), where n and m are the number of elements in currentRow and tsp, respectively.
 * * @return A pair containing the minimum distance and its corresponding vertex index.
 */
pair<int, int> minDist(std::vector<double> currentRow, std::vector<int> &tsp);

/**
 * @brief Finds the nearest neighbor for each vertex in the graph to construct a tour.
 *
 * This function applies the nearest neighbor heuristic to construct a tour in the given graph.
 * Starting from a specified initial vertex, it iteratively selects the nearest unvisited neighbor
 * until all vertices have been visited.
 *
 * @param g The graph for which to construct the tour.
 * @param tsp The vector representing the tour.
 * @param matrix A pointer to the matrix representing the distances between vertices.
 * @details This function has a time complexity of O(n^3), where n is the number of nodes
 * @return True on success, otherwise false.
 */
bool nearestNeighbour(MGraph &g,  std::vector<int> &tsp, std::vector<std::vector<double>>* matrix);

/**
 * @brief Prim's algorithm to find the minimum spanning tree of a graph.
 *
 * Prim's algorithm is a greedy algorithm that finds a minimum spanning tree.
 * It starts with vertex 0 and grows the spanning tree by iteratively adding the lowest distance edge that connects
 * a vertex in the tree to a vertex not yet in the tree.
 *
 * @param g A pointer to the graph for which to find the minimum spanning tree.
 * @details This function has a time complexity of O((V +E) log V), where V is the number of vertex and E is the number of edges.
 * @return An unordered map containing the vertices of the minimum spanning tree
*/
vector<Vertex *> prim(Graph& g);

/**
 * @brief Performs a pre-order traversal starting from a specified vertex in a graph.
 *
 * This function traverses the graph, starting from vertex 0.
 * It marks each visited vertex, adds it to the tour vector, if it is according to the path
 * defined in Prim function, and recursively traverses its adjacent vertices
 * that have not been visited yet.
 *
 * @param vertexSet An unordered map containing all vertices in the graph.
 * @param v0 The starting vertex for the pre-order traversal.
 * @param tour A vector to store the vertices visited during the pre-order traversal.
 * @details This function has a time complexity of O(V+E), where V is the number of vertex and E is the number of edges.
 */
void preOrder(vector<Vertex *> vertexSet, Vertex* v0, vector<Vertex*>& tour);

/**
 * @brief Calculates the total distance of a tour represented by a vector of vertexs.
 *
 * This function computes the total distance of a tour .It iterates through the vertexs in the tour and calculates
 * the distance between adjacent vertices.
 * Additionally, it considers the distance from the last vertex in the tour back to the starting vertex (vertex 0).
 *
 * @param tour A vector containing pointers to the vertices in the tour.
 * @details This function has a time complexity of O(n⋅d), where N is the number of nodes and td is the maxium number of outcoming edges of a node
 * @return The total distance of the tour.
 */
double calculateTourDistance(const vector<Vertex *>& tour);

/**
 * @brief Applies the triangular approximation algorithm to find a minimum tour in a graph.
 *
 * The function first constructs a minimum spanning tree of the graph using Prim's algorithm.
 * Then, it performs a pre-order traversal starting from the root of the MST to obtain a tour.
 * Next, it removes duplicate vertexs from the tour.
 * Finally, it calculates the total distance of the tour and prints the result.
 *
 * @param g The graph for which to find the minimum tour.
 * @details This function has a time complexity of O(V^2 * logV), where V is the number of nodes.
 */
void triangularApproximation(Graph &g);

/**
 * @brief Removes duplicate vertices from a vector.
 *
 * This function removes duplicate vertices from the given tour vector.
 * It utilizes an unordered set to keep track of vertices already seen.
 * For each vertex in the input tour, if the vertex is not already present in the set,
 * it is added to both the set and the result vector. This ensures that each unique vertex
 * is included exactly once in the result.
 *
 * @param tour A vector containing pointers to vertices representing the tour.
 * @details This function has a time complexity of O(n), where N is the number of elements of vector tour.
 * @return A vector containing pointers to vertices with duplicates removed.
 */
vector<Vertex *> removeDup(vector<Vertex*> tour);

/**
 * @brief Calculates the Haversine distance between two vertexs.
 *
 * This function using the vertexs latitude and longitude coordinates calculates the distance between two vertexs.
 *
 * @param v1 Pointer to the first vertex.
 * @param v2 Pointer to the second vertex.
 * @details This function has a time complexity of O(1).
 * @return The Haversine distance between the two vertices in meters.
 */
double harversineDistance(Vertex * v1, Vertex * v2);

/**
 * @brief Ensures full connectivity of a real-world graph.
 *
 * This function ensures that every vertex in the graph is connected to every other vertex
 * by adding missing edges based on the Haversine distance between vertices.
 *
 * @param g The graph to be made fully connected.
 * @details This function has a time complexity of O(n^2), where N is the number of nodes.
 */
void RealWorldFullyConnected(Graph & g);


#endif //DA_PROJETO2_UTILS_H
