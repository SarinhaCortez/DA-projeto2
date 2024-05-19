#ifndef DA_PROJETO2_DATAPARSERMATRIX_H
#define DA_PROJETO2_DATAPARSERMATRIX_H

#include "data_structures/MatrixGraph.h"
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unordered_set>

using namespace std;
/**
 * Toy Graph parser receives the name of a file xxxxx.csv
 * Extra receives a name in the form "edges_XXXX"
 * RW receives the name of a directory
 * based on this informations, Parser knows which funct to call
 */

/**
* @brief Parses a CSV file to populate a MGraph object.
*
* This function reads a CSV file containing information about the vertexz and edges
* of a toy graph. It updates the graph's adjacency matrix to reflect the distances
* between vertexs.
*
* @param filename The name of the CSV file (without the .csv extension) to be parsed.
* @param g A reference to the MGraph object to be populated.
 * @complexity O(max(V,E)), where V are the Vertices and E the edges.
*/
void MToyGraphParser(const string& filename, MGraph &g);

/**
 * @brief Converts a coordinate from degrees to radians.
 *
 * This function takes a coordinate in degrees and converts it to radians.
 *
 * @param coord The coordinate in degrees.
 * @return The coordinate converted to radians.
 * complexity O(1)
 */
double convert_radians(double coord);


/**
 * @brief Calculates the Haversine distance between two nodes of a graph.
 *
 * This function calculates the Haversine distance between two nodes, given their
 * id and a graph containing their latitude and longitude. The Haversine formula determines
 * distance between two points on a sphere given their coordinates.
 *
 * @param vertex1 The ID of the first node.
 * @param vertex2 The ID of the second node.
 * @param g A reference to the graph containing the vertexs. Each
 *          vertex is a pair of doubles representing longitude and latitude.
 * @return The Haversine distance between the two nodes.
 * @complexity O(1)
 */
int harversineDistance(int node1, int node2, MGraph &g);


/**
 *@brief Parses a CSV file to populate a MGraph object.
 *
 * This function reads edge and vertexs data from CSV files and uses it to populate an MGraph object.
 * The function first reads the edges to determine the maximum node ID, then reads the vertexs to update
 * their longitude and latitude, and finally fills the adjacency matrix with distances.
 *
 * @param edge_filename The name of the CSV file containing edge data (without the .csv extension).
 * @param g A reference to the MGraph object to be populated.
 *@complexity O(max(E,V)) where V are the vertices and E the edges
 */
void MExtraMSGraphParser(const string& edge_filename, MGraph &g);

/**
 *@brief Parses a CSV file to populate a MGraph object.
 *
 * This function reads vertexs and edge data from CSV files in a specified directory, and uses
 * it to populate an MGraph object. It first reads the nodes file to determine the matrix dimension and
 * vertex information, then reads the edges to update the adjacency matrix with distances. If an
 * edge does not exist, the Haversine distance between the nodes is calculated and filled in.
 *
 * @param dir_name The name of the directory containing the node and edge CSV files.
 * @param g A reference to the MGraph object to be populated.
 * @complexity O(V**2) where V are the vertexs
 */
void MRealWorldGraphParserHaversine(const string& dir_name, MGraph &g);

/**
*@brief Parses a CSV file to populate a MGraph object.
 *
 * This function reads vertex and edge data from CSV files in a specified directory, and uses
 * it to populate an MGraph object. It first reads the nodes file to the matrix dimension and
 * vertex information, then reads the edges to update the adjacency matrix with distances.
 *
 * @param dir_name The name of the directory containing the node and edge CSV files.
 * @param g A reference to the MGraph object to be populated.
 * @complexity O(max(V,E)) where E are the edges and V are the Vertices
 */
void MRealWorldGraphParser(const string& dir_name, MGraph &g);

/**
 * @brief receives the filename fowards to the correct Parser
 *
 * This function selects the appropriate parser based on the given path.
 *
 * @param path The path or filename indicating the dataset to be parsed.
 * @param g A reference to the MGraph object to be populated.
 * @param haversine A boolean flag indicating whether to use Haversine distance for real-world graphs.
 * @complexity in the worst case the maximum complexity between all the parsers called
 */
void Parser(const string& path, MGraph &g, bool haversine);

//void MExtraMSGraphParserHaversine(const string& edge_filename, MGraph &g);
#endif //DA_PROJETO2_DATAPARSERMATRIX_H
