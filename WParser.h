#ifndef DA_PROJETO2_DATAPARSER_H
#define DA_PROJETO2_DATAPARSER_H

#include "data_structures/WGraph.h"
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unordered_set>

using namespace std;

/**
* @brief Parses a CSV file to populate a Graph object.
*
* This function reads a CSV file containing information about the vertex and edges
* of a toy graph. It adds the corresponding Vertex and Edges in both directions
*
* @param filename The name of the CSV file (without the .csv extension) to be parsed.
* @param g A reference to the MGraph object to be populated.
 *@complexity: O(E+V), where EE is the number of edges and VV is the number of vertices in the graph.
*/
vector<int> ToyGraphParser(const string& filename, Graph &g);

/**
* @brief Parses a CSV file to populate a Graph object.
*
* This function reads a CSV file containing information about the vertex and edges
* of a Extra Medium graph. It adds the corresponding Vertex and Edges in both directions
*
* @param filename The name of the CSV file (without the .csv extension) to be parsed.
* @param g A reference to the MGraph object to be populated.
 * @complexity O(N+E), where NN is the number of nodes (vertices) and EE is the number of edges in the graph
*/
void ExtraMSGraphParser(const string& edge_filename, Graph &g);

/**
* @brief Parses a CSV file to populate a Graph object.
*
* This function reads a CSV file containing information about the vertex and edges
* of a Real World graph. It adds the corresponding Vertex and Edges in both directions
*
* @param filename The name of the CSV folder where exist the files to be parsed.
* @param g A reference to the MGraph object to be populated.
 * @complexity:O(N+E), where NN is the number of nodes (vertices) and EE is the number of edges in the graph.
*/
void RealWorldGraphParser(const string& dir_name, Graph &g);

/**
 * @brief receives the filename fowards to the correct Parser
 *
 * This function selects the appropriate parser based on the given path.
 *
 * @param path The path or filename indicating the dataset to be parsed.
 * @param g A reference to the MGraph object to be populated.
 * @complexity: the complexity of one of the functions it calls (O(N+E)).
 */
void WParser(const string& path, Graph &g);

#endif //DA_PROJETO2_DATAPARSER_H
