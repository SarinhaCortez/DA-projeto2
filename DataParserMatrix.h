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
 * Extra receives a name in the form "edges_XXXX.csv"
 * RW receives the name of a directory
 * based on this informations, Parser knows which funct to call
 */
void MToyGraphParser(const string& filename, Graph &g);
void MExtraMSGraphParser(const string& edge_filename, Graph &g);
void MRealWorldGraphParser(const string& dir_name, Graph &g);
void Parser(const string& path, Graph &g); //a wrap
#endif //DA_PROJETO2_DATAPARSERMATRIX_H
