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
void MToyGraphParser(const string& filename, MGraph &g);
void MExtraMSGraphParser(const string& edge_filename, MGraph &g);
void MRealWorldGraphParser(const string& dir_name, MGraph &g);
void Parser(const string& path, MGraph &g); //a wrap
#endif //DA_PROJETO2_DATAPARSERMATRIX_H
