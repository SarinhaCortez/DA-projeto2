#ifndef DA_PROJETO2_DATAPARSER_H
#define DA_PROJETO2_DATAPARSER_H

#include "data_structures/Graph.h"
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
void ToyGraphParser(const string& filename, Graph<int> &g);
void ExtraMSGraphParser(const string& edge_filename, Graph<int> &g);
void RealWorldGraphParser(const string& dir_name, Graph<int> &g);
void Parser(const string& path, Graph<int> &g); //a wrap
#endif //DA_PROJETO2_DATAPARSER_H
