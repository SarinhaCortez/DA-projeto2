#ifndef DA_PROJETO2_DATAPARSER_H
#define DA_PROJETO2_DATAPARSER_H

#include "data_structures/WGraph.h"
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unordered_set>

using namespace std;
vector<int> ToyGraphParser(const string& filename, Graph &g);
void ExtraMSGraphParser(const string& edge_filename, Graph &g);
void RealWorldGraphParser(const string& dir_name, Graph &g);
void WParser(const string& path, Graph &g);

#endif //DA_PROJETO2_DATAPARSER_H
