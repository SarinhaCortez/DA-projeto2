#ifndef DA_PROJETO2_UTILS_H
#define DA_PROJETO2_UTILS_H

#include "data_structures/MatrixGraph.h"
#include <vector>
#include <set>

bool BacktrakingTSP(MGraph &g, int &dist, int &minDist, std::vector<int> &tsp, std::vector<int> &final, int &numVertex, int vertex, bool&found, std::set<int> &visited);

#endif //DA_PROJETO2_UTILS_H
