#ifndef DA_PROJETO2_UTILS_H
#define DA_PROJETO2_UTILS_H

#include "data_structures/Graph.h"
#include <vector>

bool BacktrakingTSP(Graph<int> &g, int &dist, int &minDist, std::vector<Vertex<int> *> &tsp, std::vector<Vertex<int> *> &final, int &numVertex, Vertex<int> * vertex, bool &found);

#endif //DA_PROJETO2_UTILS_H
