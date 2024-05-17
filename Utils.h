#ifndef DA_PROJETO2_UTILS_H
#define DA_PROJETO2_UTILS_H

#include "data_structures/MatrixGraph.h"
#include "data_structures/Graph.h"
#include <vector>
#include <set>
#include <iostream>
using namespace std;

bool BacktrakingTSP(MGraph &g, int &dist, int &minDist, std::vector<int> &tsp, std::vector<int> &final, int &numVertex, int vertex, bool&found, std::set<int> &visited);
void triangularApproximation(Graph<int> &g);
bool notPresent(int i, std::vector<int> &tsp);
pair<int, int> minDist(std::vector<double> currentRow, std::vector<int> &tsp);
bool nearestNeighbour(MGraph &g,  std::vector<int> &tsp, std::vector<std::vector<double>>* matrix);


/*
double distanceNodes(pair<int, std::pair<double, double>>& point1, pair<int, std::pair<double, double>>& point2);
void assignClusters(std::vector<pair<int, std::pair<double, double>>>& data,  vector<pair<int, std::pair<double, double>>>& centroids, vector<pair<int, int>>& labels);
void updateCentroids(vector<pair<int, std::pair<double, double>>>& data, vector<pair<int, std::pair<double, double>>>& centroids, vector<pair<int, int>>& labels);
void kmeans(std::vector<pair<int, std::pair<double, double>>> &data, int k, int maxIterations);
double distance(const pair<int, std::pair<double, double>>& p1, const pair<int, std::pair<double, double>>& p2);
std::vector<int> solveTSP(const std::vector<pair<int, std::pair<double, double>>>& points);
vector<vector<int>> solveTSPForEachCluster(std::vector<pair<int, std::pair<double, double>>>& data,
                            std::vector<pair<int, std::pair<double, double>>>& centroids,
                            std::vector<pair<int, int>>& labels);
*/






#endif //DA_PROJETO2_UTILS_H
