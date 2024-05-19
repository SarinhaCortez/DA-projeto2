//
// Created by saracortez on 10/05/24.
//

#ifndef DA_PROJETO2_TSPSOLVER_H
#define DA_PROJETO2_TSPSOLVER_H

#include <stack>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <algorithm>
#include <random>
#include "MatrixGraph.h"

class TSPSolver {
    MGraph graph;
    double temperature;
    double coolingRate;
    int maxIterations = 100;

    double calculateCost(const std::vector<int>& tour) {
        double totalDistance = 0.0;
        for (size_t i = 0; i < tour.size() - 1; ++i) {
            totalDistance += graph.getWeight(tour[i], tour[i+1]);
        }
        totalDistance += graph.getWeight(tour.back(), tour.front());
        return totalDistance;
    }

    std::vector<int> generateNeighbor(const std::vector<int>& path) {
        std::vector<int> newPath = path; // Create a copy of the original path

        int pathSize = newPath.size();

        for (int i = 1; i < pathSize - 1; ++i) {
            for (int j = i + 1; j < pathSize; ++j) {
                // Get the indices of the vertices involved in the swap
                int a = newPath[i - 1], b = newPath[i];
                int c = newPath[j], d = (j + 1 == pathSize) ? newPath[0] : newPath[j + 1];

                // Check if the edges (a, b) and (c, d) exist in the graph
                if (graph.getWeight(a, c) >= 0 && graph.getWeight(b, d) >= 0) {
                    // Calculate the lengths of the old and new edges
                    double oldLength = graph.getWeight(a, b) + graph.getWeight(c, d);
                    double newLength = graph.getWeight(a, c) + graph.getWeight(b, d);

                    // If the new path is shorter, perform the swap
                    if (newLength < oldLength) {
                        // Construct the new path avoiding non-existent edges
                        std::vector<int> tempPath;
                        tempPath.reserve(pathSize);
                        for (int k = 0; k < i; ++k) {
                            tempPath.push_back(newPath[k]);
                        }
                        for (int k = j; k >= i; --k) {
                            tempPath.push_back(newPath[k]);
                        }
                        for (int k = j + 1; k < pathSize; ++k) {
                            tempPath.push_back(newPath[k]);
                        }

                        if (graph.getWeight(tempPath.back(), tempPath.front() - 1) >= 0) {
                            return tempPath; // Return the new valid path
                        } else {
                            return std::vector<int>(); // Return an empty path if the edge doesn't exist
                        }
                    }
                }
            }
        }

        // If no improvement is possible, return the original path
        return newPath;
    }




    double acceptanceProbability(double energy, double newEnergy, double temperature) {
        if (newEnergy < energy) {
            return 1.0;
        }

        return exp((energy - newEnergy) / temperature);
    }
    bool findHamiltonianPathDFS(int currentVertex, std::vector<int>& path, std::vector<bool>& visited, int startVertex) {
        path.push_back(currentVertex);
        visited[currentVertex] = true;

        if (path.size() == graph.getNumVertex()) {
            // Ensure the path forms a Hamiltonian cycle
            if (graph.getWeight(currentVertex, startVertex) >= 0) {
                path.push_back(startVertex);  // Close the cycle
                return true;
            } else {
                path.pop_back();
                visited[currentVertex] = false;
                return false;
            }
        }

        for (int nextVertex : graph.getAdj(currentVertex)) {
            if (!visited[nextVertex] && graph.getWeight(currentVertex, nextVertex) >= 0) {
                if (findHamiltonianPathDFS(nextVertex, path, visited, startVertex)) {
                    return true;
                }
            }
        }

        path.pop_back();
        visited[currentVertex] = false;
        return false;
    }

    std::vector<int> findInitialHamiltonianPath(int sV) {
        std::vector<int> path;
        std::vector<bool> visited(graph.getNumVertex(), false);

        if (findHamiltonianPathDFS(sV, path, visited, sV)) {
            return path;
        }
        path.clear();
        std::fill(visited.begin(), visited.end(), false);

        return std::vector<int>();
    }

public:
    TSPSolver(MGraph &graph, double initialTemperature, double coolingRate, int maxIterations)
            : graph(graph), temperature(initialTemperature), coolingRate(coolingRate), maxIterations(maxIterations) {}

    std::vector<int> solve(double &bc, int sV) {
        double averageCostIncrease = 0.0;
        int it = 0;
        std::vector<int> currentTour = findInitialHamiltonianPath(sV);

        if (currentTour.empty()) {
            return currentTour;
        }

        std::vector<int> bestTour = currentTour;
        double bestCost = calculateCost(currentTour);

        srand(time(NULL));
        for (int iteration = 0; iteration < maxIterations; ++iteration) {
            std::vector<int> newTour = generateNeighbor(currentTour);
            /*int n = 10;
            while((n-- > 0 && newTour.empty()){
                newTour = generateNeighbor(currentTour);
                if(n == 1 && newTour.empty()))return currentTour;
            }*/
            if(newTour.empty()) newTour = currentTour;
            double currentCost = calculateCost(currentTour);
            double newCost = calculateCost(newTour);

            if (acceptanceProbability(currentCost, newCost, temperature) > (double)rand() / RAND_MAX) {
                currentTour = newTour;
                if (newCost < bestCost) {
                    bestTour = newTour;
                    bestCost = newCost;
                }
            }
            temperature *= coolingRate;
            averageCostIncrease += (newCost - currentCost);
            it = iteration;
        }
        std::cout << endl << "Average Cost Increase = " << averageCostIncrease / it << std::endl;
        bc = bestCost;
        return bestTour;
    }
};


#endif //DA_PROJETO2_TSPSOLVER_H
