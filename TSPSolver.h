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

                        // Ensure there is an edge from the last vertex to the first
                        if (graph.getWeight(tempPath.back(), tempPath.front()) >= 0) {
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
            return 1.0; // Accept the new solution
        }
        // Calculate acceptance probability based on temperature and energy difference
        return exp((energy - newEnergy) / temperature);
    }
    bool findHamiltonianPathDFS(int currentVertex, std::vector<int>& path, std::unordered_set<int>& visited) {
        // Add currentVertex to the path and mark it as visited
        path.push_back(currentVertex);
        visited.insert(currentVertex);

        // Check if all vertices have been visited
        if (path.size() == graph.getNumVertex()) {
            // Verify if there's an edge from the last vertex to the first to form a Hamiltonian cycle
            if (graph.getWeight(path.back(), path.front()) >= 0) {
                return true;
            } else {
                // If not, remove the last vertex from the path and backtrack
                path.pop_back();
                visited.erase(currentVertex);
                return false;
            }
        }

        // Iterate over adjacent vertices
        for (int nextVertex : graph.getAdj(currentVertex)) {
            // Explore unvisited adjacent vertices recursively
            if (visited.find(nextVertex) == visited.end()) {
                if (findHamiltonianPathDFS(nextVertex, path, visited)) {
                    return true; // Found a Hamiltonian path
                }
            }
        }

        // If no Hamiltonian path is found from the current vertex, backtrack
        path.pop_back();
        visited.erase(currentVertex);
        return false;
    }

    std::vector<int> findInitialHamiltonianPath() {
        std::vector<int> path;
        std::unordered_set<int> visited;

        // Start DFS from each vertex to find a Hamiltonian path
        for (int startVertex = 0; startVertex < graph.getNumVertex(); ++startVertex) {
            if (findHamiltonianPathDFS(startVertex, path, visited)) {
                // If a Hamiltonian path is found, return it
                return path;
            }
        }

        // If no Hamiltonian path is found, return an empty path
        return std::vector<int>();
    }


public:
    TSPSolver(MGraph &graph, double initialTemperature, double coolingRate, int maxIterations)
            : graph(graph), temperature(initialTemperature), coolingRate(coolingRate), maxIterations(maxIterations) {}

    std::vector<int> solve(double &bc) {
        double averageCostIncrease; int it;
        std::vector<int> currentTour; // Initialize with a random tour
        currentTour = findInitialHamiltonianPath();

        std::vector<int> bestTour = currentTour;
        double bestCost = calculateCost(currentTour);

        srand(time(NULL));
        for (int iteration = 0; iteration < maxIterations; ++iteration) {
            std::vector<int> newTour = generateNeighbor(currentTour);
            double currentCost = calculateCost(currentTour);
            double newCost = calculateCost(newTour);

            if (acceptanceProbability(currentCost, newCost, temperature) > (double)rand() / RAND_MAX) {
                currentTour = newTour;
                if (newCost < bestCost) {
                    bestTour = newTour;
                    bestCost = newCost;
                }
            }
            temperature *= coolingRate; // Update temperature according to cooling schedule
            averageCostIncrease += (newCost - currentCost);
            it = iteration;
        }
        cout << "Average Cost Increase = " << averageCostIncrease/it<< std::endl;
        bc = bestCost;
        return bestTour;
    }
};



#endif //DA_PROJETO2_TSPSOLVER_H
