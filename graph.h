#ifndef GRAPH_H
#define GRAPH_H

#include <unordered_map>
#include <vector>
#include <string>

class GraphEdge {
public:
    double distance;
    double speedLimit;
    double adjustmentFactor;
    GraphEdge(double d = 0.0, double s = 0.0, double A = 1.0)
        : distance(d), speedLimit(s), adjustmentFactor(A) {}
};

class Graph {
private:
    std::unordered_map<int, std::unordered_map<int, GraphEdge>> adjacencyList;

public:
    Graph() {}
    void insertEdge(int a, int b, double d, double s);
    void loadFromFile(const std::string& filename);
    bool updateTraffic(int a, int b, double A);
    void printAdjacent(int a);
    void deleteVertex(int a);
    void updateFromFile(const std::string& filename);
    // Path and lowest weight functionality will be included within the findPath method
    void findPath(int start, int end); // Implement this function to find and print the shortest path
    double findLowestWeightPath(int start, int end); // Implement this function to calculate and return the weight of the shortest path
};

#endif // GRAPH_H