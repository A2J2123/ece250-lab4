#include "graph.h"
#include <iostream>
#include <fstream>
#include <limits>
#include <unordered_map>
#include <stack>
#include <queue>

// Comparator struct for priority queue in Dijkstra's algorithm
struct CompareByFirst {
    bool operator()(const std::pair<double, int>& a, const std::pair<double, int>& b) const {
        return a.first > b.first;
    }
};

// Helper function to calculate weight of an edge
double calculateWeight(const GraphEdge& edge) {
    return edge.distance / (edge.speedLimit * edge.adjustmentFactor);
}

// Method to insert an edge into the graph
void Graph::insertEdge(int a, int b, double d, double s) {
    // Check if the edge already exists
    if (adjacencyList.find(a) != adjacencyList.end() && adjacencyList[a].find(b) != adjacencyList[a].end()) {
        // Edge already exists, update d and s only
        adjacencyList[a][b].distance = d;
        adjacencyList[a][b].speedLimit = s;
        // Do not update A here if you want to preserve the current traffic conditions
    } else {
        // Edge does not exist, insert new edge with d, s, and A
        adjacencyList[a][b] = GraphEdge(d, s);
    }
    
    // Since the graph is undirected, repeat the process for the reverse edge b->a
    if (adjacencyList.find(b) != adjacencyList.end() && adjacencyList[b].find(a) != adjacencyList[b].end()) {
        adjacencyList[b][a].distance = d;
        adjacencyList[b][a].speedLimit = s;
    } else {
        adjacencyList[b][a] = GraphEdge(d, s);
    }
}

// Method to load graph data from a file
void Graph::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    int a, b;
    double d, s; 
    while (file >> a >> b >> d >> s) {
        insertEdge(a, b, d, s);
    }
    std::cout << "success" << std::endl;
}

// Method to update traffic conditions on an edge
bool Graph::updateTraffic(int a, int b, double A) {
    if (adjacencyList.find(a) != adjacencyList.end() && adjacencyList[a].find(b) != adjacencyList[a].end()) {
        adjacencyList[a][b].adjustmentFactor = A;
        adjacencyList[b][a].adjustmentFactor = A; // For undirected graph
        return true; 
    } else {
        return false;
    }
}

// Method to print adjacent vertices of a node
void Graph::printAdjacent(int a) {
    if (adjacencyList.find(a) == adjacencyList.end()) {
        std::cout << "failure" << std::endl;
        return;
    }
    for (const auto& neighbor : adjacencyList[a]) {
        std::cout << neighbor.first << " ";
    }
    std::cout << std::endl;
}

// Method to delete a vertex from the graph
void Graph::deleteVertex(int a) {
    if (adjacencyList.find(a) == adjacencyList.end()) {
        std::cout << "failure" << std::endl;
        return;
    }
    adjacencyList.erase(a);
    for (auto& pair : adjacencyList) {
        pair.second.erase(a);
    }
    std::cout << "success" << std::endl;
}

// Method to update traffic conditions from a file
void Graph::updateFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Could not open the file: " << filename << std::endl;
        return;
    }
    int a, b;
    double A;
    int updates = 0; 
    while (file >> a >> b >> A) {
        if(updateTraffic(a, b, A)){
            updates++; 
        }
    }
    if (updates > 0){
        std::cout << "success" << std::endl; 
    } else {
        std::cout << "failure"  << std::endl;
    }
}

// Method to find shortest path between two vertices using Dijkstra's algorithm
void Graph::findPath(int start, int end) {
    // Early failure checks
    if (adjacencyList.find(start) == adjacencyList.end() || adjacencyList.find(end) == adjacencyList.end()) {
        std::cout << "failure" << std::endl;
        return;
    }

    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> prev;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, CompareByFirst> pq;
    
    // Initialize distances and predecessors
    for (const auto& node : adjacencyList) {
        dist[node.first] = std::numeric_limits<double>::infinity();
        prev[node.first] = -1;
    }

    // Set start node distance to 0 and add it to the queue
    dist[start] = 0;
    pq.emplace(0.0, start);

    while (!pq.empty()) {
        auto current = pq.top();
        pq.pop();
        int currentNode = current.second;

        // Stop if we reach the end node
        if (currentNode == end) {
            break;
        }

        // Explore neighbors
        for (const auto& edge : adjacencyList[currentNode]) {
            int nextNode = edge.first;
            double weight = calculateWeight(edge.second); // Utilizing the calculateWeight function
            if (dist[currentNode] + weight < dist[nextNode]) {
                dist[nextNode] = dist[currentNode] + weight;
                prev[nextNode] = currentNode;
                pq.emplace(dist[nextNode], nextNode);
            }
        }
    }

    // Check if a path exists
    if (dist[end] == std::numeric_limits<double>::infinity()) {
        std::cout << "failure" << std::endl;
        return;
    }

    // Reconstruct the path from end to start
    std::stack<int> path;
    for (int at = end; at != -1; at = prev[at]) {
        path.push(at);
    }

    // Print the path
    while (!path.empty()) {
        std::cout << path.top();
        path.pop();
        if (!path.empty()) {
            std::cout << " ";
        }
    }
    std::cout << std::endl;
}

// Method to find the weight of the shortest path between two vertices
double Graph::findLowestWeightPath(int start, int end) {
    // Re-use the logic from findPath but only return the total weight of the shortest path
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
    std::unordered_map<int, double> distances;

    for (const auto& node : adjacencyList) {
        distances[node.first] = std::numeric_limits<double>::infinity();
    }

    distances[start] = 0.0;
    pq.push({0.0, start});

    while (!pq.empty()) {
        int current = pq.top().second;
        pq.pop();

        if (current == end) break;

        for (const auto& neighbor : adjacencyList[current]) {
            int next = neighbor.first;
            double weight = calculateWeight(neighbor.second);
            double newDist = distances[current] + weight;

            if (newDist < distances[next]) {
                distances[next] = newDist;
                pq.push({newDist, next});
            }
        }
    }

    // Return the weight of the shortest path or -1 if no path exists
    return distances[end] != std::numeric_limits<double>::infinity() ? distances[end] : -1;
}