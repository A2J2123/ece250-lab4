// ./graph.out < inputs/test01.in
#include "graph.h"
#include <iostream>
#include <string>

int main() {
    Graph g;
    std::string command;

    while (true) {
        std::cin >> command;
        if (command == "insert") {
            int a, b;
            double d, s;
            std::cin >> a >> b >> d >> s;
            g.insertEdge(a, b, d, s);
            std::cout << "success" << std::endl;
        } else if (command == "load") {
            std::string filename;
            std::cin >> filename;
            g.loadFromFile(filename);
        } else if (command == "traffic") {
            int a, b;
            double A;
            std::cin >> a >> b >> A;
            if(g.updateTraffic(a, b, A)){
                std::cout << "success" << std::endl; 
            } else {
                std::cout << "failure" << std::endl; 
            }
        } else if (command == "update") {
            std::string filename;
            std::cin >> filename;
            g.updateFromFile(filename);
        } else if (command == "print") {
            int a;
            std::cin >> a;
            g.printAdjacent(a);
        } else if (command == "delete") {
            int a;
            std::cin >> a;
            g.deleteVertex(a);
        } else if (command == "path") {
            int a, b;
            std::cin >> a >> b;
            g.findPath(a, b);
        } else if (command == "lowest") {
            int a, b;
            std::cin >> a >> b;
            double weight = g.findLowestWeightPath(a, b);
            if (weight>0 && weight < std::numeric_limits<double>::infinity()) {
                std::cout << weight << std::endl;
            } else {
                std::cout << "failure" << std::endl;
            }
        } else if (command == "exit") {
            break;
        }
    }

    return 0;
}