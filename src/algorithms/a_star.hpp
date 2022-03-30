
#ifndef A_STAR_H
#define A_STAR_H

#include <utility>
#include <list>
#include <map>
#include <algorithm>
#include "../types.hpp"
#include "../graph.hpp"
#include "../data_structures/fibonacci_heap.hpp"

template <typename T>
std::pair<std::list<u64>, double> aStarSearch(Graph<T> g, u64 start, u64 end) {
    FibonacciHeap<u64> heap;
    std::map<u64, u64> predecessorMap;
    std::map<u64, double> currentCostMap;
    predecessorMap[0] = 0;

    double distance = 0;
    double estimate = 0; // TODO
    currentCostMap[start] = distance;
    predecessorMap[start] = nullptr;
    heap.insert(start, distance + estimate);

    std::list<std::pair<u64, double>> edges;
    while (!heap.empty()) {
        u64 min = heap.extractMin();
        edges = g.getEdges(min);
        for (std::pair<u64, bool> edge : edges) {
            u64 nextNode = edge.first;
            double edgeLength = edge.second;
            distance = currentCostMap[min] + edgeLength;

            // Check if the destination node has been reached
            if (nextNode == end) {
                predecessorMap[nextNode] = min;
                std::list<u64>* path = new std::list<u64>(end);
                u64 node = end;
                while (node != start) {
                    node = predecessorMap[node];
                    path->push_front(node);
                }
                return std::make_pair<std::list<u64>, double>(*path, distance);
            }

            // Update distance and predecessor and add node to the heap (only if the node is new)
            if (!currentCostMap.count(nextNode)) {
                double estimate = 0; // TODO

                currentCostMap[nextNode] = distance;
                predecessorMap[nextNode] = min;
                heap.insert(nextNode, distance + estimate);
            }
        }
    }

    // Failed to find a path between start and end
    return std::make_pair<std::list<u64>, double>({}, 0);
}

#endif // A_STAR_H
