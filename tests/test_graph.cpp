#include <cassert>
#include <iostream>

#include "Graph.h"
#include "WeightedGraph.h"

// Check that sort_vertices is a topological sort of graph
bool checkTopologicalSort(const Graph &graph,
                          const std::vector<size_t> &sorted_vertices) {
    // Build a map from vertex ID to its index in the sorted list
    std::unordered_map<size_t, size_t> vertex_order;
    for (size_t i = 0; i < sorted_vertices.size(); ++i) {
        vertex_order[sorted_vertices[i]] = i;
    }

    // Ensure that each edge's source comes before its destination
    for (const auto &edge : graph.getEdges()) {
        size_t from_id = edge->from->vid;
        size_t to_id = edge->to->vid;

        if (vertex_order[from_id] >= vertex_order[to_id]) {
            std::cerr << "Invalid topological sort: " << from_id
                      << " appears after " << to_id << std::endl;
            return false;
        }
    }

    // Check that sort includes all vertices
    if (sorted_vertices.size() != graph.getNumVertices()) {
        std::cerr << "Incorrect size: " << sorted_vertices.size() << " vs "
                  << graph.getNumVertices() << std::endl;
        return false;
    }

    return true;
}

void testGraph() {
    Graph graph;

    // Add vertices
    graph.addVertex(1);
    graph.addVertex(2);
    graph.addVertex(3);
    graph.addVertex(4);
    graph.addVertex(5);

    // Add edges with costs
    graph.addEdge(1, 2, 1);
    graph.addEdge(1, 4, 2);
    graph.addEdge(2, 4, 3);
    graph.addEdge(2, 5, 4);
    graph.addEdge(3, 4, 5);
    graph.addEdge(4, 5, 6);

    // Display the graph
    graph.display();

    const auto sorted_vertices = graph.topologicalSort();
    std::cout << "  Topological Sorting Order: ";
    for (const auto &vertex : sorted_vertices) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;

    assert(checkTopologicalSort(graph, sorted_vertices));
}

void testWeightedGraph() {
    WeightedGraph<double> graph;

    // Add vertices
    graph.addVertex(1);
    graph.addVertex(2);
    graph.addVertex(3);
    graph.addVertex(4);
    graph.addVertex(5);

    // Add edges with costs
    graph.addEdge(1, 2, 1, 1);
    graph.addEdge(1, 4, 2, 2);
    graph.addEdge(2, 4, 3, 3);
    graph.addEdge(2, 5, 4, 4);
    graph.addEdge(3, 4, 5, 5);
    graph.addEdge(4, 5, 6, 6);

    // Display the graph
    graph.display();

    const auto sorted_vertices = graph.topologicalSort();
    std::cout << "  Topological Sorting Order: ";
    for (const auto &vertex : sorted_vertices) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;

    assert(checkTopologicalSort(graph, sorted_vertices));
}

void testIsDAG() {
    Graph g;

    // DAG: 1 -> 2 -> 3
    g.addVertex(1);
    g.addVertex(2);
    g.addVertex(3);
    g.addEdge(1, 2, 1);
    g.addEdge(2, 3, 2);

    assert(g.isDAG());

    // Add a cycle: 3 -> 1
    g.addEdge(3, 1, 3);
    assert(!g.isDAG());
}

void testGetPath() {
    Graph g;

    g.addVertex(1);
    g.addVertex(2);
    g.addVertex(3);
    g.addVertex(4);
    g.addEdge(1, 2, 1);
    g.addEdge(2, 3, 2);
    g.addEdge(3, 4, 3);

    auto path = g.getPath(1, 4);
    std::vector<size_t> expected = {1, 2, 3, 4};

    assert(path.size() == expected.size() - 1); // 3 edges for 4 vertices

    for (size_t i = 0; i < path.size(); ++i) {
        assert(path[i]->from->vid == expected[i]);
        assert(path[i]->to->vid == expected[i + 1]);
    }

    auto no_path = g.getPath(4, 1);
    assert(no_path.empty());
}

void testIsTotallyOrdered() {
    Graph g;

    // Total order: 10 -> 20 -> 30 -> 40
    g.addVertex(10);
    g.addVertex(20);
    g.addVertex(30);
    g.addVertex(40);
    g.addEdge(10, 20, 1);
    g.addEdge(20, 30, 2);
    g.addEdge(30, 40, 3);

    assert(g.isDAG());
    assert(g.isTotallyOrdered());

    // Break total order by adding a parallel branch
    g.addVertex(50);
    g.addEdge(20, 50, 4); // Now 20 points to two nodes: 30 and 50

    assert(g.isDAG());
    assert(!g.isTotallyOrdered());
}
