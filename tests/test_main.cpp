#include <cassert>
#include <vector>

#include "Graph.h"
#include "OptimalSet.h"
#include "Plan.h"
#include "Rulebook.h"
#include "RulebookCost.h"
#include "WeightedGraph.h"

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

    const auto sortedVertices = graph.topologicalSort();
    std::cout << "  Topological Sorting Order: ";
    for (const auto &vertex : sortedVertices) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
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

    const auto sortedVertices = graph.topologicalSort();
    std::cout << "  Topological Sorting Order: ";
    for (const auto &vertex : sortedVertices) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
}

Rulebook testRulebook() {
    Rulebook rulebook;

    rulebook.addEquivalentRules({1, 2, 3});
    rulebook.addEquivalentRules({0, 4});
    rulebook.addEquivalentRules({6});
    rulebook.addEquivalentRules({7});

    rulebook.addGTRelation(2, 4);
    rulebook.addGTRelation(0, 6);
    rulebook.addGTRelation(7, 6);
    rulebook.addGTRelation(4, 6);

    rulebook.build();

    rulebook.display();

    std::cout << "Order: ";
    for (auto it = rulebook.begin(); it != rulebook.end(); ++it) {
        std::cout << "{ ";
        for (auto elem : *it) {
            std::cout << elem << " ";
        }
        std::cout << "} ";
    }
    std::cout << std::endl;

    // Successors
    std::cout << "Successors:" << std::endl;
    for (size_t i = 0; i < 8; ++i) {
        std::cout << "  " << i << ": ";
        const auto successors = rulebook.getSuccessors(i);
        for (auto j : successors) {
            std::cout << j << " ";
        }
        std::cout << std::endl;
    }
    return rulebook;
}

void testRulebookCost(const Rulebook &rulebook) {
    RulebookCost::setRulebook(rulebook);
    size_t num_rules = rulebook.getNumRules();
    RulebookCost cost1(num_rules);
    RulebookCost cost2(num_rules);
    cost2.setRuleCost(1, 2);
    cost1.setRuleCost(0, 2);
    bool res1 = cost1 <= cost2;
    bool res2 = cost2 <= cost1;
    bool res3 = cost1 == cost2;
    bool res4 = cost1 < cost2;
    bool res5 = cost1 > cost2;
    assert(res1);
    assert(!res2);
    assert(!res3);
    assert(res4);
    assert(!res5);
}

void testOptimalSet() {
    OptimalSet<Plan, double> optimalSet;
    std::vector<int> element1 = {1, 2, 3};
    std::vector<int> element2 = {2, 2, 4};
    std::vector<int> element3 = {2, 3, 4};
    std::vector<int> element4 = {2, 3, 4};

    Plan p1(element1);
    Plan p2(element2);
    Plan p3(element3);
    Plan p4(element4);

    optimalSet.insert(p1, 10.0, 1);
    optimalSet.insert(p1, 8.0, 2);
    optimalSet.insert(p2, 5.0, 3);
    optimalSet.insert(p3, 15.0, 4);
    optimalSet.insert(p3, 5.0, 5);

    // Print the elements in the optimal set
    std::cout << optimalSet << std::endl;
}

void testSearch() {
    Rulebook rulebook;

    rulebook.addEquivalentRules({0});
    rulebook.addEquivalentRules({1});
    rulebook.build();

    RulebookCost::setRulebook(rulebook);
    size_t num_rules = rulebook.getNumRules();

    WeightedGraph<RulebookCost> graph;

    // Add vertices
    graph.addVertex(1);
    graph.addVertex(2);
    graph.addVertex(3);
    graph.addVertex(4);
    graph.addVertex(5);
    graph.addVertex(6);

    // Add edges with costs
    RulebookCost cost(num_rules);
    cost.setRuleCost(0, 1);
    cost.setRuleCost(1, 1);
    graph.addEdge(1, 2, cost, 1);

    cost.setRuleCost(0, 10);
    cost.setRuleCost(1, 10);
    graph.addEdge(2, 6, cost, 2);

    cost.setRuleCost(0, 1);
    cost.setRuleCost(1, 1);
    graph.addEdge(1, 3, cost, 3);
    graph.addEdge(3, 4, cost, 4);
    graph.addEdge(4, 5, cost, 5);
    graph.addEdge(5, 6, cost, 6);

    std::cout << "Graph:" << std::endl;
    graph.display();

    const auto res = graph.getOptimalPaths(1, 6);
    std::cout << "Optimal set: " << res << std::endl;
}

int main() {
    std::cout << "Testing Graph ..." << std::endl;
    testGraph();
    std::cout << std::endl;
    std::cout << "Testing WeightedGraph ..." << std::endl;
    testWeightedGraph();
    std::cout << std::endl;
    std::cout << "Testing Rulebook ..." << std::endl;
    const Rulebook rulebook = testRulebook();
    std::cout << std::endl;
    std::cout << "Testing RulebookCost ..." << std::endl;
    testRulebookCost(rulebook);
    std::cout << std::endl;
    std::cout << "Testing OptimalSet ..." << std::endl;
    testOptimalSet();
    std::cout << std::endl;
    std::cout << "Testing Search ..." << std::endl;
    testSearch();
    std::cout << std::endl;

    return 0;
}
