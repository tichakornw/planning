#include <cassert>
#include <iostream>

#include "Rulebook.h"
#include "RulebookCost.h"
#include "WeightedGraph.h"

void testOptimalPaths() {
    std::cout << "Testing OptimalPaths ..." << std::endl;
    Rulebook rulebook;

    rulebook.addRule(RuleSum("r0"));
    rulebook.addRule(RuleSum("r1"));
    rulebook.build();

    RulebookCost::setRulebook(rulebook);
    WeightedGraph<RulebookCost> graph;

    // Add vertices
    graph.addVertex(1);
    graph.addVertex(2);
    graph.addVertex(3);
    graph.addVertex(4);
    graph.addVertex(5);
    graph.addVertex(6);

    // Add edges with costs
    RulebookCost cost11;
    cost11.setRuleCost(0, 1);
    cost11.setRuleCost(1, 1);
    graph.addEdge(1, 2, cost11, 1);

    RulebookCost cost1010;
    cost1010.setRuleCost(0, 10);
    cost1010.setRuleCost(1, 10);
    graph.addEdge(2, 6, cost1010, 2);

    graph.addEdge(1, 3, cost11, 3);
    graph.addEdge(3, 4, cost11, 4);
    graph.addEdge(4, 5, cost11, 5);
    graph.addEdge(5, 6, cost11, 6);

    std::cout << "Graph:" << std::endl;
    graph.display();

    const auto optimal_set = graph.getOptimalPaths(1, 6);

    RulebookCost optimal_cost;
    optimal_cost.setRuleCost(0, 4);
    optimal_cost.setRuleCost(1, 4);

    assert(optimal_set.getNumElements() == 1);

    const auto all_eids = optimal_set.getAllElementIDs();
    assert(all_eids.size() == 1);

    const auto elwithcost = optimal_set.getElement(all_eids[0]);
    assert(elwithcost.cost == optimal_cost);

    const std::vector<std::shared_ptr<Edge>> expected_path = {
        graph.getEdge(3), graph.getEdge(4), graph.getEdge(5), graph.getEdge(6)};
    assert(elwithcost.element.size() == expected_path.size());

    for (size_t i = 0; i < elwithcost.element.size(); ++i) {
        assert(elwithcost.element[i] == expected_path[i]);
    }

    std::cout << "Optimal set: " << optimal_set << std::endl;
    std::cout << "Done" << std::endl;
}
