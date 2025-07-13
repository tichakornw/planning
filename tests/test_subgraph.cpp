#include <cassert>
#include <iostream>
#include <set>

#include "Rulebook.h"
#include "RulebookCost.h"
#include "WeightedGraph.h"

// Check that two graphs have exactly the same set of edges
template <typename CostType>
bool haveSameEdges(const WeightedGraph<CostType> &g1,
                   const WeightedGraph<CostType> &g2) {
    using EdgeTuple = std::tuple<size_t, size_t, size_t, CostType>;

    auto extractEdgeTuples = [](const WeightedGraph<CostType> &g) {
        std::vector<EdgeTuple> edgeTuples;
        for (const auto &edge : g.getEdges()) {
            auto wedge =
                std::dynamic_pointer_cast<WeightedEdge<CostType>>(edge);
            if (!wedge)
                throw std::runtime_error("Edge is not of expected type.");

            edgeTuples.emplace_back(wedge->from->vid, wedge->to->vid,
                                    wedge->eid, wedge->cost);
        }
        std::sort(edgeTuples.begin(), edgeTuples.end());
        return edgeTuples;
    };

    auto edges1 = extractEdgeTuples(g1);
    auto edges2 = extractEdgeTuples(g2);

    return edges1 == edges2;
}

void testSubgraph() {
    WeightedGraph<double> graph;

    // Add vertices
    graph.addVertex(1);
    graph.addVertex(2);
    graph.addVertex(3);
    graph.addVertex(4);
    graph.addVertex(5);
    graph.addVertex(6);

    // Add edges with costs
    graph.addEdge(1, 2, 1, 1);
    graph.addEdge(2, 3, 2, 2);
    graph.addEdge(1, 4, 2, 3);
    graph.addEdge(4, 3, 2, 4);
    graph.addEdge(1, 5, 1, 5);
    graph.addEdge(5, 6, 1, 6);
    graph.addEdge(6, 3, 1, 7);

    // Display the graph
    std::cout << "Graph:" << std::endl;
    graph.display();

    graph.reduceToOptimalSubgraph(1, 3);
    std::cout << "Optimal subgraph:" << std::endl;
    graph.display();
    assert(graph.is_consistent());

    // Define the expected edges after reduction
    std::set<std::tuple<size_t, size_t, double, size_t>> expected_edges = {
        {1, 2, 1, 1}, {2, 3, 2, 2}, {1, 5, 1, 5}, {5, 6, 1, 6}, {6, 3, 1, 7},
    };

    // Check that actual edges match expected
    std::set<std::tuple<size_t, size_t, double, size_t>> actual_edges;
    for (const auto &edge : graph.getEdges()) {
        auto wedge = std::dynamic_pointer_cast<WeightedEdge<double>>(edge);
        assert(wedge); // all edges must be WeightedEdge
        actual_edges.insert(
            {wedge->from->vid, wedge->to->vid, wedge->cost, wedge->eid});
    }

    assert(actual_edges == expected_edges);
}

void testSubgraphRulebook() {
    using WGraph = WeightedGraph<RulebookCost>;
    WGraph graph;

    // Set up Rulebook
    Rulebook rulebook;
    size_t rid_sum = rulebook.addRule(RuleSum("sumRule")); // index 0
    size_t rid_max = rulebook.addRule(RuleMax("maxRule")); // index 1
    rulebook.build();
    RulebookCost::setRulebook(rulebook);

    // Create a simple DAG
    graph.addVertex(0);
    graph.addVertex(1);
    graph.addVertex(2);
    graph.addVertex(3);

    RulebookCost c1;
    c1.setRuleCost(rid_sum, 5.0);
    c1.setRuleCost(rid_max, 10.0);
    size_t e01 = graph.addEdge(0, 1, c1);

    RulebookCost c2;
    c2.setRuleCost(rid_sum, 2.0);
    c2.setRuleCost(rid_max, 4.0);
    size_t e12 = graph.addEdge(1, 2, c2);

    RulebookCost c3;
    c3.setRuleCost(rid_sum, 4.0);
    c3.setRuleCost(rid_max, 7.0);
    size_t e03 = graph.addEdge(0, 3, c3);

    RulebookCost c4;
    c4.setRuleCost(rid_sum, 4.0);
    c4.setRuleCost(rid_max, 6.0);
    size_t e32 = graph.addEdge(3, 2, c4);

    WGraph graph_copy = graph;

    // Reduce based on RuleCostSum (index 0)
    const auto cost1 = graph.reduceToOptimalSubgraph<RuleCostSum>(
        0, 2, [](const RulebookCost &cost) {
            return cost.getSubCost<RuleCostSum>(0); // index of RuleSum
        });

    assert(cost1.getValue() == 7.0);
    assert(graph.getEdge(e03) == nullptr);
    assert(graph.getEdge(e32) == nullptr);

    // Get optimal subgraph
    auto opt_subgraph = graph_copy.extractOptimalSubgraph<RuleCostSum>(
        0, 2, [](const RulebookCost &cost) {
            return cost.getSubCost<RuleCostSum>(0); // index of RuleSum
        });

    assert(haveSameEdges(opt_subgraph, graph));

    auto path = graph.getPath(0, 2);
    // Expect path: 0 -> 1 -> 2 (sum: 5 + 2 = 7 vs. 4 + 4 = 8)
    assert(path.size() == 2);
    assert(path[0]->from->vid == 0 && path[0]->to->vid == 1);
    assert(path[1]->from->vid == 1 && path[1]->to->vid == 2);

    // Reconstruct the graph and test with RuleCostMax
    WGraph graph2;
    graph2.addVertex(0);
    graph2.addVertex(1);
    graph2.addVertex(2);
    graph2.addVertex(3);
    e01 = graph2.addEdge(0, 1, c1);
    e12 = graph2.addEdge(1, 2, c2);
    e03 = graph2.addEdge(0, 3, c3);
    e32 = graph2.addEdge(3, 2, c4);

    graph_copy = graph2;

    // Reduce based on RuleCostMax (index 1)
    const auto cost2 = graph2.reduceToOptimalSubgraph<RuleCostMax>(
        0, 2, [](const RulebookCost &cost) {
            return cost.getSubCost<RuleCostMax>(1); // index of RuleMax
        });

    assert(cost2.getValue() == 7.0);
    assert(graph2.getEdge(e01) == nullptr);
    assert(graph2.getEdge(e12) == nullptr);

    // Get optimal subgraph
    opt_subgraph = graph_copy.extractOptimalSubgraph<RuleCostMax>(
        0, 2, [](const RulebookCost &cost) {
            return cost.getSubCost<RuleCostMax>(1); // index of RuleMax
        });

    assert(haveSameEdges(opt_subgraph, graph2));

    auto path2 = graph2.getPath(0, 2);
    // Expect path: 0 -> 3 -> 2 (max: max(10,4) = 10 vs. max(7,6) = 7)
    assert(path2.size() == 2);
    assert(path2[0]->from->vid == 0 && path2[0]->to->vid == 3);
    assert(path2[1]->from->vid == 3 && path2[1]->to->vid == 2);
}
