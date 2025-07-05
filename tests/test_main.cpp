#include <cassert>
#include <vector>
#include <set>

#include "Graph.h"
#include "OptimalSet.h"
#include "Plan.h"
#include "Rule.h"
#include "RuleCost.h"
#include "Rulebook.h"
#include "RulebookCost.h"
#include "WeightedGraph.h"


// Check that sort_vertices is a topological sort of graph
bool checkTopologicalSort(const Graph& graph, const std::vector<size_t>& sorted_vertices) {
    // Build a map from vertex ID to its index in the sorted list
    std::unordered_map<size_t, size_t> vertex_order;
    for (size_t i = 0; i < sorted_vertices.size(); ++i) {
        vertex_order[sorted_vertices[i]] = i;
    }

    // Ensure that each edge's source comes before its destination
    for (const auto& edge : graph.getEdges()) {
        size_t from_id = edge->from->vid;
        size_t to_id = edge->to->vid;

        if (vertex_order[from_id] >= vertex_order[to_id]) {
            std::cerr << "Invalid topological sort: "
                      << from_id << " appears after " << to_id << std::endl;
            return false;
        }
    }

    // Check that sort includes all vertices
    if (sorted_vertices.size() != graph.getNumVertices()) {
        std::cerr << "Incorrect size: " << sorted_vertices.size() << " vs " << graph.getNumVertices()
                  << std::endl;
        return false;
    }

    return true;
}

// Check that two graphs have exactly the same set of edges
template <typename CostType>
bool haveSameEdges(const WeightedGraph<CostType>& g1, const WeightedGraph<CostType>& g2) {
        using EdgeTuple = std::tuple<size_t, size_t, size_t, CostType>;

    auto extractEdgeTuples = [](const WeightedGraph<CostType>& g) {
        std::vector<EdgeTuple> edgeTuples;
        for (const auto& edge : g.getEdges()) {
            auto wedge = std::dynamic_pointer_cast<WeightedEdge<CostType>>(edge);
            if (!wedge)
                throw std::runtime_error("Edge is not of expected type.");

            edgeTuples.emplace_back(
                wedge->from->vid,
                wedge->to->vid,
                wedge->eid,
                wedge->cost
            );
        }
        std::sort(edgeTuples.begin(), edgeTuples.end());
        return edgeTuples;
    };

    auto edges1 = extractEdgeTuples(g1);
    auto edges2 = extractEdgeTuples(g2);

    return edges1 == edges2;
}

void testGraph() {
    std::cout << "Testing Graph ..." << std::endl;
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
    std::cout << "Done" << std::endl;
}

void testWeightedGraph() {
    std::cout << "Testing WeightedGraph ..." << std::endl;
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
    std::cout << "Done"  << std::endl;
}

void testIsDAG() {
    std::cout << "Testing graph.isDAG ..." << std::endl;
    Graph g;

    // DAG: 1 -> 2 -> 3
    g.addVertex(1); g.addVertex(2); g.addVertex(3);
    g.addEdge(1, 2, 1);
    g.addEdge(2, 3, 2);

    assert(g.isDAG());

    // Add a cycle: 3 -> 1
    g.addEdge(3, 1, 3);
    assert(!g.isDAG());

    std::cout << "Done" << std::endl;
}

void testGetPath() {
    std::cout << "Testing graph.getPath ..." << std::endl;
    Graph g;

    g.addVertex(1); g.addVertex(2); g.addVertex(3); g.addVertex(4);
    g.addEdge(1, 2, 1);
    g.addEdge(2, 3, 2);
    g.addEdge(3, 4, 3);

    auto path = g.getPath(1, 4);
    std::vector<size_t> expected = {1, 2, 3, 4};

    assert(path.size() == expected.size() - 1);  // 3 edges for 4 vertices

    for (size_t i = 0; i < path.size(); ++i) {
        assert(path[i]->from->vid == expected[i]);
        assert(path[i]->to->vid == expected[i + 1]);
    }

    auto no_path = g.getPath(4, 1);
    assert(no_path.empty());

    std::cout << "Done" << std::endl;
}

void testIsTotallyOrdered() {
    std::cout << "Testing graph.isTotallyOrdered ..." << std::endl;
    Graph g;

    // Total order: 10 -> 20 -> 30 -> 40
    g.addVertex(10); g.addVertex(20); g.addVertex(30); g.addVertex(40);
    g.addEdge(10, 20, 1);
    g.addEdge(20, 30, 2);
    g.addEdge(30, 40, 3);

    assert(g.isDAG());
    assert(g.isTotallyOrdered());

    // Break total order by adding a parallel branch
    g.addVertex(50);
    g.addEdge(20, 50, 4);  // Now 20 points to two nodes: 30 and 50

    assert(g.isDAG());
    assert(!g.isTotallyOrdered());

    std::cout << "Done" << std::endl;
}

void testRules() {
    std::cout << "Testing Rules ..." << std::endl;
    RuleSum rsum("sumRule");
    auto a_ptr = rsum.makeCost(3.0);
    auto b_ptr = rsum.makeCost(4.0);

    const RuleCost& a = *a_ptr;
    const RuleCost& b = *b_ptr;

    const auto c = a + b;
    std::cout << "c = " << *c << std::endl;
    assert(c->getValue() == 7.0);

    RuleMax rmax("maxRule");
    auto x_ptr = rmax.makeCost(2.5);
    auto y_ptr = rmax.makeCost(9.1);
    const RuleCost& x = *x_ptr;
    const RuleCost& y = *y_ptr;

    const auto z = x + y;
    std::cout << "z = " << *z << std::endl;
    assert(z->getValue() == 9.1);

    auto w_ptr = rmax.makeCost(1.2);
    const RuleCost& w = *w_ptr;
    const RuleCost& v = *z;
    const auto u = v + w;
    assert(u->getValue() == 9.1);
    std::cout << "Done"  << std::endl;
}

Rulebook testRulebook(bool verbose = false) {
    std::cout << "Testing Rulebook ..." << std::endl;
    Rulebook rulebook;

    RuleSum r0("r0");
    RuleMax r1("r1");
    RuleSum r2("r2");

    rulebook.addRule(r0);
    rulebook.addRule(r1);
    rulebook.addRule(r2);
    size_t r3id = rulebook.addRule(RuleMax("r3"));

    assert(r3id > 2);
    assert(rulebook.addRule(r2));
    assert(rulebook.addRule(r1));

    rulebook.setEquivalentClasses({
            {1, 2, r3id},
            {0, 4}
        });

    bool exceptionThrown = false;
    try {
        rulebook.addGTRelation(2, 6);
    } catch (const std::exception& e) {
        exceptionThrown = true;
    }
    assert(exceptionThrown && "Expected exception not thrown by addGTRelation(2, 7)");
    try {
        rulebook.addGTRelation(2, r3id);
    } catch (const std::exception& e) {
        exceptionThrown = true;
    }
    assert(exceptionThrown && "Expected exception not thrown by addGTRelation(2, r3id)");

    rulebook.addGTRelation(2, 5);
    rulebook.addGTRelation(1, 4);

    rulebook.build();

    rulebook.display();

    std::vector<std::unordered_set<size_t>> classes;
    if (verbose)
        std::cout << "Order: ";
    for (auto it = rulebook.begin(); it != rulebook.end(); ++it) {
        std::unordered_set<size_t> s(it->begin(), it->end());
        classes.push_back(s);
        if (verbose) {
            std::cout << "{ ";
            for (auto elem : *it) {
                std::cout << elem << " ";
            std::cout << "} ";
            }
        }
    }
    if (verbose)
        std::cout << std::endl;
    // There should be exactly 3 sets
    assert(classes.size() == 3);

    // Check that the first set is exactly {1, 2, r3id}, the second set is either {0, 5} or {6}
    std::unordered_set<size_t> set04 = {0, 4};
    std::unordered_set<size_t> set5 = {5};
    assert(classes[0] == std::unordered_set<size_t>({1, 2, r3id}));
    assert((classes[1] == set04 and classes[2] == set5) or (classes[1] == set5 and classes[2] == set04));

    // Successors
    std::unordered_set<size_t> set045 = {0, 4, 5};
    std::unordered_set<size_t> set_empty;
    if (verbose)
        std::cout << "Successors:" << std::endl;
    for (size_t i = 0; i < 8; ++i) {
        const auto successors = rulebook.getSuccessors(i);
        if (i != 1 && i != 2 && i != r3id)
            assert(successors == set_empty);
        else
            assert(successors == set045);
        if (verbose) {
            std::cout << "  " << i << ": ";
            for (auto j : successors) {
                std::cout << j << " ";
            }
            std::cout << std::endl;
        }
    }
    std::cout << "Done"  << std::endl;
    return rulebook;
}

void testRulebookCost(const Rulebook &rulebook) {
    std::cout << "Testing RulebookCost ..." << std::endl;
    RulebookCost::setRulebook(rulebook);
    RulebookCost cost1;
    RulebookCost cost2;
    cost1.setRuleCost(0, 2);
    cost1.setRuleCost(1, 1);
    cost2.setRuleCost(1, 2);
    cost2.setRuleCost(0, 1);
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

    RulebookCost cost3 = cost1 + cost2;
    assert(cost3[0]->getValue() == 3);
    assert(cost3[1]->getValue() == 2);

    std::cout << "Done"  << std::endl;
}

void testRulebookCost2() {
    std::cout << "Testing RulebookCost2 ..." << std::endl;

    Rulebook rulebook;
    rulebook.addRule(RuleSum("r0"));
    rulebook.addRule(RuleSum("r1"));
    rulebook.build();

    RulebookCost::setRulebook(rulebook);
    // Add edges with costs
    RulebookCost cost44;
    cost44.setRuleCost(0, 4);
    cost44.setRuleCost(1, 4);

    RulebookCost cost1111;
    cost1111.setRuleCost(0, 11);
    cost1111.setRuleCost(1, 11);

    assert(cost44 <= cost1111);
    assert(cost44 < cost1111);
    std::cout << "Done"  << std::endl;
}

void testOptimalSet() {
    std::cout << "Testing OptimalSet ..." << std::endl;
    OptimalSet<Plan, double> optimal_set;
    std::vector<int> element1 = {1, 2, 3};
    std::vector<int> element2 = {2, 2, 4};
    std::vector<int> element3 = {2, 3, 4};
    std::vector<int> element4 = {2, 3, 4};

    Plan p1(element1);
    Plan p2(element2);
    Plan p3(element3);
    Plan p4(element4);

    optimal_set.insert(p1, 10.0, 1);
    optimal_set.insert(p1, 8.0, 2);
    optimal_set.insert(p2, 5.0, 3);
    optimal_set.insert(p3, 15.0, 4);
    optimal_set.insert(p3, 5.0, 5);

    assert(optimal_set.getNumElements() == 2);
    assert(optimal_set.isIn(3));
    assert(optimal_set.isIn(5));

    const auto el3 = optimal_set.getElement(3);
    const auto el5 = optimal_set.getElement(5);
    assert(el3.element == p2);
    assert(el5.element == p3);
    assert(el3.cost == 5.0);
    assert(el5.cost == 5.0);
    assert(el3.eid == 3);
    assert(el5.eid == 5);

    // Print the elements in the optimal set
    std::cout << optimal_set << std::endl;
    std::cout << "Done"  << std::endl;
}

void testSubgraph() {
    std::cout << "Testing Subgraph ..." << std::endl;
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

    graph.reduceToOptimalSubgraph<double>(1, 3);
    std::cout << "Optimal subgraph:" << std::endl;
    graph.display();
    assert(graph.is_consistent());

    // Define the expected edges after reduction
    std::set<std::tuple<size_t, size_t, double, size_t>> expected_edges = {
        {1, 2, 1, 1},
        {2, 3, 2, 2},
        {1, 5, 1, 5},
        {5, 6, 1, 6},
        {6, 3, 1, 7},
    };

    // Check that actual edges match expected
    std::set<std::tuple<size_t, size_t, double, size_t>> actual_edges;
    for (const auto& edge : graph.getEdges()) {
        auto wedge = std::dynamic_pointer_cast<WeightedEdge<double>>(edge);
        assert(wedge); // all edges must be WeightedEdge
        actual_edges.insert({wedge->from->vid, wedge->to->vid, wedge->cost, wedge->eid});
    }

    assert(actual_edges == expected_edges);
    std::cout << "Done"  << std::endl;
}

void testSubgraphRulebook() {
    std::cout << "Testing Subgraph with Rulebook ..." << std::endl;
    using WGraph = WeightedGraph<RulebookCost>;
    WGraph graph;

    // Set up Rulebook
    Rulebook rulebook;
    size_t rid_sum = rulebook.addRule(RuleSum("sumRule"));   // index 0
    size_t rid_max = rulebook.addRule(RuleMax("maxRule"));   // index 1
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
        0, 2, [](const RulebookCost& cost) {
            return cost.getSubCost<RuleCostSum>(0);  // index of RuleSum
        });

    assert(cost1.getValue() == 7.0);
    assert(graph.getEdge(e03) == nullptr);
    assert(graph.getEdge(e32) == nullptr);

    // Get optimal subgraph
    auto opt_subgraph = graph_copy.extractOptimalSubgraph<RuleCostSum>(
        0, 2, [](const RulebookCost& cost) {
            return cost.getSubCost<RuleCostSum>(0);  // index of RuleSum
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
        0, 2, [](const RulebookCost& cost) {
            return cost.getSubCost<RuleCostMax>(1);  // index of RuleMax
        });

    assert(cost2.getValue() == 7.0);
    assert(graph2.getEdge(e01) == nullptr);
    assert(graph2.getEdge(e12) == nullptr);

    // Get optimal subgraph
    opt_subgraph = graph_copy.extractOptimalSubgraph<RuleCostMax>(
        0, 2, [](const RulebookCost& cost) {
            return cost.getSubCost<RuleCostMax>(1);  // index of RuleMax
        });

    assert(haveSameEdges(opt_subgraph, graph2));

    auto path2 = graph2.getPath(0, 2);
    // Expect path: 0 -> 3 -> 2 (max: max(10,4) = 10 vs. max(7,6) = 7)
    assert(path2.size() == 2);
    assert(path2[0]->from->vid == 0 && path2[0]->to->vid == 3);
    assert(path2[1]->from->vid == 3 && path2[1]->to->vid == 2);

    std::cout << "Done" << std::endl;
}

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
        graph.getEdge(3),
        graph.getEdge(4),
        graph.getEdge(5),
        graph.getEdge(6)
    };
    assert(elwithcost.element.size() == expected_path.size());

    for (size_t i = 0; i < elwithcost.element.size(); ++i) {
        assert(elwithcost.element[i] == expected_path[i]);
    }

    std::cout << "Optimal set: " << optimal_set << std::endl;
    std::cout << "Done"  << std::endl;
}

int main() {
    testGraph();
    testWeightedGraph();
    testIsDAG();
    testGetPath();
    testIsTotallyOrdered();
    testRules();
    const Rulebook rulebook = testRulebook();
    testRulebookCost(rulebook);
    testRulebookCost2();
    testOptimalSet();
    testSubgraph();
    testSubgraphRulebook();
    testOptimalPaths();

    return 0;
}
