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

    assert(checkTopologicalSort(graph, sorted_vertices));
    std::cout << "Done"  << std::endl;
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

    graph.reduceToOptimalSubgraph<double>(1, 3, false);
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
    testRules();
    const Rulebook rulebook = testRulebook();
    testRulebookCost(rulebook);
    testRulebookCost2();
    testOptimalSet();
    testSubgraph();
    testOptimalPaths();

    return 0;
}
