#include "Rulebook.h"

// From test_graph.cpp
void testGraph();
void testWeightedGraph();
void testIsDAG();
void testGetPath();
void testIsTotallyOrdered();

// From test_rulebook.cpp
void testRules();
Rulebook testRulebook(bool verbose = false);

// From test_rulebook_cost.cpp
void testRulebookCost(const Rulebook &rulebook);
void testRulebookCost2();

// From test_optimal_set.cpp
void testOptimalSet();

// From test_subgraph.cpp
void testSubgraph();
void testSubgraphRulebook();

// From test_optimal_paths.cpp
void testOptimalPaths();

// From test_avoidance.cpp
void testAvoidance();

// From test_random.cpp
void testRandom();

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
    testAvoidance();
    testRandom();

    return 0;
}
