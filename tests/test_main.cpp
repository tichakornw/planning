#include <algorithm>
#include <cctype>
#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>

#include "Rulebook.h"

// From examples/grid.cpp
void testGrid();

// From test_navigation.cpp
void testNavigationCost();

// From test_world2d
void testWorld2DGeometry();

// From test_rrt_star
void testRRTStar();

// From test_state_tree
void testStateTree();

// From test_state_graph.cpp
void testStateVertexEdge();
void testStateGraph();

// From test_linear_transition.cpp
void testLinearTransition();

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

struct NamedTest {
    std::string name; // Proper capitalization
    std::function<void()> func;
};

std::string toLower(const std::string &str) {
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return result;
}

int main(int argc, char *argv[]) {
    std::vector<NamedTest> test_list = {
        {"NavigationCost", testNavigationCost},
        {"World2DGeometry", testWorld2DGeometry},
        {"RRTStar", testRRTStar},
        {"StateTree", testStateTree},
        {"StateGraph", testStateGraph},
        {"StateVertexEdge", testStateVertexEdge},
        {"LinearTransition", testLinearTransition},
        {"Graph", testGraph},
        {"WeightedGraph", testWeightedGraph},
        {"IsDAG", testIsDAG},
        {"GetPath", testGetPath},
        {"IsTotallyOrdered", testIsTotallyOrdered},
        {"Rules", testRules},
        {"Rulebook",
         [] {
             const Rulebook rulebook = testRulebook();
             testRulebookCost(rulebook);
         }},
        {"RulebookCost", testRulebookCost2},
        {"OptimalSet", testOptimalSet},
        {"Subgraph", testSubgraph},
        {"SubgraphRulebook", testSubgraphRulebook},
        {"OptimalPaths", testOptimalPaths},
        {"Avoidance", testAvoidance},
        {"Random", testRandom},
        {"Grid", testGrid}};

    std::unordered_map<std::string, std::function<void()>> test_func_map;
    std::unordered_map<std::string, std::string> test_name_map;
    for (const auto &test : test_list) {
        test_func_map[toLower(test.name)] = test.func;
        test_name_map[toLower(test.name)] = test.name;
    }

    if (argc > 1) {
        std::string arg = toLower(argv[1]);

        if (arg == "--list") {
            std::cout << "Available tests:\n";
            for (const auto &test : test_list) {
                std::cout << "  " << test.name << "\n";
            }
            return 0;
        }

        if (test_func_map.count(arg)) {
            std::cout << "Testing " << test_name_map[arg] << "...\n";
            test_func_map[arg]();
            std::cout << "Done" << std::endl;
        } else {
            std::cerr << "Unknown test: " << argv[1] << "\n";
            return 1;
        }
    } else {
        // Run all tests
        for (const auto &test : test_list) {
            std::cout << "Testing " << test.name << "...\n";
            test.func();
            std::cout << "Done" << std::endl;
        }
    }

    return 0;
}
