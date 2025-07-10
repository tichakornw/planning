#include <cassert>
#include <chrono>
#include <iostream>
#include <set>
#include <vector>

#include "OptimalSet.h"
#include "RulebookCost.h"
#include "RulebookPlanner.h"
#include "ScenarioGrid.h"
#include "WeightedGraph.h"

void testGrid() {
    std::cout << "Testing grid world ..." << std::endl;
    ScenarioGrid scenario;
    RulebookPlanner planner(scenario);
    planner.setDebug(true);
    auto start = std::chrono::high_resolution_clock::now();
    auto edge_path =
        planner.getOptimalPlan(scenario.getVidQinit(), scenario.getVidQgoal());

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::milliseconds>(stop - start);

    const auto actual_optimal_path = scenario.getPStatePath(edge_path);
    std::cout << "Path: ";
    for (const auto &state : actual_optimal_path)
        std::cout << state << " ";
    std::cout << std::endl;
    std::cout << "Done" << std::endl;
}
