#include <cassert>
#include <chrono>
#include <iostream>

#include "RulebookPlanner.h"
#include "ScenarioRandom.h"

void plan(ScenarioRandom &scenario) {
    RulebookPlanner planner(scenario);
    auto start = std::chrono::high_resolution_clock::now();
    planner.getOptimalPlans();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "  planning time (ms): " << duration.count() << std::endl;
    ;
}

void testRandom() {
    std::cout << "Testing Random Scenario ..." << std::endl;

    const size_t max_grid = 8;
    const size_t max_rules = 5;

    for (size_t num_x = 5; num_x < max_grid; ++num_x) {
        for (size_t num_y = 5; num_y < max_grid; ++num_y) {
            ScenarioRandom scenario(num_x, num_y, 5);
            plan(scenario);
        }
    }

    for (size_t num_rules = 1; num_rules <= max_rules; ++num_rules) {
        ScenarioRandom scenario(10, 10, num_rules);
        plan(scenario);
    }

    std::cout << "Done" << std::endl;
}
