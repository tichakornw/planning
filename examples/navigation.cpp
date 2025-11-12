#include <chrono>
#include <cassert>
#include <iostream>
#include "LinearTransition.h"
#include "World2D.h"
#include "ScenarioNavigation.h"
#include "RRTStarPlanner.h"
#include "StateGraph.h"
#include "json_utils.h"


// -----------------------------------------------------------------------------
// Set up the world and scenario, and saves the world JSON
// -----------------------------------------------------------------------------
ScenarioNavigation setupNavigationWorld(const std::string &world_filename)
{
    using StateSpace = World2D;
    using State = StateSpace::State;

    World2D world(-5, 5, -5, 5);
    world.addObstacle({4.0, 3.0}, 2.0);        // Circular obstacle at origin
    world.addObstacle({-2.0, -4.0}, 1.0);
    world.addRegion(0.0, 2.0, -5.0, 1.0);    // Busy region

    State start(-4.0, -4.0);
    State goal(4.0, -4.0);
    double clearance = 0.5;

    // Save world JSON immediately
    saveWorldJson(world_filename, world, clearance);

    // Build and return scenario
    ScenarioNavigation scenario(world, start, goal, clearance);
    return scenario;
}

// -----------------------------------------------------------------------------
// Run the planner and returns path, tree, and timing
// -----------------------------------------------------------------------------
auto runNavigationPlanning(const ScenarioNavigation &scenario,
                           size_t iterations,
                           size_t retry,
                           const std::string &plan_filename)
{
    using StateSpace = World2D;
    using State = StateSpace::State;
    using Transition = LinearTransition<State>;
    using Cost = RulebookCost;

    RRTStarPlanner<StateSpace, Cost, Transition> planner(scenario);

    // Run planner and measure time
    auto t0 = std::chrono::high_resolution_clock::now();
    planner.run(iterations); // max iterations = 1000
    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    size_t num_trial = 0;
    while (!planner.addGoal(scenario.getGoalState()) && num_trial < retry) {
        t0 = std::chrono::high_resolution_clock::now();
        planner.run(iterations);
        t1 = std::chrono::high_resolution_clock::now();
        elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    }

    // Extract path and tree
    auto path = planner.getOptimalPlan();
    const auto &tree = planner.getTree();

    assert(path.size() > 0);

    // Compute total path cost
    Cost total_cost;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        total_cost = total_cost + scenario.getCost(path[i]->transition);
    }

    std::cout << "Planning finished in " << elapsed_ms << " ms\n";
    std::cout << "Path cost: " << total_cost << "\n";
    savePlanningResultJson(plan_filename, tree, path, scenario.getInitState(), scenario.getGoalState(), elapsed_ms, total_cost);

    return std::tuple(total_cost, elapsed_ms);
}

// -----------------------------------------------------------------------------
// Main test function
// -----------------------------------------------------------------------------
void testNavigation(size_t iterations, size_t retry)
{
    std::string world_file = "results/world.json";
    std::string plan_file = "results/navigation.json";

    auto scenario = setupNavigationWorld(world_file);

    const auto &[total_Cost, elapsed_ms] =
        runNavigationPlanning(scenario, iterations, retry, plan_file);
}

void testNavigation() {
    testNavigation(1000, 10);
}
