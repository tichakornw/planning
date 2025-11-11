#include "ScenarioNavigation.h"
#include "RRTStarPlanner.h"
#include "World2D.h"
#include "LinearTransition.h"
#include <fstream>
#include <chrono>
#include <iostream>

void testNavigation(size_t iterations, size_t retry) {
    using StateSpace = World2D;
    using State = StateSpace::State;
    using Transition = LinearTransition<State>;
    using Cost = RulebookCost;

    // ----------------------------
    // 1. Construct World2D
    // ----------------------------
    World2D world(-5, 5, -5, 5);
    world.addObstacle({4.0, 3.0}, 1.0);        // Circular obstacle at origin
    world.addRegion(0.0, 2.0, -5.0, 1.0);    // Busy region

    State start(-4.0, -4.0);
    State goal(4.0, -4.0);
    double clearance = 0.5;

    ScenarioNavigation scenario(world, start, goal, clearance);

    // ----------------------------
    // 2. Create RRT* planner
    // ----------------------------
    RRTStarPlanner<World2D, Cost, Transition> planner(scenario);

    // ----------------------------
    // 3. Run planner and measure time
    // ----------------------------
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

    // ----------------------------
    // 4. Extract path and tree
    // ----------------------------
    auto path = planner.getOptimalPlan();
    auto tree = planner.getTree();

    assert(path.size() > 0);

    // ----------------------------
    // 5. Compute total path cost
    // ----------------------------
    RulebookCost total_cost;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        total_cost = total_cost + scenario.getCost(path[i]->transition);
    }

    std::cout << "Planning finished in " << elapsed_ms << " ms\n";
    std::cout << "Path cost: " << total_cost << "\n";

}


void testNavigation() {
    testNavigation(1000, 10);
}
