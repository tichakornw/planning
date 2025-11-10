#include "LinearTransition.h"
#include "ScenarioNavigation.h"
#include "World2D.h"
#include "test_utils.h"
#include <iostream>

void testNavigationCost() {
    using StateSpace = World2D;
    using State = StateSpace::State;
    using Transition = LinearTransition<State>;

    // ----------------------------
    // 1. Create world
    // ----------------------------
    StateSpace world(-5.0, 5.0, -5.0, 5.0);

    // Add one circular obstacle (center = (0,0), radius = 1)
    world.addObstacle({0.0, 0.0}, 1.0);

    // Add one busy region: rectangle from (-2,-2) to (2,-1)
    world.addRegion({-2.0, 2.0, -2.0, -1.0});

    // ----------------------------
    // 2. Define start and goal
    // ----------------------------
    State start(-4.0, -4.0);
    State goal(4.0, 4.0);
    double clearance = 0.5;

    // ----------------------------
    // 3. Create scenario
    // ----------------------------
    ScenarioNavigation scenario(world, start, goal, clearance);

    std::cout << "ScenarioNavigation created successfully.\n";
    std::cout << "World has " << world.obstacles.size() << " obstacle(s) and "
              << world.regions.size() << " region(s).\n";

    // ----------------------------
    // 4. Test cost function
    // ----------------------------
    Transition t1(start, goal);
    auto cost_fn = scenario.costFn();

    RulebookCost cost = cost_fn(t1);
    std::cout << "Cost summary:\n";
    std::cout << "  total cost = " << cost << "\n";

    assert(approxEqual(cost.getRuleCost(0), 2.0));
    assert(approxEqual(cost.getRuleCost(1), 1.41421, 1e-3));
    assert(approxEqual(cost.getRuleCost(2), 3.0));
    assert(approxEqual(cost.getRuleCost(3), 2.0));
    assert(approxEqual(cost.getRuleCost(4), 11.3137, 1e-3));

    // ----------------------------
    // 5. Optional: test collisionFn (should be false by default)
    // ----------------------------
    auto coll_fn = scenario.collisionFn();
    bool collides = coll_fn(t1);
    std::cout << "Collision detected? " << (collides ? "yes" : "no") << "\n";

    assert(collides == false &&
           "Collision function should return false by default");
}
