#include "LinearTransition.h"
#include "RRTStarPlanner.h"
#include "StateGraph.h"
#include "StateTree.h"
#include "World2D.h"
#include "test_utils.h"
#include <cassert>
#include <iostream>

// Simple test of RRT* in a small 2D world
void testRRTStar() {
    using State = typename World2D::State;
    using Transition = LinearTransition<State>;
    using Cost = double;

    // 1. Define a simple 2D world (no obstacles)
    World2D world(-1.0, 1.0, -1.0, 1.0);
    State start{0.0, 0.0};

    // 2. Define cost and collision functions
    auto cost_fn = [](const Transition &tr) { return tr.getLength(); };
    auto collision_fn = [&](const Transition &tr) {
        // No obstacles in this test
        return false;
    };

    // 3. Create the planner
    RRTStarPlanner<World2D, Cost, Transition> planner(&world, start, cost_fn,
                                                      collision_fn);

    // 4. Run a small number of iterations
    State goal{0.8, 0.8};
    planner.run(200);
    planner.addGoal(goal);

    // 5. Get internal tree for inspection
    auto &tree = planner.getTree();

    size_t num_vertices = tree.getNumVertices();
    std::cout << "Tree has " << num_vertices << " vertices.\n";
    assert(num_vertices > 1);

    // 6. Check that at least one vertex is close to goal
    double best_dist = 1e9;
    for (size_t i = 0; i < num_vertices; ++i) {
        auto v = tree.getStateVertex(i);
        double d = (v->state - goal).norm();
        best_dist = std::min(best_dist, d);
    }

    std::cout << "Closest vertex to goal is " << best_dist << " away.\n";
    assert(best_dist < 0.5);

    // 7. Check cost propagation: cost of any child > parent's cost
    for (size_t i = 0; i < num_vertices; ++i) {
        auto v = tree.getVertex(i);

        // Root node has no parent edge
        if (v->in_edges.empty())
            continue;

        // Each vertex in a StateTree should have exactly one parent edge
        assert(v->in_edges.size() == 1 &&
               "Each vertex must have exactly one parent edge in StateTree");

        auto edge = tree.getParentEdge(i);
        assert(edge && "Edge must be a StateTransitionEdge");

        auto parent = edge->from;
        double c_parent = tree.getCostToCome(parent->vid);
        double c_child = tree.getCostToCome(v->vid);
        double expected = c_parent + edge->cost;

        // Check cost monotonicity
        assert(c_child >= c_parent - 1e-6);

        // Check cost propagation consistency
        assert(approxEqual(c_child, expected));
    }
}
