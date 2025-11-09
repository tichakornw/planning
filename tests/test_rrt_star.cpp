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
    // 1. Define a simple 2D world (no obstacles)
    World2D world(-1.0, 1.0, -1.0, 1.0);

    using State = typename World2D::State;
    using Transition = LinearTransition<State>;
    using Cost = double;

    using SVertex = StateVertex<State>;

    // 2. Define cost and collision functions
    auto cost_fn = [](const Transition &tr) { return tr.getLength(); };
    auto collision_fn = [&](const Transition &tr) {
        // No obstacles in this test
        return false;
    };

    // 3. Create the planner
    RRTStarPlanner<World2D, Cost, Transition> planner(&world, cost_fn,
                                                      collision_fn);

    // 4. Initialize tree at origin
    State start{0.0, 0.0};
    planner.initialize(start);

    // 5. Run a small number of iterations
    State goal{0.8, 0.8};
    planner.run(goal, 200);

    // 6. Get internal tree for inspection
    auto &tree = planner.getTree();

    size_t num_vertices = tree.getNumVertices();
    std::cout << "Tree has " << num_vertices << " vertices.\n";
    assert(num_vertices > 1);

    // 7. Check that at least one vertex is close to goal
    double best_dist = 1e9;
    for (size_t i = 0; i < num_vertices; ++i) {
        auto v = tree.getStateVertex(i);
        double d = (v->state - goal).norm();
        best_dist = std::min(best_dist, d);
    }

    std::cout << "Closest vertex to goal is " << best_dist << " away.\n";
    assert(best_dist < 0.5);

    // 8. Check cost propagation: cost of any child > parent's cost
    for (size_t i = 0; i < num_vertices; ++i) {
        auto v = tree.getVertex(i);

        // Root node has no parent edge
        if (v->in_edges.empty())
            continue;

        // Each vertex in a StateTree should have exactly one parent edge
        assert(v->in_edges.size() == 1 &&
               "Each vertex must have exactly one parent edge in StateTree");

        auto edge = std::dynamic_pointer_cast<
            StateTransitionEdge<Point2D, double, LinearTransition<Point2D>>>(
            v->in_edges.front());
        assert(edge && "Edge must be a StateTransitionEdge");

        auto parent = std::dynamic_pointer_cast<SVertex>(edge->from);
        double c_parent = tree.getCostToCome(parent->vid);
        double c_child = tree.getCostToCome(v->vid);
        double expected = c_parent + edge->cost;

        // Check cost monotonicity
        assert(c_child >= c_parent - 1e-6);

        // Check cost propagation consistency
        assert(approxEqual(c_child, expected));
    }
}
