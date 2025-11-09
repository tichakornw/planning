#include "LinearTransition.h"
#include "Point2D.h"
#include "StateTree.h"
#include "test_utils.h"
#include <cassert>
#include <iostream>

void testStateTree() {
    using Tree = StateTree<Point2D, double, LinearTransition<Point2D>>;
    Tree tree;

    size_t v0 = tree.addStateVertex({0, 0}, true); // root
    size_t v1 = tree.addStateVertex({1, 0});
    size_t v2 = tree.addStateVertex({1, 1});
    size_t v3 = tree.addStateVertex({2, 1}); // deeper child

    auto t01 = LinearTransition<Point2D>({0, 0}, {1, 0});
    auto t12 = LinearTransition<Point2D>({1, 0}, {1, 1});
    auto t23 = LinearTransition<Point2D>({1, 1}, {2, 1});
    auto t02 =
        LinearTransition<Point2D>({0, 0}, {1, 1}); // shortcut for rewiring

    // Build tree: 0 -> 1 -> 2 -> 3
    tree.addStateTransition(v0, v1, t01.getLength(), t01);
    tree.addStateTransition(v1, v2, t12.getLength(), t12);
    tree.addStateTransition(v2, v3, t23.getLength(), t23);

    tree.printTree();

    // --- Check initial costs ---
    assert(approxEqual(tree.getCostToCome(v0), 0.0));
    assert(approxEqual(tree.getCostToCome(v1), t01.getLength()));
    assert(
        approxEqual(tree.getCostToCome(v2), t01.getLength() + t12.getLength()));
    assert(approxEqual(tree.getCostToCome(v3),
                       t01.getLength() + t12.getLength() + t23.getLength()));

    // --- Rewire v2 directly to root (should affect both v2 and v3) ---
    tree.rewire(v0, v2, t02.getLength(), t02);

    // After rewiring:
    // cost(v2) = cost(v0) + |t02|
    // cost(v3) = cost(v2) + |t23| (should be automatically updated)
    assert(tree.getParent(v2)->vid == v0);
    assert(approxEqual(tree.getCostToCome(v2), t02.getLength()));
    assert(
        approxEqual(tree.getCostToCome(v3), t02.getLength() + t23.getLength()));

    // --- Add another branch to confirm isolated updates ---
    size_t v4 = tree.addStateVertex({0, 2});
    auto t04 = LinearTransition<Point2D>({0, 0}, {0, 2});
    tree.addStateTransition(v0, v4, t04.getLength(), t04);

    // This branch should not be affected by rewiring
    assert(approxEqual(tree.getCostToCome(v4), t04.getLength()));
}
