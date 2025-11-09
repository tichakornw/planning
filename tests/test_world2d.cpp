#include "World2D.h"
#include "test_utils.h"
#include <cassert>
#include <cmath>
#include <iostream>

void testCircleRegion() {
    std::cout << "Running CircleRegion tests...\n";
    {
        CircleRegion c({0, 0}, 1.0);
        assert(c.contain({0, 0}));
        assert(c.contain({0.5, 0.5}));
        assert(!c.contain({2.0, 0.0}));
        std::cout << "  contain() passed.\n";
    }

    {
        CircleRegion c({0, 0}, 1.0);
        assert(approxEqual(c.distanceToPoint({0, 0}), 0.0));
        assert(approxEqual(c.distanceToPoint({1, 0}), 0.0));
        assert(approxEqual(c.distanceToPoint({2, 0}), 1.0));
        std::cout << "  distanceToPoint() passed.\n";
    }

    {
        CircleRegion c({0, 0}, 1.0);
        // Through center
        assert(approxEqual(c.distanceToLineSegment({-2, 0}, {2, 0}), 0.0));
        // Far away (distance from y=5 to radius=1)
        assert(approxEqual(c.distanceToLineSegment({-2, 5}, {2, 5}), 4.0));
        std::cout << "  distanceToLineSegment() passed.\n";
    }

    {
        CircleRegion c({0, 0}, 1.0);
        // Through center — intersects across diameter
        assert(approxEqual(c.intersectionLength({-2, 0}, {2, 0}), 2.0));
        // Tangent — should be 0
        assert(approxEqual(c.intersectionLength({-2, 1}, {2, 1}), 0.0));
        // Entirely inside
        assert(approxEqual(c.intersectionLength({-0.5, 0}, {0.5, 0}), 1.0));
        // Buffered larger radius
        assert(approxEqual(c.intersectionLength({-2, 0}, {2, 0}, 1.0), 4.0));
        std::cout << "  intersectionLength() passed.\n";
    }

    {
        CircleRegion c({0, 1}, 1.0);
        Point2D p1(-2, 0), p2(2, 0);
        double left_overlap = c.projectedOverlapLength(p1, p2, true);
        double right_overlap = c.projectedOverlapLength(p1, p2, false);
        assert(left_overlap > 0.0);
        assert(approxEqual(right_overlap, 0.0));
        std::cout << "  projectedOverlapLength() passed.\n";
    }
}

void testRectangleRegion() {
    std::cout << "Running RectangleRegion tests...\n";
    {
        RectangleRegion rect(-1, 1, -1, 1);
        assert(rect.contain({0, 0}));
        assert(rect.contain({1, -1}));
        assert(!rect.contain({2, 0}));
        std::cout << "  contain() passed.\n";
    }

    {
        RectangleRegion rect(0, 2, 0, 2);
        assert(approxEqual(rect.distanceToPoint({1, 1}), 0.0));  // inside
        assert(approxEqual(rect.distanceToPoint({3, 1}), 1.0));  // right side
        assert(approxEqual(rect.distanceToPoint({1, -1}), 1.0)); // below
        assert(approxEqual(rect.distanceToPoint({-1, -1}),
                           std::sqrt(2.0))); // bottom-left corner
        std::cout << "  distanceToPoint() passed.\n";
    }

    {
        RectangleRegion rect(0, 2, 0, 2);

        // Fully inside diagonal
        double len1 = rect.intersectionLength({0.5, 0.5}, {1.5, 1.5});
        assert(approxEqual(len1, std::sqrt(2.0)));

        // Crosses the rectangle horizontally
        double len2 = rect.intersectionLength({-1, 1}, {3, 1});
        assert(approxEqual(len2, 2.0)); // width of rect = 2

        // Completely outside (below)
        double len3 = rect.intersectionLength({-1, -1}, {3, -1});
        assert(approxEqual(len3, 0.0));

        // Touching top edge (tangent)
        double len4 = rect.intersectionLength({-1, 2}, {3, 2});
        assert(approxEqual(len4, 0.0));
        std::cout << "  intersectionLength() passed.\n";
    }
}

void testWorld2D() {
    std::cout << "Running World2D tests...\n";

    {
        World2D world(-5, 5, -5, 5);

        // Add a circular obstacle at origin with radius 1
        world.obstacles.emplace_back(Point2D(0, 0), 1.0);

        // Add a rectangular "busy" region from (2,2) to (4,4)
        world.regions.emplace_back(2, 4, 2, 4);

        // Inside obstacle
        assert(world.isInsideObstacle({0.5, 0.5}));
        // Outside obstacle
        assert(!world.isInsideObstacle({2.0, 0.0}));

        // Inside busy region
        assert(world.isInsideRegion({3.0, 3.0}));
        // Outside busy region
        assert(!world.isInsideRegion({0.0, 0.0}));

        std::cout << "  Obstacle and region containment passed.\n";
    }

    {
        World2D world(0, 10, 0, 5);

        double EPS = 1e-6;

        // Check many samples are within bounds
        for (int i = 0; i < 1000; ++i) {
            Point2D p = world.sample();
            assert(p.x >= world.xmin - EPS && p.x <= world.xmax + EPS);
            assert(p.y >= world.ymin - EPS && p.y <= world.ymax + EPS);
        }

        std::cout << "  Sampling bounds passed.\n";
    }
}

void testWorld2DGeometry() {
    testCircleRegion();
    testRectangleRegion();
    testWorld2D();
}
