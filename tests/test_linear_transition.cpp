#include "LinearTransition.h"
#include "Point2D.h"
#include <cassert>
#include <cmath>
#include <iostream>

void testLinearTransition() {
    using LT = LinearTransition<Point2D>;

    // Define start and end points
    Point2D p0{0.0, 0.0};
    Point2D p1{1.0, 1.0};

    // Maximum distance between samples
    double max_dist = 0.1;

    // Create LinearTransition
    LT tr(p0, p1);

    // Check length
    double expected_length = std::sqrt((p1.x - p0.x) * (p1.x - p0.x) +
                                       (p1.y - p0.y) * (p1.y - p0.y));
    assert(std::abs(tr.getLength() - expected_length) < 1e-9);

    // Check number of samples
    size_t expected_steps =
        static_cast<size_t>(std::ceil(tr.getLength() / max_dist)) + 1;
    std::vector<Point2D> samples = tr.getSamples(max_dist);
    assert(samples.size() == expected_steps);

    // Check first and last samples
    assert(samples.front().x == p0.x && samples.front().y == p0.y);
    assert(samples.back().x == p1.x && samples.back().y == p1.y);

    // Check intermediate samples spacing (roughly)
    for (size_t i = 1; i < samples.size(); ++i) {
        double dx = samples[i].x - samples[i - 1].x;
        double dy = samples[i].y - samples[i - 1].y;
        double dist = std::sqrt(dx * dx + dy * dy);
        assert(dist <= max_dist + 1e-9); // allow tiny floating point error
    }

    // Stream output
    std::cout << tr << "\n";
    std::cout << "Samples:\n";
    for (const auto &s : tr.getSamples(max_dist))
        std::cout << "  " << s << "\n";
}
