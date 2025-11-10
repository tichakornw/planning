#pragma once
#include "Point2D.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <random>
#include <vector>

/**
 * @brief Represents a circular region in 2D space.
 *
 * Provides geometric queries such as containment, distances,
 * and intersection overlap with line segments.
 */
class CircleRegion {
  public:
    Point2D center; ///< Center of the circle
    double radius;  ///< Radius of the circle

    /// Construct a circle centered at `c` with radius `r`
    CircleRegion(const Point2D &c = {}, double r = 0.0)
        : center(c), radius(r) {}

    /**
     * @brief Check whether a point lies inside or on the circle.
     * @param p Query point
     * @return true if distance(center, p) ≤ radius
     */
    bool contain(const Point2D &p) const {
        return center.distance(p) <= radius;
    }

    /**
     * @brief Compute the shortest Euclidean distance from a point to the circle
     * boundary.
     *
     * Returns zero if the point lies inside or on the circle.
     */
    double distanceToPoint(const Point2D &p) const {
        return std::max(0.0, center.distance(p) - radius);
    }

    /**
     * @brief Compute the shortest distance from the circle to a finite line
     * segment.
     *
     * Projects the circle center onto the segment (p1,p2),
     * clamps the projection to stay within [p1,p2],
     * and computes the distance from the circle boundary to that closest point.
     */
    double distanceToLineSegment(const Point2D &p1, const Point2D &p2) const {
        // Standard point-to-segment distance formula
        auto d = p2 - p1;
        double len2 = d.norm2();

        if (len2 < 1e-9)
            return distanceToPoint(p1);

        double t = ((center - p1).dot(d)) / len2;
        t = std::clamp(t, 0.0, 1.0);
        Point2D projection = p1 + d * t;
        return distanceToPoint(projection);
    }

    /**
     * @brief Compute the length of a line segment that lies inside (or
     * intersects) this circle.
     *
     * The line segment is defined by its endpoints p1 and p2.
     * Optionally, a positive `buffer` can be provided to enlarge the circle
     * (i.e., an effective radius of `radius + buffer` is used).
     *
     * The function computes where the infinite line passing through (p1, p2)
     * intersects the circle. It then determines the portion of the finite
     * segment [p1, p2] that lies inside the circle (or buffered circle), and
     * returns the geometric length of that portion.
     *
     * @param p1     Start point of the segment
     * @param p2     End point of the segment
     * @param buffer Optional positive margin added to the circle radius
     * @return Length of the segment inside the (buffered) circle
     */
    double intersectionLength(const Point2D &p1, const Point2D &p2,
                              double buffer = 0.0) const {
        assert(buffer >= 0.0);

        // Effective radius (expanded by buffer distance)
        const double r = radius + buffer;

        // Direction vector of the segment
        auto d = p2 - p1;
        double len = d.norm();
        if (len < 1e-9)
            return contain(p1) ? len : 0.0;

        // Quadratic coefficients for intersection with circle
        // Equation: |p1 + t*d - center|^2 = r^2,  with t in [0,1]
        double a = d.dot(d);
        double b = 2.0 * ((p1.x - center.x) * d.x + (p1.y - center.y) * d.y);
        double c = (p1.x - center.x) * (p1.x - center.x) +
                   (p1.y - center.y) * (p1.y - center.y) - r * r;

        double disc = b * b - 4 * a * c;
        const double EPS = 1e-12;

        // Case 1: No real roots (disc < 0) -> segment is either entirely
        // outside or entirely inside the circle. Check midpoint to decide.
        if (disc < -EPS) {
            Point2D mid = p1 + (d * 0.5);
            return contain(mid) ? len : 0.0;
        }

        // Case 2: Tangent (disc approximately zero). Tangent point contributes
        // zero length intersection (a single point).
        if (std::fabs(disc) <= EPS) {
            return 0.0;
        }

        // Case 3: Compute intersection parameters (t1, t2) along the line
        double sqrt_disc = std::sqrt(disc);
        double t1 = (-b - sqrt_disc) / (2 * a);
        double t2 = (-b + sqrt_disc) / (2 * a);

        // Clamp to [0,1] since we only care about the segment portion
        double tmin = std::max(0.0, std::min(t1, t2));
        double tmax = std::min(1.0, std::max(t1, t2));

        // Case 4: Intersection interval lies outside the segment
        // If the clamped interval is degenerate or outside segment, check
        // midpoint:
        if (tmax <= tmin + EPS) {
            // Degenerate overlap (effectively zero length).
            // But if the entire segment is inside (unlikely here since disc>0),
            // fallback to midpoint check for safety.
            Point2D mid = p1 + d * 0.5;
            return contain(mid) ? len : 0.0;
        }

        // Case 5: Segment overlaps with circle between [tmin, tmax]
        double overlap_len = (tmax - tmin) * len;
        return overlap_len;
    }

    /**
     * @brief Compute how much of a segment overlaps with the *projection
     * footprint* of the circle onto the line, conditioned on which side of the
     * line the circle lies.
     *
     * The function projects the circle onto the line defined by (p1,p2).
     * If the circle center lies on the requested side (left/right), it returns
     * the projected overlap length along the line segment that corresponds
     * to the circle’s projection interval.
     *
     * @param p1 Start point of the line segment
     * @param p2 End point of the line segment
     * @param left_of_line If true, return overlap only if circle is on left
     * side
     * @return Length of overlap on requested side (0 if none)
     */
    double projectedOverlapLength(const Point2D &p1, const Point2D &p2,
                                  bool left_of_line) const {
        auto d = p2 - p1;
        double len = d.norm();
        if (len < 1e-9)
            return 0.0;

        // Unit direction along the line
        Vector2D d_unit = d * (1 / len);

        // Left-hand normal of the line
        Vector2D n(-d_unit.y, d_unit.x);

        // Signed perpendicular distance from circle center to line
        double s = (center - p1).dot(n);

        // Project circle center onto the line
        double proj_center = (center - p1).dot(d_unit);

        // Projected interval of the circle on the infinite line
        double t1 = proj_center - radius;
        double t2 = proj_center + radius;

        // Clamp to finite segment [0, len]
        double start = std::max(0.0, t1);
        double end = std::min(len, t2);

        // No overlap
        if (end <= start)
            return 0.0;

        double overlap = end - start;

        // Determine which side of the line the circle center lies on
        bool circle_on_left = (s > 0);

        // Return the overlap only if the circle is on the queried side
        bool relevant_side = (left_of_line && circle_on_left) ||
                             (!left_of_line && !circle_on_left);
        return relevant_side ? overlap : 0.0;
    }
};

/**
 * @brief Axis-aligned rectangular region in 2D.
 *
 * Defined by its min/max x and y bounds.
 * Provides methods for containment, distance, and segment intersection.
 */
class RectangleRegion {
  public:
    double xmin, xmax, ymin, ymax;

    RectangleRegion(double xmin = 0, double xmax = 0, double ymin = 0,
                    double ymax = 0)
        : xmin(xmin), xmax(xmax), ymin(ymin), ymax(ymax) {}

    /**
     * @brief Check whether a point lies inside or on the boundary of the
     * rectangle.
     */
    bool contain(const Point2D &p) const {
        return p.x >= xmin && p.x <= xmax && p.y >= ymin && p.y <= ymax;
    }

    /**
     * @brief Compute the shortest Euclidean distance from a point to the
     * rectangle.
     *
     * If the point is inside the rectangle, returns 0.
     */
    double distanceToPoint(const Point2D &p) const {
        double dx = std::max({xmin - p.x, 0.0, p.x - xmax});
        double dy = std::max({ymin - p.y, 0.0, p.y - ymax});
        return std::sqrt(dx * dx + dy * dy);
    }

    /**
     * @brief Compute the length of a line segment that lies inside the
     * rectangle.
     *
     * Uses the Liang–Barsky line clipping algorithm for efficient computation.
     *
     * @param p1 Start point of the segment
     * @param p2 End point of the segment
     * @return Length of the portion of the segment inside the rectangle
     */
    double intersectionLength(const Point2D &p1, const Point2D &p2) const {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double t0 = 0.0;
        double t1 = 1.0;

        // progressively narrows down the valid parameter range [t0, t1]
        // where the segment lies inside the rectangle
        auto clip = [&](double p, double q) -> bool {
            if (std::fabs(p) < 1e-12) {
                // Line is parallel to this edge
                // If q < 0 → outside; if q == 0 → exactly on the edge → no
                // interior overlap
                if (q <= 0)
                    return false; // outside or exactly on edge
                return true;      // inside region (strictly)
            }
            double r = q / p;
            if (p < 0) {
                if (r > t1)
                    return false;
                if (r > t0)
                    t0 = r;
            } else if (p > 0) {
                if (r < t0)
                    return false;
                if (r < t1)
                    t1 = r;
            }
            return true;
        };

        if (!clip(-dx, p1.x - xmin))
            return 0.0; // left
        if (!clip(dx, xmax - p1.x))
            return 0.0; // right
        if (!clip(-dy, p1.y - ymin))
            return 0.0; // bottom
        if (!clip(dy, ymax - p1.y))
            return 0.0; // top

        if (t1 < t0)
            return 0.0;

        double segment_length = std::sqrt(dx * dx + dy * dy);
        double inside_length = (t1 - t0) * segment_length;
        return inside_length;
    }
};

/**
 * @brief Represents a 2D world with circular obstacles and rectangular regions.
 *
 * The world is bounded by [xmin, xmax] × [ymin, ymax].
 * It supports queries for whether a point lies inside an obstacle or region,
 * and can sample random points uniformly within its boundaries.
 */
class World2D {
  public:
    using State = Point2D; ///< Defines the type of state used by this space

    std::vector<CircleRegion> obstacles; ///< Circular obstacles to avoid
    std::vector<RectangleRegion>
        regions;                   ///< Rectangular areas (e.g., busy zones)
    double xmin, xmax, ymin, ymax; ///< World boundaries

    /**
     * @brief Construct a bounded 2D world.
     *
     * @param xmin_ Minimum x boundary
     * @param xmax_ Maximum x boundary
     * @param ymin_ Minimum y boundary
     * @param ymax_ Maximum y boundary
     */
    World2D(double xmin_, double xmax_, double ymin_, double ymax_)
        : xmin(xmin_), xmax(xmax_), ymin(ymin_), ymax(ymax_) {}

    /**
     * @brief Add a circular obstacle to the world.
     * @param center Center point of the obstacle.
     * @param radius Radius of the obstacle.
     */
    void addObstacle(const Point2D &center, double radius) {
        obstacles.emplace_back(center, radius);
    }

    /**
     * @brief Add a rectangular region (e.g., busy or restricted zone).
     * @param xmin Minimum x boundary.
     * @param ymin Minimum y boundary.
     * @param xmax Maximum x boundary.
     * @param ymax Maximum y boundary.
     */
    void addRegion(double xmin, double xmax, double ymin, double ymax) {
        regions.emplace_back(xmin, xmax, ymin, ymax);
    }

    /**
     * @brief Add a rectangular region using an existing RectangleRegion object.
     */
    void addRegion(const RectangleRegion &region) { regions.push_back(region); }

    /**
     * @brief Check if a point lies inside any circular obstacle.
     */
    bool isInsideObstacle(const State &p) const {
        for (const auto &c : obstacles)
            if (c.contain(p))
                return true;
        return false;
    }

    /**
     * @brief Check if a point lies inside any rectangular region.
     */
    bool isInsideRegion(const State &p) const {
        for (const auto &r : regions)
            if (r.contain(p))
                return true;
        return false;
    }

    /**
     * @brief Uniformly sample a random point within the world boundaries.
     *
     * Uses a thread-local Mersenne Twister RNG to ensure good randomness in
     * multi-threaded contexts without contention.
     */
    State sample() const {
        static thread_local std::mt19937 gen(std::random_device{}());
        std::uniform_real_distribution<double> dist_x(xmin, xmax);
        std::uniform_real_distribution<double> dist_y(ymin, ymax);

        State p(dist_x(gen), dist_y(gen));
        return p;
    }

    /**
     * @brief Get the dimension of this state space.
     */
    size_t getDimension() const { return 2; }

    /**
     * @brief Get the total area (volume) of the 2D state space.
     */
    double getVolume() const { return (xmax - xmin) * (ymax - ymin); }

    /**
     * @brief Get the volume of the unit ball in this dimension.
     *
     */
    double getUnitBallVolume() const { return M_PI; }
};
