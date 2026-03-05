#include "World2D.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <random>

CircleRegion::CircleRegion(const Point2D &c, double r) : center(c), radius(r) {}

bool CircleRegion::contain(const Point2D &p) const {
    return center.distance(p) <= radius;
}

double CircleRegion::distanceToPoint(const Point2D &p) const {
    return std::max(0.0, center.distance(p) - radius);
}

double CircleRegion::distanceToLineSegment(const Point2D &p1, const Point2D &p2) const {
    auto d = p2 - p1;
    double len2 = d.norm2();

    if (len2 < 1e-9)
        return distanceToPoint(p1);

    double t = ((center - p1).dot(d)) / len2;
    t = std::clamp(t, 0.0, 1.0);
    Point2D projection = p1 + d * t;
    return distanceToPoint(projection);
}

double CircleRegion::intersectionLength(const Point2D &p1, const Point2D &p2, double buffer) const {
    assert(buffer >= 0.0);

    const double r = radius + buffer;
    auto d = p2 - p1;
    double len = d.norm();
    if (len < 1e-9)
        return contain(p1) ? len : 0.0;

    double a = d.dot(d);
    double b = 2.0 * ((p1.x - center.x) * d.x + (p1.y - center.y) * d.y);
    double c = (p1.x - center.x) * (p1.x - center.x) +
               (p1.y - center.y) * (p1.y - center.y) - r * r;

    double disc = b * b - 4 * a * c;
    const double EPS = 1e-12;

    if (disc < -EPS) {
        Point2D mid = p1 + (d * 0.5);
        return contain(mid) ? len : 0.0;
    }

    if (std::fabs(disc) <= EPS) {
        return 0.0;
    }

    double sqrt_disc = std::sqrt(disc);
    double t1 = (-b - sqrt_disc) / (2 * a);
    double t2 = (-b + sqrt_disc) / (2 * a);

    double tmin = std::max(0.0, std::min(t1, t2));
    double tmax = std::min(1.0, std::max(t1, t2));

    if (tmax <= tmin + EPS) {
        Point2D mid = p1 + d * 0.5;
        return contain(mid) ? len : 0.0;
    }

    double overlap_len = (tmax - tmin) * len;
    return overlap_len;
}

double CircleRegion::projectedOverlapLength(const Point2D &p1, const Point2D &p2, bool left_of_line) const {
    auto d = p2 - p1;
    double len = d.norm();
    if (len < 1e-9)
        return 0.0;

    Vector2D d_unit = d * (1 / len);
    Vector2D n(-d_unit.y, d_unit.x);

    double s = (center - p1).dot(n);
    double proj_center = (center - p1).dot(d_unit);

    double t1 = proj_center - radius;
    double t2 = proj_center + radius;

    double start = std::max(0.0, t1);
    double end = std::min(len, t2);

    if (end <= start)
        return 0.0;

    double overlap = end - start;
    bool circle_on_left = (s > 0);
    bool relevant_side = (left_of_line && circle_on_left) || (!left_of_line && !circle_on_left);
    
    return relevant_side ? overlap : 0.0;
}

// ==========================================
// RectangleRegion Implementation
// ==========================================

RectangleRegion::RectangleRegion(double xmin, double xmax, double ymin, double ymax)
    : xmin(xmin), xmax(xmax), ymin(ymin), ymax(ymax) {}

bool RectangleRegion::contain(const Point2D &p) const {
    return p.x >= xmin && p.x <= xmax && p.y >= ymin && p.y <= ymax;
}

double RectangleRegion::distanceToPoint(const Point2D &p) const {
    double dx = std::max({xmin - p.x, 0.0, p.x - xmax});
    double dy = std::max({ymin - p.y, 0.0, p.y - ymax});
    return std::sqrt(dx * dx + dy * dy);
}

double RectangleRegion::intersectionLength(const Point2D &p1, const Point2D &p2) const {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double t0 = 0.0;
    double t1 = 1.0;

    auto clip = [&](double p, double q) -> bool {
        if (std::fabs(p) < 1e-12) {
            if (q <= 0) return false;
            return true;
        }
        double r = q / p;
        if (p < 0) {
            if (r > t1) return false;
            if (r > t0) t0 = r;
        } else if (p > 0) {
            if (r < t0) return false;
            if (r < t1) t1 = r;
        }
        return true;
    };

    if (!clip(-dx, p1.x - xmin)) return 0.0;
    if (!clip(dx, xmax - p1.x)) return 0.0;
    if (!clip(-dy, p1.y - ymin)) return 0.0;
    if (!clip(dy, ymax - p1.y)) return 0.0;

    if (t1 < t0) return 0.0;

    double segment_length = std::sqrt(dx * dx + dy * dy);
    return (t1 - t0) * segment_length;
}

// ==========================================
// World2D Implementation
// ==========================================

World2D::World2D(double xmin_, double xmax_, double ymin_, double ymax_)
    : xmin(xmin_), xmax(xmax_), ymin(ymin_), ymax(ymax_) {}

void World2D::addObstacle(const Point2D &center, double radius) {
    obstacles.emplace_back(center, radius);
}

void World2D::addRegion(double xmin, double xmax, double ymin, double ymax) {
    regions.emplace_back(xmin, xmax, ymin, ymax);
}

void World2D::addRegion(const RectangleRegion &region) { 
    regions.push_back(region); 
}

bool World2D::isInsideObstacle(const State &p) const {
    for (const auto &c : obstacles)
        if (c.contain(p))
            return true;
    return false;
}

bool World2D::isInsideRegion(const State &p) const {
    for (const auto &r : regions)
        if (r.contain(p))
            return true;
    return false;
}

World2D::State World2D::sample() const {
    static thread_local std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<double> dist_x(xmin, xmax);
    std::uniform_real_distribution<double> dist_y(ymin, ymax);

    return State(dist_x(gen), dist_y(gen));
}

size_t World2D::getDimension() const { 
    return 2; 
}

double World2D::getVolume() const { 
    return (xmax - xmin) * (ymax - ymin); 
}

double World2D::getUnitBallVolume() const { 
    return M_PI; 
}

void World2D::toJsonStream(std::ostream &os) const {
    os << std::fixed << std::setprecision(6);
    os << "{\n";

    // --- Boundaries ---
    os << "  \"bounds\": {\"xmin\": " << xmin << ", \"xmax\": " << xmax
       << ", \"ymin\": " << ymin << ", \"ymax\": " << ymax << "},\n";

    // --- Obstacles ---
    os << "  \"obstacles\": [\n";
    for (size_t i = 0; i < obstacles.size(); ++i) {
        const auto &obs = obstacles[i];
        os << "    {\"x\": " << obs.center.x << ", \"y\": " << obs.center.y
           << ", \"radius\": " << obs.radius << "}";
        if (i + 1 < obstacles.size())
            os << ",";
        os << "\n";
    }
    os << "  ],\n";

    // --- Regions ---
    os << "  \"regions\": [\n";
    for (size_t i = 0; i < regions.size(); ++i) {
        const auto &r = regions[i];
        os << "    {\"xmin\": " << r.xmin << ", \"xmax\": " << r.xmax
           << ", \"ymin\": " << r.ymin << ", \"ymax\": " << r.ymax << "}";
        if (i + 1 < regions.size())
            os << ",";
        os << "\n";
    }
    os << "  ]\n";

    os << "}";
}