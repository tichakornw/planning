#pragma once

#include "Point2D.h"
#include <iostream>
#include <vector>

/**
 * @brief Represents a circular region in 2D space.
 */
class CircleRegion {
  public:
    Point2D center;
    double radius;

    CircleRegion(const Point2D &c = {}, double r = 0.0);

    bool contain(const Point2D &p) const;
    double distanceToPoint(const Point2D &p) const;
    double distanceToLineSegment(const Point2D &p1, const Point2D &p2) const;
    double intersectionLength(const Point2D &p1, const Point2D &p2, double buffer = 0.0) const;
    double projectedOverlapLength(const Point2D &p1, const Point2D &p2, bool left_of_line) const;
};

/**
 * @brief Axis-aligned rectangular region in 2D.
 */
class RectangleRegion {
  public:
    double xmin, xmax, ymin, ymax;

    RectangleRegion(double xmin = 0, double xmax = 0, double ymin = 0, double ymax = 0);

    bool contain(const Point2D &p) const;
    double distanceToPoint(const Point2D &p) const;
    double intersectionLength(const Point2D &p1, const Point2D &p2) const;
};

/**
 * @brief Represents a 2D world with circular obstacles and rectangular regions.
 */
class World2D {
  public:
    using State = Point2D;

    std::vector<CircleRegion> obstacles;
    std::vector<RectangleRegion> regions;
    double xmin, xmax, ymin, ymax;

    World2D(double xmin_, double xmax_, double ymin_, double ymax_);

    void addObstacle(const Point2D &center, double radius);
    void addRegion(double xmin, double xmax, double ymin, double ymax);
    void addRegion(const RectangleRegion &region);

    bool isInsideObstacle(const State &p) const;
    bool isInsideRegion(const State &p) const;

    State sample() const;

    size_t getDimension() const;
    double getVolume() const;
    double getUnitBallVolume() const;
    
    void toJsonStream(std::ostream &os) const;
};