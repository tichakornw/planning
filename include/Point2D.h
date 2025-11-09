#ifndef _POINT_H
#define _POINT_H

#include <cmath>
#include <iostream>
#include <vector>

// A class that represents a 2D vector
class Vector2D {
  public:
    double x;
    double y;

  public:
    // Constructors
    Vector2D() : x(0), y(0) {}
    Vector2D(const double x, const double y) : x(x), y(y) {}

    // Copy constructor
    Vector2D(const Vector2D &other) : x(other.x), y(other.y) {}

    // Move constructor
    Vector2D(Vector2D &&other) noexcept : x(0), y(0) { swap(*this, other); }

    // Copy assignment operator
    Vector2D &operator=(const Vector2D &other) {
        if (this != &other) {
            x = other.x;
            y = other.y;
        }
        return *this;
    }

    // Move assignment operator
    Vector2D &operator=(Vector2D &&other) noexcept {
        swap(*this, other);
        return *this;
    }

    // Swap function
    friend void swap(Vector2D &first, Vector2D &second) noexcept {
        std::swap(first.x, second.x);
        std::swap(first.y, second.y);
    }

    // Return the norm of this vector
    double norm() const { return sqrt(this->norm2()); }

    // Return the square of the norm of this vector
    double norm2() const { return (this->x * this->x) + (this->y * this->y); }

    // Return the dot product of this vector and the given vector
    double dot(const Vector2D &v) const {
        return (this->x * v.x) + (this->y * v.y);
    }

    // Return the vector obtain by scaling this vector by a constant c.
    Vector2D operator*(const double c) const {
        return Vector2D(c * this->x, c * this->y);
    }

    friend std::ostream &operator<<(std::ostream &os, const Vector2D &v) {
        os << "[" << v.x << "," << v.y << "]";
        return os;
    }
};

// non-member operator* for double * State
inline Vector2D operator*(double c, const Vector2D &v) {
    return v * c; // reuse member operator*
}

// A class that represents a point in 2D
class Point2D {
  public:
    double x;
    double y;

  public:
    // Constructors
    Point2D() : x(0), y(0) {}
    Point2D(const double x, const double y) : x(x), y(y) {}

    // Copy constructor
    Point2D(const Point2D &other) : x(other.x), y(other.y) {}

    // Move constructor
    Point2D(Point2D &&other) noexcept : x(0), y(0) { swap(*this, other); }

    // Copy assignment operator
    Point2D &operator=(const Point2D &other) {
        if (this != &other) {
            x = other.x;
            y = other.y;
        }
        return *this;
    }

    // Move assignment operator
    Point2D &operator=(Point2D &&other) noexcept {
        swap(*this, other);
        return *this;
    }

    // Swap function
    friend void swap(Point2D &first, Point2D &second) noexcept {
        std::swap(first.x, second.x);
        std::swap(first.y, second.y);
    }

    // Return the distance from this point to the given point
    double distance(const Point2D &p) const {
        return std::hypot(this->x - p.x, this->y - p.y);
    }

    // Return a Vector2D object corresponding to the vector from the given point
    // to this point
    Vector2D operator-(const Point2D &p) const {
        return Vector2D(this->x - p.x, this->y - p.y);
    }

    // Return a point obtained by moving this point by the given vector
    Point2D operator+(const Vector2D &v) const {
        return Point2D(this->x + v.x, this->y + v.y);
    }

    bool operator==(const Point2D &other) const {
        return x == other.x && y == other.y;
    }

    // Convert this point to a Vector object
    Vector2D to_vector() const { return Vector2D(this->x, this->y); }

    std::vector<double> to_std_vector() const {
        return std::vector<double>{this->x, this->y};
    }

    friend std::ostream &operator<<(std::ostream &os, const Point2D &p) {
        os << "[" << p.x << "," << p.y << "]";
        return os;
    }
};

#endif
