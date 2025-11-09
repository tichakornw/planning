#pragma once
#include <cmath>

inline bool approxEqual(double a, double b, double eps = 1e-6) {
    return std::fabs(a - b) < eps;
}
