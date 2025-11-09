#pragma once
#include <cassert>
#include <cmath>
#include <ostream>
#include <vector>

template <typename State> class LinearTransition {
  private:
    State from;
    State to;
    std::vector<State> samples; // discretized points along the line segment
    double length{0.0};

  public:
    LinearTransition() = default;

    LinearTransition(const State &start, const State &end)
        : from(start), to(end) {
        length = distance(from, to);
    }

    State getStartState() const { return from; }

    State getEndState() const { return to; }

    double getLength() const { return length; }

    std::vector<State> getSamples(double max_sample_dist = 0.1) {
        assert(max_sample_dist > 0.0 && "max_sample_dist must be positive");

        size_t num_samples =
            static_cast<size_t>(std::ceil(length / max_sample_dist)) + 1;
        if (num_samples < 2)
            num_samples = 2;
        discretize(num_samples);
        return samples;
    }

  private:
    // Distance metric (requires State supports a distance() method)
    static double distance(const State &a, const State &b) {
        if constexpr (requires(const State &s) { s.distance(b); }) {
            return a.distance(b);
        } else {
            static_assert(sizeof(State) == 0,
                          "State must implement distance(const State&) method");
        }
    }

    // Discretize the straight line between from and to
    void discretize(size_t num_samples) {
        assert(num_samples >= 2);
        samples.clear();
        for (size_t i = 0; i < num_samples; ++i) {
            double t = static_cast<double>(i) / (num_samples - 1);
            samples.push_back(interpolate(from, to, t));
        }
    }

    // Linear interpolation between states (you must specialize or overload)
    static State interpolate(const State &a, const State &b, double t) {
        // Default: works if State supports arithmetic
        return a + t * (b - a);
    }

  public:
    friend std::ostream &operator<<(std::ostream &os,
                                    const LinearTransition &tr) {
        os << "LinearTransition[" << tr.from << " -> " << tr.to
           << ", len=" << tr.length << ", steps=" << tr.samples.size() << "]";
        return os;
    }
};
