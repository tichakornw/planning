#ifndef _RULEBOOKCOST_H
#define _RULEBOOKCOST_H

#include "RuleCost.h"
#include "Rulebook.h"
#include <iostream>
#include <memory>
#include <vector>

class RulebookCost {
  public:
    RulebookCost(double v = 0.0);

    void setRuleCost(size_t rule_index, double cost);
    double getRuleCost(size_t index) const;

    std::shared_ptr<RuleCost> &operator[](size_t index) { return cost_vector[index]; }
    const std::shared_ptr<RuleCost> &operator[](size_t index) const { return cost_vector[index]; }

    size_t getNumRules() const { return cost_vector.size(); }
    const std::vector<std::shared_ptr<RuleCost>> &getCosts() const { return cost_vector; }

    static void setRulebook(const Rulebook &rb);
    static RulebookCost getZero();

    // TEMPLATES MUST STAY IN THE HEADER
    template <typename SubCostType>
    SubCostType getSubCost(size_t rule_index) const {
        auto cost_ptr = std::dynamic_pointer_cast<SubCostType>(cost_vector[rule_index]);
        if (!cost_ptr) {
            throw std::runtime_error("SubCost type mismatch at rule index " +
                                     std::to_string(rule_index));
        }
        return *cost_ptr;
    }

    friend std::ostream &operator<<(std::ostream &os, const RulebookCost &c);

    RulebookCost operator+(const RulebookCost &other) const;
    bool operator<=(const RulebookCost &other) const;

    // These rely on operator<=, so they can stay inline as tiny wrappers
    bool operator>=(const RulebookCost &other) const { return other <= *this; }
    bool operator<(const RulebookCost &other) const { return (*this <= other) && !(other <= *this); }
    bool operator>(const RulebookCost &other) const { return (other <= *this) && !(*this <= other); }
    bool operator==(const RulebookCost &other) const { return (other <= *this) && (*this <= other); }

  private:
    static std::unique_ptr<Rulebook> rulebook;
    std::vector<std::shared_ptr<RuleCost>> cost_vector;
    double value;
};

template <> class std::numeric_limits<RulebookCost> {
  public:
    static constexpr bool is_specialized = true;

    static RulebookCost max() noexcept {
        return RulebookCost(std::numeric_limits<double>::max());
    }

    static RulebookCost min() noexcept {
        return RulebookCost(std::numeric_limits<double>::lowest());
    }

    static RulebookCost lowest() noexcept {
        return RulebookCost(std::numeric_limits<double>::lowest());
    }

    static constexpr bool is_signed = true;
    static constexpr bool is_integer = false;
    static constexpr bool is_exact = false;
    static constexpr bool has_infinity = false;
    static constexpr bool has_quiet_NaN = false;
    static constexpr bool has_signaling_NaN = false;
    static constexpr std::float_denorm_style has_denorm = std::denorm_absent;
    static constexpr bool has_denorm_loss = false;
    static constexpr std::float_round_style round_style = std::round_to_nearest;
    static constexpr bool is_iec559 = false;
    static constexpr bool is_bounded = true;
    static constexpr bool is_modulo = false;
};

// Make RulebookCost hashable
namespace std {
template <> struct hash<RulebookCost> {
    std::size_t operator()(const RulebookCost &cost) const {
        std::size_t seed = 0;
        for (const auto &rule_cost : cost.getCosts()) {
            // Hash using getValue() (same logic for RuleCostMax/RuleCostSum)
            std::size_t h = std::hash<double>()(rule_cost->getValue());

            // Combine hashes (boost-like)
            seed ^= h + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};
} // namespace std

#endif
