#ifndef _RULEBOOKCOST_H
#define _RULEBOOKCOST_H

#include "Rulebook.h"
#include <iostream>
#include <stdexcept>

class RulebookCost {
  public:
    RulebookCost() : cost_vector(rulebook->getNumRules(), 0.0) {}

    RulebookCost(size_t num_rules) : cost_vector(num_rules, 0.0) {}

    void setRuleCost(size_t rule_index, double cost) {
        cost_vector[rule_index] = cost;
    }

    double getRuleCost(size_t rule_index) const {
        return cost_vector[rule_index];
    }

    static void setRulebook(const Rulebook &rb) {
        rulebook = std::make_unique<Rulebook>(rb);
    }

    RulebookCost getZero() { return RulebookCost(cost_vector.size()); }

    // Overloading the stream insertion operator for easy printing
    friend std::ostream &operator<<(std::ostream &os, const RulebookCost &c) {
        os << "[";

        for (auto it = c.cost_vector.begin(); it != c.cost_vector.end(); ++it) {
            os << *it;
            if (std::next(it) != c.cost_vector.end()) {
                os << ",";
            }
        }
        os << "]";
        return os;
    }

    RulebookCost operator+(const RulebookCost &other) const {
        if (cost_vector.size() != other.cost_vector.size()) {
            std::cout << "This cost: " << cost_vector.size()
                      << " other: " << other.cost_vector.size() << std::endl;
            throw std::invalid_argument(
                "Cost vectors must be of the same size");
        }

        RulebookCost result(cost_vector.size());
        for (size_t i = 0; i < cost_vector.size(); ++i) {
            result.cost_vector[i] = cost_vector[i] + other.cost_vector[i];
        }
        return result;
    }

    bool operator<=(const RulebookCost &other) const {
        std::unordered_set<size_t> overridden_rules;
        for (auto it = rulebook->begin(); it != rulebook->end(); ++it) {
            for (auto rule_index : *it) {
                if (overridden_rules.find(rule_index) != overridden_rules.end())
                    continue;
                if (getRuleCost(rule_index) > other.getRuleCost(rule_index))
                    return false;
                if (getRuleCost(rule_index) < other.getRuleCost(rule_index)) {
                    const auto successors = rulebook->getSuccessors(rule_index);
                    overridden_rules.insert(successors.begin(),
                                            successors.end());
                }
            }
        }

        return true;
    }

    bool operator>=(const RulebookCost &other) const { return other <= *this; }

    // Overloading comparison operators
    bool operator<(const RulebookCost &other) const {
        return (*this <= other) && !(other <= *this);
    }

    bool operator>(const RulebookCost &other) const {
        return (other <= *this) && !(*this <= other);
    }

    bool operator==(const RulebookCost &other) const {
        return (other <= *this) && (*this <= other);
    }

  private:
    static std::unique_ptr<Rulebook> rulebook;
    std::vector<double> cost_vector;
    double value;
};

std::unique_ptr<Rulebook> RulebookCost::rulebook = nullptr;

#endif
