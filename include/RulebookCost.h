#ifndef _RULEBOOKCOST_H
#define _RULEBOOKCOST_H

#include "Rulebook.h"
#include "RuleCost.h"
#include <iostream>
#include <stdexcept>

class RulebookCost {
  public:
    RulebookCost() {
        if (!rulebook) {
            throw std::runtime_error("Rulebook must be initialized before RulebookCost.");
        }

        const size_t n = rulebook->getNumRules();
        cost_vector.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            cost_vector.push_back(rulebook->getRule(i).makeCost(0.0));
        }
    }

    void setRuleCost(size_t rule_index, double cost) {
        cost_vector[rule_index]->setValue(cost);
    }

    std::shared_ptr<RuleCost>& operator[](size_t index) {
        return cost_vector[index];
    }

    const std::shared_ptr<RuleCost>& operator[](size_t index) const {
        return cost_vector[index];
    }

    static void setRulebook(const Rulebook &rb) {
        rulebook = std::make_unique<Rulebook>(rb);
    }

    RulebookCost getZero() { return RulebookCost(); }

    // Overloading the stream insertion operator for easy printing
    friend std::ostream &operator<<(std::ostream &os, const RulebookCost &c) {
        os << "[";

        for (auto it = c.cost_vector.begin(); it != c.cost_vector.end(); ++it) {
            os << (*it)->getValue();
            if (std::next(it) != c.cost_vector.end()) {
                os << ",";
            }
        }
        os << "]";
        return os;
    }

    RulebookCost operator+(const RulebookCost &other) const {
        if (cost_vector.size() != other.cost_vector.size()) {
            std::cerr << "This cost: " << cost_vector.size()
                      << " other: " << other.cost_vector.size() << std::endl;
            throw std::invalid_argument(
                "Cost vectors must be of the same size");
        }

        RulebookCost result;
        result.cost_vector.resize(cost_vector.size());

        for (size_t i = 0; i < cost_vector.size(); ++i) {
            result.cost_vector[i] = *cost_vector[i] + *other.cost_vector[i];
        }

        return result;
    }

    bool operator<=(const RulebookCost &other) const {
        std::unordered_set<size_t> overridden_rules;
        for (auto it = rulebook->begin(); it != rulebook->end(); ++it) {
            for (auto rule_index : *it) {
                if (overridden_rules.find(rule_index) != overridden_rules.end())
                    continue;

                const RuleCost& rule_cost = *cost_vector[rule_index];
                const RuleCost& other_cost = *other[rule_index];

                if (rule_cost > other_cost)
                    return false;
                if (rule_cost < other_cost) {
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
    std::vector<std::shared_ptr<RuleCost>> cost_vector;
    double value;
};

std::unique_ptr<Rulebook> RulebookCost::rulebook = nullptr;

#endif
