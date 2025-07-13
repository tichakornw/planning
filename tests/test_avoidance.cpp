#include <cassert>
#include <chrono>
#include <iostream>
#include <set>
#include <vector>

#include "OptimalSet.h"
#include "RulebookCost.h"
#include "RulebookPlanner.h"
#include "ScenarioAvoidance.h"
#include "WeightedGraph.h"

using WEdge = WeightedEdge<RulebookCost>;
using WEdgePtr = std::shared_ptr<WEdge>;

struct PathWithCost {
    std::vector<std::pair<size_t, size_t>> path;
    std::vector<double> cost;

    bool operator==(const PathWithCost &other) const {
        const double epsilon = 1e-3;

        if (path != other.path)
            return false;

        if (cost.size() != other.cost.size())
            return false;

        for (size_t i = 0; i < cost.size(); ++i) {
            if (std::abs(cost[i] - other.cost[i]) > epsilon)
                return false;
        }

        return true;
    }
};

struct PathWithCostComparator {
    bool operator()(const PathWithCost &a, const PathWithCost &b) const {
        if (a.path < b.path)
            return true;
        if (b.path < a.path)
            return false;

        // Use lexicographic comparison with tolerance
        if (a.cost.size() != b.cost.size())
            return a.cost.size() < b.cost.size();

        for (size_t i = 0; i < a.cost.size(); ++i) {
            if (std::abs(a.cost[i] - b.cost[i]) > 1e-3)
                return a.cost[i] < b.cost[i];
        }

        return false; // They are approximately equal
    }
};

std::set<PathWithCost, PathWithCostComparator> extractPathWithCostSet(
    const OptimalSet<std::vector<WEdgePtr>, RulebookCost> &optimal_set) {
    std::set<PathWithCost, PathWithCostComparator> result;
    for (auto &eid : optimal_set.getAllElementIDs()) {
        const auto element = optimal_set.getElement(eid);
        PathWithCost p;

        // Convert path to endpoint sequence
        for (const auto &edge : element.element) {
            p.path.emplace_back(edge->from->vid, edge->to->vid);
        }

        // Extract cost as vector of doubles from RulebookCost
        const auto &rulebook_cost = element.cost;
        size_t num_rules = rulebook_cost.getNumRules();
        p.cost.reserve(num_rules);
        for (size_t i = 0; i < num_rules; ++i) {
            p.cost.push_back(
                rulebook_cost[i]
                    ->getValue()); // assuming operator[] returns double
        }

        result.insert(std::move(p));
    }
    return result;
}

void testAvoidance() {
    ScenarioAvoidance scenario;
    RulebookPlanner planner(scenario);
    auto start = std::chrono::high_resolution_clock::now();
    const auto actual_optimal_set = planner.getOptimalPlans();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::milliseconds>(stop - start);

    assert(planner.getGraphSize() == scenario.getGraphSize());
    assert(planner.getGraphSize() == 35);
    assert(planner.getNumRules() == scenario.getNumRules());
    assert(planner.getNumRules() == 4);

    const auto actual = extractPathWithCostSet(actual_optimal_set);

    std::set<PathWithCost, PathWithCostComparator> expected = {
        // 119
        {{{0, 8}, {8, 9}, {9, 17}, {17, 11}, {11, 12}, {12, 6}},
         {0, 2, 4, 15.3137}},
        // 113
        {{{0, 8}, {8, 9}, {9, 17}, {17, 18}, {18, 12}, {12, 6}},
         {0, 4, 2, 15.3137}},
        // 101
        {{{0, 8}, {8, 9}, {9, 10}, {10, 11}, {11, 5}, {5, 6}},
         {0, 0, 6, 13.6569}},
        // 96
        {{{0, 1}, {1, 9}, {9, 17}, {17, 11}, {11, 5}, {5, 6}},
         {0, 2, 4, 15.3137}},
        // 95
        {{{0, 1}, {1, 9}, {9, 17}, {17, 11}, {11, 12}, {12, 6}},
         {0, 2, 4, 15.3137}},
        // 128
        {{{0, 8}, {8, 16}, {16, 17}, {17, 18}, {18, 12}, {12, 6}},
         {0, 6, 0, 15.3137}},
        // 89
        {{{0, 1}, {1, 9}, {9, 17}, {17, 18}, {18, 12}, {12, 6}},
         {0, 4, 2, 15.3137}},
        // 120
        {{{0, 8}, {8, 9}, {9, 17}, {17, 11}, {11, 5}, {5, 6}},
         {0, 2, 4, 15.3137}},
        // 107
        {{{0, 8}, {8, 9}, {9, 10}, {10, 18}, {18, 12}, {12, 6}},
         {0, 2, 4, 15.3137}},
        // 81
        {{{0, 1}, {1, 9}, {9, 10}, {10, 18}, {18, 12}, {12, 6}},
         {0, 2, 4, 15.3137}},
        // 99
        {{{0, 8}, {8, 9}, {9, 10}, {10, 11}, {11, 12}, {12, 6}},
         {0, 0, 6, 13.6569}},
        // 73
        {{{0, 1}, {1, 9}, {9, 10}, {10, 11}, {11, 5}, {5, 6}},
         {0, 0, 6, 13.6569}},
        // 71
        {{{0, 1}, {1, 9}, {9, 10}, {10, 11}, {11, 12}, {12, 6}},
         {0, 0, 6, 13.6569}},

    };

    assert(actual == expected);
}
