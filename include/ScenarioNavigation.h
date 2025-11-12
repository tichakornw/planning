#pragma once

#include "LinearTransition.h"
#include "Point2D.h"
#include "Rule.h"
#include "Rulebook.h"
#include "RulebookCost.h"
#include "ScenarioSampling.h"
#include "World2D.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

class ScenarioNavigation
    : public ScenarioSampling<World2D, RulebookCost,
                              LinearTransition<World2D::State>> {
  public:
    using StateSpace = World2D;
    using State = typename StateSpace::State;
    using Transition = LinearTransition<State>;
    using Cost = RulebookCost;

  protected:
    double clearance;
    size_t rid_collision, rid_busy, rid_clearance, rid_rhr, rid_length;

  public:
    ScenarioNavigation(const World2D &world, const Point2D &start,
                       const Point2D &goal, double required_clearance)
        : ScenarioSampling<StateSpace, Cost, Transition>(world, start, goal),
          clearance(required_clearance) {}

    // -------------------------------
    // Cost computation
    // -------------------------------
    CostFn costFn() const override {
        return [this](const Transition &trans) -> RulebookCost {
            return getCost(trans);
        };
    }

    virtual Cost getCost(const Transition &trans) const {
        const Point2D &p1 = trans.getStartState();
        const Point2D &p2 = trans.getEndState();
        RulebookCost cost;
        cost.setRuleCost(rid_collision, getCollisionCost(p1, p2));
        cost.setRuleCost(rid_busy, getBusyCost(p1, p2));
        cost.setRuleCost(rid_clearance, getClearanceCost(p1, p2));
        cost.setRuleCost(rid_rhr, getRightHandRuleCost(p1, p2));
        cost.setRuleCost(rid_length, trans.getLength());
        return cost;
    }

  protected:
    // -------------------------------
    // Rulebook setup
    // -------------------------------
    void defineRulebook() override {
        // Define 4 rules
        const RuleSum r0("no_collision");   // Don't hit circular obstacles
        const RuleSum r1("avoid_busy");     // Don't enter busy region
        const RuleSum r2("keep_clearance"); // Stay safe distance away
        const RuleSum r3("rid_rhr");        // Stay to the right of obstacle
        const RuleSum r4("min_length");     // Minimize path length

        rid_collision = rulebook.addRule(r0);
        rid_busy = rulebook.addRule(r1);
        rid_clearance = rulebook.addRule(r2);
        rid_rhr = rulebook.addRule(r3);
        rid_length = rulebook.addRule(r4);

        // Priority hierarchy
        rulebook.addGTRelation(rid_collision, rid_busy);
        rulebook.addGTRelation(rid_busy, rid_clearance);
        rulebook.addGTRelation(rid_clearance, rid_rhr);
        rulebook.addGTRelation(rid_rhr, rid_length);
    }

    // -------------------------------
    // Individual cost terms
    // -------------------------------
    double getCollisionCost(const Point2D &p1, const Point2D &p2) const {
        // Penalize the length of line segment that is inside any circle
        double total = 0.0;
        for (const auto &obs : space.obstacles) {
            total += obs.intersectionLength(p1, p2, 0.0);
        }
        return total;
    }

    double getBusyCost(const Point2D &p1, const Point2D &p2) const {
        // Penalize the length of line segment that is inside the busy region
        double total = 0.0;
        for (const auto &region : space.regions) {
            total += region.intersectionLength(p1, p2);
        }
        return total;
    }

    double getClearanceCost(const Point2D &p1, const Point2D &p2) const {
        // Penalize if too close to any obstacle
        double total = 0.0;
        for (const auto &obs : space.obstacles) {
            total += obs.intersectionLength(p1, p2, clearance);
        }
        return total;
    }

    double getRightHandRuleCost(const Point2D &p1, const Point2D &p2) const {
        double total = 0.0;
        for (const auto &obs : space.obstacles)
            total += obs.projectedOverlapLength(p1, p2, false);
        return total;
    }
};
