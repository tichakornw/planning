#pragma once

#include "LinearTransition.h"
#include "Point2D.h"
#include "Rule.h"
#include "Rulebook.h"
#include "RulebookCost.h"
#include "ScenarioNavigation.h"
#include "World2D.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

class ScenarioNavigationClassical : public ScenarioNavigation {
  public:
    using StateSpace = World2D;
    using State = typename StateSpace::State;
    using Transition = LinearTransition<State>;
    using Cost = RulebookCost;

  public:
    ScenarioNavigationClassical(const World2D &world, const Point2D &start,
                                const Point2D &goal, double required_clearance)
        : ScenarioNavigation(world, start, goal, required_clearance) {}

    // -------------------------------
    // Cost computation
    // -------------------------------
    Cost getCost(const Transition &trans) const override {
        RulebookCost cost;
        cost.setRuleCost(rid_length, trans.getLength());
        return cost;
    }

    // -------------------------------
    // Collision function
    // -------------------------------
    CollisionFn collisionFn() const override {
        return [this](const Transition &trans) -> bool {
            const Point2D &p1 = trans.getStartState();
            const Point2D &p2 = trans.getEndState();
            // A transition is in collision if either obstacle or busy area cost
            // > 0
            bool collides = (getCollisionCost(p1, p2) > 0.0) ||
                            (getBusyCost(p1, p2) > 0.0) ||
                            (getClearanceCost(p1, p2) > 0.0);
            return collides;
        };
    }

  protected:
    // -------------------------------
    // Rulebook setup
    // -------------------------------
    void defineRulebook() override {
        // Define 1 rule
        const RuleSum r4("min_length"); // Minimize path length
        rid_length = rulebook.addRule(r4);
    }
};
