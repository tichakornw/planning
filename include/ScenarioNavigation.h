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

    ScenarioNavigation(const World2D &world, const Point2D &start,
                       const Point2D &goal, double required_clearance);

    CostFn costFn() const override;
    virtual Cost getCost(const Transition &trans) const;

  protected:
    double clearance;
    size_t rid_collision, rid_busy, rid_clearance, rid_rhr, rid_length;

    void defineRulebook() override;

    double getCollisionCost(const Point2D &p1, const Point2D &p2) const;
    double getBusyCost(const Point2D &p1, const Point2D &p2) const;
    double getClearanceCost(const Point2D &p1, const Point2D &p2) const;
    double getRightHandRuleCost(const Point2D &p1, const Point2D &p2) const;
};