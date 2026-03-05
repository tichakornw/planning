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

    ScenarioNavigationClassical(const World2D &world, const Point2D &start,
                                const Point2D &goal, double required_clearance);

    // -------------------------------
    // Cost computation
    // -------------------------------
    Cost getCost(const Transition &trans) const override;

    // -------------------------------
    // Collision function
    // -------------------------------
    CollisionFn collisionFn() const override;

  protected:
    // -------------------------------
    // Rulebook setup
    // -------------------------------
    void defineRulebook() override;
};