#include "ScenarioNavigationClassical.h"

ScenarioNavigationClassical::ScenarioNavigationClassical(const World2D &world, const Point2D &start,
                                                       const Point2D &goal, double required_clearance)
    : ScenarioNavigation(world, start, goal, required_clearance) {}

ScenarioNavigationClassical::Cost ScenarioNavigationClassical::getCost(const Transition &trans) const {
    RulebookCost cost;
    cost.setRuleCost(rid_length, trans.getLength());
    return cost;
}

ScenarioNavigation::CollisionFn ScenarioNavigationClassical::collisionFn() const {
    return [this](const Transition &trans) -> bool {
        const Point2D &p1 = trans.getStartState();
        const Point2D &p2 = trans.getEndState();
        // A transition is in collision if either obstacle or busy area cost > 0
        bool collides = (getCollisionCost(p1, p2) > 0.0) ||
                        (getBusyCost(p1, p2) > 0.0) ||
                        (getClearanceCost(p1, p2) > 0.0);
        return collides;
    };
}

void ScenarioNavigationClassical::defineRulebook() {
    // Define 1 rule
    const RuleSum r4("min_length"); // Minimize path length
    rid_length = rulebook.addRule(r4);
}