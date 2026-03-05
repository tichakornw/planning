#ifndef _SCENARIO_RANDOM_H
#define _SCENARIO_RANDOM_H

#include <vector>
#include <unordered_set>
#include <iostream>

#include "GridWorld.h"
#include "Rule.h"
#include "Rulebook.h"
#include "RulebookCost.h"
#include "ScenarioDiscrete.h"

class ScenarioRandom : public ScenarioDiscrete<DiscreteState2D> {
  public:
    ScenarioRandom();
    ScenarioRandom(size_t num_x, size_t num_y, size_t num_rules);

  private:
    size_t num_x;
    size_t num_y;
    size_t num_rules;

    void setRandomInitAndGoal();
    size_t getRandomInt(size_t min_value, size_t max_value);
    double getRandomDouble(double min_value, double max_value);
    std::vector<std::unordered_set<size_t>> getRandomSubsets(int n);
    RulebookCost getRandomCost();

    void defineRulebook() override;
    void buildGraph() override;
};

#endif