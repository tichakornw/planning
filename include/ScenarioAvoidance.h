#ifndef _SCENARIO_AVOIDANCE_H
#define _SCENARIO_AVOIDANCE_H

#include <cmath>

#include "GridWorld.h"
#include "Rule.h"
#include "Rulebook.h"
#include "RulebookCost.h"
#include "ScenarioDiscrete.h"

class ScenarioAvoidance : public ScenarioDiscrete<DiscreteState2D> {
  public:
    ScenarioAvoidance(size_t refinement_option = 0);

  private:
    size_t refinement_option;
    const size_t num_x = 7;
    const size_t num_y = 5;
    const double dx = 2.0;
    const double dy = 2.0;
    
    // IDs of the rules
    size_t rid_blockage, rid_lane, rid_clearance, rid_length;

    void defineRulebook() override;
    void buildGraph() override;

    DiscreteState2D getStateAt(size_t row, size_t col);
    bool isInObstacle(size_t row, size_t col, size_t obstacle_x_min, size_t obstacle_x_max);
    bool isNearObstacle(size_t row, size_t col, size_t obstacle_x_min, size_t obstacle_x_max);
    bool isNotInLane(size_t row);
    
    RulebookCost getCost(size_t prev_row, size_t prev_col, size_t next_row,
                         size_t next_col, size_t obstacle_x_min,
                         size_t obstacle_x_max, double length);
};

#endif