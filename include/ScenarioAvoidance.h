#ifndef _SCENARIO_AVOIDANCE_H
#define _SCENARIO_AVOIDANCE_H

#include <cmath>

#include "RulebookCost.h"
#include "Rulebook.h"
#include "Rule.h"
#include "Scenario.h"

class ScenarioAvoidance : public Scenario {
  public:
    ScenarioAvoidance() { setup(); }

  protected:
    void setup() override {
        const size_t num_x = 7;
        const size_t num_y = 5;
        const double dx = 2.0;
        const double dy = 2.0;
        buildRulebook();
        RulebookCost::setRulebook(rulebook);
        buildGrid(num_x, num_y, dx, dy);
        init_vid = 0;
        goal_vid = num_x - 1;
    }

  private:
    void buildRulebook() {
        const RuleSum r0("r0");
        const RuleSum r1("r1");
        const RuleSum r2("r2");
        const RuleSum r3("r3");

        const size_t r0_id = rulebook.addRule(r0);
        const size_t r1_id = rulebook.addRule(r1);
        const size_t r2_id = rulebook.addRule(r2);
        const size_t r3_id = rulebook.addRule(r3);

        rulebook.setEquivalentClasses({
                {r0_id}, {r1_id}, {r2_id}, {r3_id}
            });

        rulebook.addGTRelation(r0_id, r1_id);
        rulebook.addGTRelation(r0_id, r2_id);
        rulebook.addGTRelation(r1_id, r3_id);
        rulebook.addGTRelation(r2_id, r3_id);

        rulebook.build();

        displayRulebook();
    }

    void buildGrid(size_t num_x, size_t num_y, double dx, double dy) {
        size_t obstacle_x_min = floor(static_cast<double>(num_x) / 2) - 1;
        size_t obstacle_x_max = ceil(static_cast<double>(num_x) / 2);
        double diag_length = sqrt(dx * dx + dy * dy);

        for (size_t vid = 0; vid < num_x * num_y; ++vid)
            graph.addVertex(vid);

        for (size_t row = 0; row < num_y; ++row) {
            for (size_t col = 0; col < num_x; ++col) {
                size_t start_vertex = num_x * row + col;
                if (col + 1 < num_x) {
                    // Horizontal edge
                    const RulebookCost cost1 =
                        getCost(row, col, row, col + 1, obstacle_x_min,
                                obstacle_x_max, dx);
                    graph.addEdge(start_vertex, start_vertex + 1, cost1, 0);
                    // Diagonal edge up
                    if (row + 1 < num_y) {
                        const RulebookCost cost2 =
                            getCost(row, col, row + 1, col + 1, obstacle_x_min,
                                    obstacle_x_max, diag_length);
                        graph.addEdge(start_vertex, start_vertex + num_x + 1,
                                      cost2, 1);
                    }
                    // Diagonal edge down
                    if (row > 0) {
                        const RulebookCost cost3 =
                            getCost(row, col, row - 1, col + 1, obstacle_x_min,
                                    obstacle_x_max, diag_length);
                        graph.addEdge(start_vertex, start_vertex - num_x + 1,
                                      cost3, 2);
                    }
                }
            }
        }
        // Display the graph
        std::cout << "Graph: " << std::endl;
        graph.display();
    }

    bool isInObstacle(size_t row, size_t col, size_t obstacle_x_min,
                      size_t obstacle_x_max) {
        if (row > 0)
            return false;
        return (col >= obstacle_x_min && col <= obstacle_x_max);
    }

    bool isNearObstacle(size_t row, size_t col, size_t obstacle_x_min,
                        size_t obstacle_x_max) {
        if (row > 1)
            return false;
        return (col >= obstacle_x_min && col <= obstacle_x_max);
    }

    bool isNotInLane(size_t row) { return (row > 1); }

    RulebookCost getCost(size_t prev_row, size_t prev_col, size_t next_row,
                         size_t next_col, size_t obstacle_x_min,
                         size_t obstacle_x_max, double length) {
        double blockage_cost =
            isInObstacle(prev_row, prev_col, obstacle_x_min, obstacle_x_max)
                ? 1.0
                : 0.0;
        if (isInObstacle(next_row, next_col, obstacle_x_min, obstacle_x_max))
            blockage_cost = blockage_cost + 1.0;
        double lane_cost = isNotInLane(prev_row) ? 1.0 : 0.0;
        if (isNotInLane(next_row))
            lane_cost = lane_cost + 1.0;
        double clearance_cost =
            isNearObstacle(prev_row, prev_col, obstacle_x_min, obstacle_x_max)
                ? 1.0
                : 0.0;
        if (isNearObstacle(next_row, next_col, obstacle_x_min, obstacle_x_max))
            clearance_cost = clearance_cost + 1.0;
        RulebookCost cost;
        cost.setRuleCost(0, blockage_cost);
        cost.setRuleCost(1, lane_cost);
        cost.setRuleCost(2, clearance_cost);
        cost.setRuleCost(3, length);
        return cost;
    }
};

#endif
