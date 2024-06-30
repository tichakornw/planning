#ifndef _SCENARIO_H
#define _SCENARIO_H

#include "OptimalSet.h"
#include "Rulebook.h"
#include "RulebookCost.h"
#include "WeightedGraph.h"

class Scenario {
  public:
    virtual ~Scenario() =
        default; // Virtual destructor for proper cleanup of derived classes.

    OptimalSet<std::vector<std::shared_ptr<WeightedEdge<RulebookCost>>>,
               RulebookCost>
    getOptimalPlans() const {
        return graph.getOptimalPaths(init_vid, goal_vid);
    }

    size_t getGraphSize() const { return graph.getNumVertices(); }

    size_t getNumRules() const { return rulebook.getNumRules(); }

  protected:
    // Pure virtual function to be implemented by derived classes.
    virtual void setup() = 0;

    void displayRulebook() {
        std::cout << "Rulebook:" << std::endl;
        rulebook.display();

        std::cout << "Order: ";
        for (auto it = rulebook.begin(); it != rulebook.end(); ++it) {
            std::cout << "{ ";
            for (auto elem : *it) {
                std::cout << elem << " ";
            }
            std::cout << "} ";
        }
        std::cout << std::endl;

        // Successors
        std::cout << "Successors:" << std::endl;
        for (size_t i = 0; i < 4; ++i) {
            std::cout << "    " << i << ": ";
            const auto successors = rulebook.getSuccessors(i);
            for (auto j : successors) {
                std::cout << j << " ";
            }
            std::cout << std::endl;
        }
    }

    Rulebook rulebook;
    WeightedGraph<RulebookCost> graph;
    size_t init_vid;
    size_t goal_vid;
};

#endif
