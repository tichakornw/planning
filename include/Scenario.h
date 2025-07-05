#ifndef _SCENARIO_H
#define _SCENARIO_H

#include "OptimalSet.h"
#include "Rulebook.h"
#include "RulebookCost.h"
#include "WeightedGraph.h"

class Scenario {
  public:
    using WEdge = WeightedEdge<RulebookCost>;
    using WEdgePtr = std::shared_ptr<WEdge>;

    virtual ~Scenario() =
        default; // Virtual destructor for proper cleanup of derived classes.

    size_t getGraphSize() const { return graph.getNumVertices(); }

    size_t getNumRules() const { return rulebook.getNumRules(); }

    Rulebook &getRulebook() { return rulebook; }
    const Rulebook &getRulebook() const { return rulebook; }

    WeightedGraph<RulebookCost> &getGraph() { return graph; }
    const WeightedGraph<RulebookCost> &getGraph() const { return graph; }

    size_t get_init() const { return init_vid; }
    size_t get_goal() const { return goal_vid; }

    virtual void printGraph(const WeightedGraph<RulebookCost> &graph) const {
        graph.display();
    }

  protected:
    Rulebook rulebook;
    WeightedGraph<RulebookCost> graph;
    size_t init_vid;
    size_t goal_vid;

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
};

#endif
