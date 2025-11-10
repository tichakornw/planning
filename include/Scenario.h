#ifndef _SCENARIO_H
#define _SCENARIO_H

#include "OptimalSet.h"
#include "Rulebook.h"
#include "RulebookCost.h"
#include "WeightedGraph.h"

template <typename State> class Scenario {
  public:
    Scenario() = default;

    Scenario(const State &sinit, const State &sgoal)
        : init_state(sinit), goal_state(sgoal) {}

    virtual ~Scenario() =
        default; // Virtual destructor for proper cleanup of derived classes.

    size_t getNumRules() const { return rulebook.getNumRules(); }

    Rulebook &getRulebook() { return rulebook; }
    const Rulebook &getRulebook() const { return rulebook; }

    void setDebug(bool d) { debug = d; }

  protected:
    Rulebook rulebook;
    // initial and goal states
    State init_state;
    State goal_state;
    bool debug = false;

    // Pure virtual function to be implemented by derived classes.
    virtual void buildRulebook() = 0;

    virtual void setup() {
        buildRulebook();
        RulebookCost::setRulebook(rulebook);
    }

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
