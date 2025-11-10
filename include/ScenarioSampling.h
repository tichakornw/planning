#pragma once
#include "Scenario.h"
#include <functional>

template <typename StateSpace, typename CostType, typename StateTransition>
class ScenarioSampling : public Scenario<typename StateSpace::State> {
  public:
    using State = typename StateSpace::State;
    using CostFn = std::function<CostType(const StateTransition &)>;
    using CollisionFn = std::function<bool(const StateTransition &)>;

  protected:
    StateSpace space;

  public:
    ScenarioSampling() = default;

    ScenarioSampling(const State &sinit, const State &sgoal)
        : Scenario<typename StateSpace::State>(sinit, sgoal) {}

    ScenarioSampling(const StateSpace &space_in, const State &sinit,
                     const State &sgoal)
        : Scenario<typename StateSpace::State>(sinit, sgoal), space(space_in) {}

    ~ScenarioSampling() = default;

    const StateSpace *getStateSpace() const { return &space; }

    virtual CostFn costFn() const = 0;

    // Optional: Derived class *may* override this
    virtual CollisionFn collisionFn() const {
        return [](const StateTransition &) { return false; };
    }
};
