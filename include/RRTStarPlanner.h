#pragma once

#include "Planner.h"
#include "ScenarioSampling.h"
#include "StateTree.h"
#include <functional>
#include <random>

template <typename StateSpace, typename CostType, typename StateTransition>
class RRTStarPlanner : public Planner<CostType> {
  public:
    using State = typename StateSpace::State;

    using Tree = StateTree<State, CostType, StateTransition>;
    using SVertex = StateVertex<State>;
    using SVertexPtr = std::shared_ptr<SVertex>;
    using SEdge = StateTransitionEdge<State, CostType, StateTransition>;
    using SEdgePtr = std::shared_ptr<SEdge>;

    using SampleFn = std::function<State()>;
    using CostFn = std::function<CostType(const StateTransition &)>;
    using CollisionFn = std::function<bool(const StateTransition &)>;

  private:
    Tree tree;
    const StateSpace &space;
    CostFn cost_fn;
    CollisionFn collision_fn;

    double gamma_rrt;
    double inv_dim; ///< cached 1.0 / dim

    size_t root_vid = std::numeric_limits<size_t>::max();
    size_t goal_vid = std::numeric_limits<size_t>::max();

  public:
    RRTStarPlanner(
        const StateSpace *space, CostFn cost_fn,
        CollisionFn collision_fn =
            [](const StateTransition &) { return false; })
        : Planner<CostType>(tree), space(*space), cost_fn(std::move(cost_fn)),
          collision_fn(std::move(collision_fn)) {
        initializeConstants();
    }

    // Takes a ScenarioSampling-derived object
    RRTStarPlanner(
        const ScenarioSampling<StateSpace, CostType, StateTransition> &scenario)
        : Planner<CostType>(tree), space(*scenario.getStateSpace()),
          cost_fn(scenario.costFn()), collision_fn(scenario.collisionFn()) {
        initializeConstants();
    }

    // Initialize the tree with the root state
    void initialize(const State &start_state) {
        root_vid = tree.addStateVertex(start_state, true);
    }

    // -------------------------------------------------------
    // Run RRT* for a fixed number of iterations
    // -------------------------------------------------------
    void run(const State &goal_state, size_t iterations) {
        if (root_vid == std::numeric_limits<size_t>::max())
            throw std::runtime_error(
                "RRTStarPlanner: must call initialize() first");

        for (size_t i = 0; i < iterations; ++i) {
            State x_rand = space.sample();
            auto new_vid = connectToTree(x_rand);
            if (new_vid == std::numeric_limits<size_t>::max())
                continue;
            double neighbor_radius = computeRadius(tree.getNumVertices());
            rewireNeighbors(new_vid, neighbor_radius);
        }

        goal_vid = connectToTree(goal_state);
    }

    // -------------------------------------------------------
    // Accessors
    // -------------------------------------------------------
    const Tree &getTree() const { return tree; }
    Tree &getTree() { return tree; }

    size_t getGoalVID() const { return goal_vid; }
    size_t getRootVID() const { return root_vid; }

    std::vector<typename Planner<CostType>::WEdgePtr> getOptimalPlan() const {
        return getOptimalPlan(root_vid, goal_vid);
    }

    std::vector<typename Planner<CostType>::WEdgePtr>
    getOptimalPlan(size_t init_vid, size_t goal_vid) const override {
        return tree.getPath(init_vid, goal_vid);
    }

    OptimalSet<std::vector<typename Planner<CostType>::WEdgePtr>, CostType>
    getOptimalPlans(size_t init_vid, size_t goal_vid) const override {
        OptimalSet<std::vector<typename Planner<CostType>::WEdgePtr>, CostType>
            opt;
        opt.insert(getOptimalPlan(init_vid, goal_vid),
                   tree.getCostToCome(goal_vid), 0);
        return opt;
    }

  private:
    void initializeConstants() {
        double dim = space.getDimension();
        inv_dim = 1.0 / dim;
        double volume = space.getVolume();
        double unit_ball_volume = space.getUnitBallVolume();
        gamma_rrt = 2.0 * std::pow(1.0 + inv_dim, inv_dim) *
                    std::pow(volume / unit_ball_volume, inv_dim);
    }

    size_t connectToTree(const State &state) {
        auto nearest_v = tree.nearestVertex(state);
        if (!nearest_v)
            return std::numeric_limits<size_t>::max();

        StateTransition trans(nearest_v->state, state);
        if (collision_fn(trans))
            return std::numeric_limits<size_t>::max();

        CostType cost = cost_fn(trans);
        size_t new_vid = tree.addStateVertex(trans.getEndState());
        tree.addStateTransition(nearest_v->vid, new_vid, cost, trans);
        return new_vid;
    }

    double computeRadius(size_t n) const {
        if (n < 2)
            return std::numeric_limits<double>::infinity();
        return gamma_rrt * std::pow(std::log((double)n) / (double)n, inv_dim);
    }

    // -------------------------------------------------------
    // Try to rewire nearby vertices to this new node
    // -------------------------------------------------------
    void rewireNeighbors(size_t new_vid, double radius) {
        auto new_v = tree.getStateVertex(new_vid);
        for (const auto &[vid, vertex] : tree.getVertices()) {
            if (vid == new_vid)
                continue;
            auto sv = std::dynamic_pointer_cast<SVertex>(vertex);
            double d = sv->state.distance(new_v->state);
            if (d > radius)
                continue;

            StateTransition trans(new_v->state, sv->state);
            if (collision_fn(trans))
                continue;

            CostType trans_cost = cost_fn(trans);
            CostType new_cost_to_come =
                tree.getCostToCome(new_vid) + trans_cost;
            if (new_cost_to_come < tree.getCostToCome(vid)) {
                tree.rewire(new_vid, vid, trans_cost, trans);
            }
        }
    }
};
