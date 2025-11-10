#pragma once
#include "Scenario.h"

template <typename State> class ScenarioDiscrete : public Scenario<State> {
  public:
    using WEdge = WeightedEdge<RulebookCost>;
    using WEdgePtr = std::shared_ptr<WEdge>;

    ScenarioDiscrete() = default;

    ScenarioDiscrete(const State &sinit, const State &sgoal)
        : Scenario<State>(sinit, sgoal) {}

    ~ScenarioDiscrete() = default;

    size_t getGraphSize() const { return graph.getNumVertices(); }

    WeightedGraph<RulebookCost> &getGraph() { return graph; }
    const WeightedGraph<RulebookCost> &getGraph() const { return graph; }

    size_t getInit() const { return init_vid; }
    size_t getGoal() const { return goal_vid; }

    virtual void printGraph(const WeightedGraph<RulebookCost> &graph) const {
        if (vid2state.size() == 0) {
            graph.display();
            return;
        }

        for (const auto &pair : graph.getVertices()) {
            std::cout << "  " << vid2state.at(pair.first) << " ->";
            for (size_t i = 0; i < pair.second->out_edges.size(); ++i) {
                auto edge = pair.second->out_edges[i];
                auto weighted_edge = std::dynamic_pointer_cast<WEdge>(edge);
                std::cout << " (" << vid2state.at(weighted_edge->to->vid) << ","
                          << weighted_edge->cost << ")";
            }
            std::cout << std::endl;
        }
    }

  protected:
    WeightedGraph<RulebookCost> graph;
    // vertex id of init_state and goal_state
    size_t init_vid;
    size_t goal_vid;
    // map between vertex ID and state
    std::unordered_map<size_t, State> vid2state;
    std::unordered_map<State, size_t> state2vid;

    // Pure virtual function to be implemented by derived classes.
    virtual void buildGraph() = 0;

    void setup() override {
        Scenario<State>::setup();
        buildGraph();
    }

    size_t addState(const State &state) {
        const size_t vid = graph.addVertex();
        vid2state.insert({vid, state});
        state2vid.insert({state, vid});
        if (state == this->init_state)
            init_vid = vid;
        if (state == this->goal_state)
            goal_vid = vid;
        return vid;
    }
};
