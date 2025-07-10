#ifndef _SCENARIO_GRID_H
#define _SCENARIO_GRID_H

#include <cmath>

#include "GridWorld.h"
#include "Rule.h"
#include "Rulebook.h"
#include "RulebookCost.h"
#include "Scenario.h"
#include "WeightedGraph.h"

class ScenarioGrid : public Scenario {
  public:
    using WEdge = WeightedEdge<RulebookCost>;
    using WEdgePtr = std::shared_ptr<WEdge>;

    ScenarioGrid()
        : world(1, 5, 1, 5), p1_region(3, 5, 1, 2), p2_region(5, 5, 4, 4),
          obs_region(1, 3, 4, 4), obs_clearance(2), qmin(0), qmax(1),
          sinit(1, 1), sgoal(4, 5), init_pstate(0, 0, 0),
          goal_pstate(sgoal.x, sgoal.y, 0) {
        setup();
    }

    size_t getVidQinit() { return qinit_vid; }

    size_t getVidQgoal() { return qgoal_vid; }

    std::vector<DiscreteProductState2D>
    getPStatePath(const std::vector<WEdgePtr> &path) const {
        std::vector<DiscreteProductState2D> pstate_path;
        if (path.empty())
            return pstate_path;

        pstate_path.push_back(vid2pstate.at(path[0]->from->vid));

        for (const auto &edge : path) {
            pstate_path.push_back(vid2pstate.at(edge->to->vid));
        }

        return pstate_path;
    }

    void printGraph(const WeightedGraph<RulebookCost> &graph) const override {
        for (const auto &pair : graph.getVertices()) {
            std::cout << "  " << vid2pstate.at(pair.first) << " ->";
            for (size_t i = 0; i < pair.second->out_edges.size(); ++i) {
                auto edge = pair.second->out_edges[i];
                auto weighted_edge = std::dynamic_pointer_cast<WEdge>(edge);
                std::cout << " (" << vid2pstate.at(weighted_edge->to->vid)
                          << "," << weighted_edge->cost << ")";
            }
            std::cout << std::endl;
        }
    }

  protected:
    void setup() override {
        assert(init_pstate.q >= qmin);
        assert(init_pstate.q <= qmax);
        assert(goal_pstate.q >= qmin);
        assert(goal_pstate.q <= qmax);
        assert(world.contain(sinit));
        assert(world.contain(sgoal));
        assert(world.contain(goal_pstate));
        buildRulebook();
        RulebookCost::setRulebook(rulebook);
        buildGraph();
    }

  private:
    // Reserving (x,y) = (0,0) for state marked as init when constructing
    // the product automaton so the world shouldn't contain this.
    const DiscreteRegion2D world;
    const DiscreteRegion2D p1_region;
    const DiscreteRegion2D p2_region;
    const DiscreteRegion2D obs_region;
    const double obs_clearance;
    const int qmin;
    const int qmax;
    // Available actions
    const std::vector<DiscreteAction2D> actions = {
        DiscreteAction2D(0, 1), DiscreteAction2D(1, 0), DiscreteAction2D(0, -1),
        DiscreteAction2D(-1, 0)};
    // IDs of the rules
    size_t rid_ltl, rid_obs, rid_len;
    // initial and goal states in the world
    const DiscreteState2D sinit;
    const DiscreteState2D sgoal;
    // initial and goal states in the product automaton
    const DiscreteProductState2D init_pstate;
    const DiscreteProductState2D goal_pstate;
    // vertex id of the init and goal states in the product automaton
    size_t qinit_vid, qgoal_vid;
    // all vids for product states (sinit, q)
    std::vector<size_t> sinit_vids;
    // all vids for product states (init, q)
    std::vector<size_t> init_vids;
    // map between vertex ID and product automaton state
    std::unordered_map<size_t, DiscreteProductState2D> vid2pstate;
    std::unordered_map<DiscreteProductState2D, size_t> pstate2vid;

    void buildRulebook() {
        const RuleSum r0("ltl");
        const RuleMax r1("obs");
        const RuleSum r2("len");

        rid_ltl = rulebook.addRule(r0);
        rid_obs = rulebook.addRule(r1);
        rid_len = rulebook.addRule(r2);

        rulebook.addGTRelation(rid_ltl, rid_obs);
        rulebook.addGTRelation(rid_obs, rid_len);

        rulebook.build();

        displayRulebook();
    }

    void buildGraph() {
        const DiscreteProductState2D tmp(1, 1, 0);

        // init state (0,0)
        for (int q = qmin; q <= qmax; ++q) {
            const DiscreteProductState2D pstate(0, 0, q);
            const size_t vid = addState(pstate);
            init_vids.push_back(vid);
            if (pstate == init_pstate)
                qinit_vid = vid;
        }

        // (x, y, q) for (x, y) in world
        for (int x = world.xmin; x <= world.xmax; ++x) {
            for (int y = world.ymin; y <= world.ymax; ++y) {
                for (int q = qmin; q <= qmax; ++q) {
                    const DiscreteProductState2D pstate(x, y, q);
                    const size_t vid = addState(pstate);
                    if (x == sinit.x && y == sinit.y)
                        sinit_vids.push_back(vid);
                    if (pstate == goal_pstate)
                        qgoal_vid = vid;
                }
            }
        }

        // Connections from (init, q)
        for (auto vertex1 : init_vids) {
            for (auto vertex2 : sinit_vids) {
                const auto ps1 = vid2pstate.at(vertex1);
                const auto ps2 = vid2pstate.at(vertex2);
                graph.addEdge(vertex1, vertex2, getCostFromInit(ps1, ps2));
            }
        }

        // Other connections
        for (int x = world.xmin; x <= world.xmax; ++x) {
            for (int y = world.ymin; y <= world.ymax; ++y) {
                for (int q = qmin; q <= qmax; ++q) {
                    const DiscreteProductState2D ps1(x, y, q);
                    addConnectionFrom(ps1);
                }
            }
        }
    }

    void addConnectionFrom(const DiscreteProductState2D &ps1) {
        size_t vertex1 = pstate2vid.at(ps1);
        for (const auto &action : actions) {
            DiscreteProductState2D ps2 = ps1 + action;
            if (!world.contain(ps2))
                continue;
            for (int q = 0; q <= 1; ++q) {
                ps2.q = q;
                size_t vertex2 = pstate2vid.at(ps2);
                graph.addEdge(vertex1, vertex2, getCost(ps1, ps2));
            }
        }
    }

    RulebookCost getCostFromInit(const DiscreteProductState2D &ps1,
                                 const DiscreteProductState2D &ps2) {
        RulebookCost cost;
        cost.setRuleCost(rid_ltl, getLTLCost(ps1, ps2));
        cost.setRuleCost(rid_obs, getObsCost(ps1, ps2));
        cost.setRuleCost(rid_len, 0.0);
        return cost;
    }

    RulebookCost getCost(const DiscreteProductState2D &ps1,
                         const DiscreteProductState2D &ps2) {
        RulebookCost cost;
        cost.setRuleCost(rid_ltl, getLTLCost(ps1, ps2));
        cost.setRuleCost(rid_obs, getObsCost(ps1, ps2));
        cost.setRuleCost(rid_len, getLength(ps1, ps2));
        return cost;
    }

    double getLTLCost(const DiscreteProductState2D &ps1,
                      const DiscreteProductState2D &ps2) {
        // q0 -> q0
        if (ps1.q == 0 && ps2.q == 0) {
            return p1_region.contain(ps2) ? 1.0 : 0.0;
        }
        if (ps1.q == 0 && ps2.q == 1) {
            return p2_region.contain(ps2) ? 0.0 : 1.0;
        }
        if (ps1.q == 1 && ps2.q == 1) {
            return 0.0;
        }
        return 1.0;
    }

    double getObsCost(const DiscreteProductState2D &ps1,
                      const DiscreteProductState2D &ps2) {
        const double min_dist =
            std::min(obs_region.distanceTo(ps1), obs_region.distanceTo(ps2));
        return std::max(0.0, obs_clearance - min_dist);
    }

    double getLength(const DiscreteProductState2D &ps1,
                     const DiscreteProductState2D &ps2) {
        return std::sqrt((ps1.x - ps2.x) * (ps1.x - ps2.x) +
                         (ps1.y - ps2.y) * (ps1.y - ps2.y));
    }

    size_t addState(const DiscreteProductState2D &pstate) {
        const size_t vid = graph.addVertex();
        vid2pstate.insert({vid, pstate});
        pstate2vid.insert({pstate, vid});
        return vid;
    }
};

#endif
