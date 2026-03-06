#ifndef _SCENARIO_GRID_H
#define _SCENARIO_GRID_H

#include <cmath>
#include <vector>
#include <memory>
#include <iostream>

#include "GridWorld.h"
#include "Rule.h"
#include "Rulebook.h"
#include "RulebookCost.h"
#include "ScenarioDiscrete.h"
#include "WeightedGraph.h"

class ScenarioGrid : public ScenarioDiscrete<DiscreteProductState2D> {
  public:
    using WEdge = WeightedEdge<RulebookCost>;
    using WEdgePtr = std::shared_ptr<WEdge>;

    // Default arguments go in the declaration
    ScenarioGrid(int xmax = 5, int ymax = 5);

    std::vector<DiscreteProductState2D>
    getPStatePath(const std::vector<WEdgePtr> &path) const;

    void printGraph(const WeightedGraph<RulebookCost> &graph) const override;

  private:
    // Reserving (x,y) = (0,0) for state marked as init when constructing
    // the product automaton so the world shouldn't contain this.
    const DiscreteRegion2D world;
    const DiscreteRegion2D p1_region;
    const DiscreteRegion2D p2_region;
    const DiscreteRegion2D obs_region;
    const double obs_clearance;
    const int qmin, qmax;
    
    // Available actions
    const std::vector<DiscreteAction2D> actions;
    
    // IDs of the rules
    size_t rid_ltl, rid_obs, rid_len;
    
    // initial and goal states in the world
    const DiscreteState2D sinit;
    
    // all vids for product states (sinit, q)
    std::vector<size_t> sinit_vids;
    // all vids for product states (init, q)
    std::vector<size_t> init_vids;

    void defineRulebook() override;
    void buildGraph() override;
    void addConnectionFrom(const DiscreteProductState2D &ps1);

    RulebookCost getCostFromInit(const DiscreteProductState2D &ps1,
                                 const DiscreteProductState2D &ps2);
    RulebookCost getCost(const DiscreteProductState2D &ps1,
                         const DiscreteProductState2D &ps2);
    double getLTLCost(const DiscreteProductState2D &ps1,
                      const DiscreteProductState2D &ps2);
    double getObsCost(const DiscreteProductState2D &ps1,
                      const DiscreteProductState2D &ps2);
    double getLength(const DiscreteProductState2D &ps1,
                     const DiscreteProductState2D &ps2);
};

#endif