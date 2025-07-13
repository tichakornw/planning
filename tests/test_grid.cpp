#include <cassert>
#include <chrono>
#include <iostream>
#include <set>
#include <vector>

#include "OptimalSet.h"
#include "RulebookCost.h"
#include "RulebookPlanner.h"
#include "ScenarioGrid.h"
#include "WeightedGraph.h"

void testGridPlanner(bool iterated,
                     const std::vector<DiscreteProductState2D> &expected_path,
                     size_t num_experiments = 10) {
    using WEdge = WeightedEdge<RulebookCost>;
    using WEdgePtr = std::shared_ptr<WEdge>;

    ScenarioGrid scenario;
    RulebookPlanner planner(scenario);
    planner.setDebug(false);
    std::vector<WEdgePtr> edge_path;
    size_t total_duration_ms = 0;

    for (size_t i = 0; i < num_experiments; ++i) {
        auto start = std::chrono::high_resolution_clock::now();
        if (iterated)
            edge_path =
                planner.getOptimalPlan(scenario.getInit(), scenario.getGoal());
        else
            edge_path =
                planner.getDijkstraPlan(scenario.getInit(), scenario.getGoal());
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = duration_cast<std::chrono::milliseconds>(stop - start);
        total_duration_ms += duration.count();
        std::cout << "    it " << i << " time: " << duration.count()
                  << std::endl;
    }
    const double average_duration_ms =
        static_cast<double>(total_duration_ms) / num_experiments;
    const auto actual_path = scenario.getPStatePath(edge_path);
    std::cout << "  Path: ";
    for (const auto &state : actual_path)
        std::cout << state << " ";
    std::cout << std::endl;
    std::cout << "  Average time: " << average_duration_ms << std::endl;

    assert(actual_path.size() == expected_path.size());
    for (size_t i = 0; i < expected_path.size(); ++i) {
        assert(actual_path[i].x == expected_path[i].x &&
               actual_path[i].y == expected_path[i].y &&
               actual_path[i].q == expected_path[i].q);
    }
}

void testGrid() {
    std::cout << "Iterated Dijkstra's" << std::endl;
    const std::vector<DiscreteProductState2D> expected_optimal_path_iterated = {
        {0, 0, 0}, {1, 2, 0}, {1, 3, 0}, {2, 3, 0},
        {3, 3, 0}, {4, 3, 0}, {4, 4, 1}, {4, 5, 1}};
    testGridPlanner(true, expected_optimal_path_iterated);

    std::cout << "Dijkstra's" << std::endl;
    const std::vector<DiscreteProductState2D> expected_optimal_path_dijkstra = {
        {0, 0, 0}, {1, 2, 0}, {1, 1, 0}, {2, 1, 0}, {3, 1, 0},
        {4, 1, 0}, {4, 2, 0}, {4, 3, 0}, {4, 4, 1}, {4, 5, 1}};
    testGridPlanner(false, expected_optimal_path_dijkstra);
}
