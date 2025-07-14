#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <vector>

#include "OptimalSet.h"
#include "RulebookCost.h"
#include "RulebookPlanner.h"
#include "ScenarioGrid.h"
#include "WeightedGraph.h"

inline const std::vector<DiscreteProductState2D> &getEmptyPath() {
    static const std::vector<DiscreteProductState2D> empty;
    return empty;
}

std::map<size_t, std::vector<size_t>> testGridPlanner(
    bool iterated, std::vector<size_t> grid_sizes, size_t num_experiments = 1,
    bool verbose = false, bool check = false,
    const std::vector<DiscreteProductState2D> &expected_path = getEmptyPath()) {

    using WEdge = WeightedEdge<RulebookCost>;
    using WEdgePtr = std::shared_ptr<WEdge>;

    std::map<size_t, std::vector<size_t>> size_time_map;

    for (auto grid_size : grid_sizes) {
        ScenarioGrid scenario(grid_size, grid_size);
        RulebookPlanner planner(scenario);
        std::vector<WEdgePtr> edge_path;
        size_t total_duration_ms = 0;
        size_time_map[grid_size] = std::vector<size_t>();
        if (verbose)
            planner.setDebug(true);
        for (size_t i = 0; i < num_experiments; ++i) {
            auto start = std::chrono::high_resolution_clock::now();
            if (iterated)
                edge_path = planner.getOptimalPlan(scenario.getInit(),
                                                   scenario.getGoal());
            else
                edge_path = planner.getDijkstraPlan(scenario.getInit(),
                                                    scenario.getGoal());
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration =
                duration_cast<std::chrono::milliseconds>(stop - start);
            auto comp_time = duration.count();
            size_time_map[grid_size].push_back(comp_time);
            total_duration_ms += comp_time;
        }
        const auto actual_path = scenario.getPStatePath(edge_path);
        if (check) {
            assert(actual_path.size() == expected_path.size());
            for (size_t i = 0; i < expected_path.size(); ++i) {
                assert(actual_path[i].x == expected_path[i].x &&
                       actual_path[i].y == expected_path[i].y &&
                       actual_path[i].q == expected_path[i].q);
            }
        }
        if (verbose) {
            const double average_duration_ms =
                static_cast<double>(total_duration_ms) / num_experiments;
            std::cout << "  Path: ";
            for (const auto &state : actual_path)
                std::cout << state << ", ";
            std::cout << std::endl;
            std::cout << "  Average time: " << average_duration_ms << " ms"
                      << std::endl;
        }
    }

    return size_time_map;
}

void writeJson(const std::map<size_t, std::vector<size_t>> &data,
               const std::string &filename) {
    std::ofstream out(filename);
    out << "{\n";

    size_t count = 0;
    for (const auto &[key, vec] : data) {
        out << "  \"" << key * key << "\": [";
        for (size_t i = 0; i < vec.size(); ++i) {
            out << vec[i];
            if (i + 1 < vec.size())
                out << ", ";
        }
        out << "]";
        if (++count < data.size())
            out << ",";
        out << "\n";
    }

    out << "}\n";
}

void testGridDefault() {
    const std::vector<size_t> default_grid_sizes = {5};
    std::cout << "Iterated Dijkstra's" << std::endl;
    const std::vector<DiscreteProductState2D> expected_optimal_path_iterated = {
        {0, 0, 0}, {1, 2, 0}, {1, 3, 0}, {2, 3, 0},
        {3, 3, 0}, {4, 3, 0}, {4, 4, 1}, {4, 5, 1}};
    testGridPlanner(true, default_grid_sizes, 1, true, true,
                    expected_optimal_path_iterated);

    std::cout << "Dijkstra's" << std::endl;
    const std::vector<DiscreteProductState2D> expected_optimal_path_dijkstra = {
        {0, 0, 0}, {1, 2, 0}, {1, 1, 0}, {2, 1, 0}, {3, 1, 0},
        {4, 1, 0}, {4, 2, 0}, {4, 3, 0}, {4, 4, 1}, {4, 5, 1}};
    testGridPlanner(false, default_grid_sizes, 1, true, true,
                    expected_optimal_path_dijkstra);
}

void testGridVarySize() {
    const size_t num_experiments = 20;
    std::vector<size_t> grid_sizes;
    for (size_t gs = 10; gs <= 100; gs += 10) {
        grid_sizes.push_back(gs);
    }
    auto time_iterated =
        testGridPlanner(true, grid_sizes, num_experiments, false, false);
    auto time_dijkstra =
        testGridPlanner(false, grid_sizes, num_experiments, false, false);
    const std::string result_file_iterated =
        "results/comp_time_grid_iterated.json";
    const std::string result_file_dijkstra =
        "results/comp_time_grid_dijkstra.json";
    writeJson(time_iterated, result_file_iterated);
    writeJson(time_dijkstra, result_file_dijkstra);
}

void testGrid() {
    testGridDefault();
    testGridVarySize();
}
