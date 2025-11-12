#pragma once

#include "StateGraph.h"
#include "World2D.h"
#include <fstream>
#include <iomanip>
#include <memory>
#include <vector>

template <typename State, typename CostType, typename StateTransition>
void savePlanningResultJson(
    const std::string &filename,
    const StateGraph<State, CostType, StateTransition> &graph,
    const std::vector<
        std::shared_ptr<StateTransitionEdge<State, CostType, StateTransition>>>
        &path,
    const State &start_state, const State &goal_state, double elapsed_ms,
    const CostType &path_cost) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "Error: cannot open " << filename << " for writing\n";
        return;
    }

    ofs << std::fixed << std::setprecision(6); // nicer floating point output
    ofs << "{\n";

    // Save graph
    ofs << "  \"graph\": ";
    graph.toJsonStream(ofs);
    ofs << ",\n";

    // Save path
    ofs << "  \"path\": ";
    toJsonStreamEdges(ofs, path);
    ofs << ",\n";

    // Add start and goal
    ofs << "  \"start\": {\"x\": " << start_state.x
        << ", \"y\": " << start_state.y << "},\n";
    ofs << "  \"goal\":  {\"x\": " << goal_state.x
        << ", \"y\": " << goal_state.y << "},\n";

    // --- Metadata ---
    ofs << "  \"elapsed_ms\": " << elapsed_ms << ",\n";
    ofs << "  \"path_cost\": " << path_cost << "\n";

    ofs << "}\n";

    ofs.close();
}

void saveWorldJson(const std::string &filename, const World2D &world,
                   double clearance) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "Error: cannot open " << filename << " for writing\n";
        return;
    }

    ofs << std::fixed << std::setprecision(6); // nicer floating point output
    ofs << "{\n";

    // Save world
    ofs << "  \"world\": ";
    world.toJsonStream(ofs);
    ofs << ",\n";

    // Add clearance
    ofs << "  \"clearance\": " << clearance << "\n";

    ofs << "}\n";
    ofs.close();
}
