#include "ScenarioNavigation.h"
#include "ScenarioNavigationClassical.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

// From navigation.cpp
void testNavigation(size_t iterations, size_t retry, bool classical);

std::pair<RulebookCost, double>
runNavigationPlanning(const ScenarioNavigation &scenario, size_t iterations,
                      size_t retry, const std::string &plan_filename = "");

ScenarioNavigation setupNavigationWorld(const std::string &world_filename = "");

ScenarioNavigationClassical
setupNavigationWorldClassical(const std::string &world_filename = "");

struct Arguments {
    bool get_stats = false;
    bool save_results = false;
    bool classical = false;
};

Arguments parse_args(int argc, char *argv[]) {
    Arguments args;

    args.get_stats = false;
    args.save_results = false;
    // Check if the optional arguments are provided
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--stats") {
            args.get_stats = true;
        }
        if (arg == "--save") {
            args.save_results = true;
        }
        if (arg == "--classical") {
            args.classical = true;
        }
    }
    return args;
}

std::vector<std::pair<RulebookCost, double>>
runPlanner(const ScenarioNavigation &scenario, size_t num_runs,
           size_t iterations) {
    std::vector<std::pair<RulebookCost, double>> results;
    results.reserve(num_runs);

    // -------------------------------
    // Run planning multiple times
    // -------------------------------
    std::cout << "Iterations: " << iterations << "\n";
    for (size_t i = 0; i < num_runs; ++i) {
        auto [total_cost, elapsed_ms] =
            runNavigationPlanning(scenario, iterations, 10);
        results.emplace_back(total_cost, elapsed_ms);
        std::cout << "  Run " << i + 1 << "/" << num_runs
                  << ": cost = " << total_cost << ", time = " << elapsed_ms
                  << " ms\n";
    }
    return results;
}

void toJsonStreamStats(
    std::ostream &os,
    const std::vector<std::pair<RulebookCost, double>> &results,
    size_t iterations, bool is_last) {
    os << std::fixed << std::setprecision(6);

    for (size_t i = 0; i < results.size(); ++i) {
        const auto &[cost, time] = results[i];
        os << "    {\n";
        os << "      \"iterations\": " << iterations << ",\n";
        os << "      \"elapsed_ms\": " << time << ",\n";
        os << "      \"path_cost\": " << cost << "\n";
        os << "    }";
        if (!is_last || i + 1 < results.size())
            os << ",";
        os << "\n";
    }
}

void collectStats(bool classical) {
    const std::vector<size_t> all_iterations = {100,  167,  278,  464,  774,
                                                1292, 2154, 3594, 5995, 10000};
    std::unique_ptr<ScenarioNavigation> scenario;
    if (classical)
        scenario = std::make_unique<ScenarioNavigationClassical>(
            setupNavigationWorldClassical());
    else
        scenario = std::make_unique<ScenarioNavigation>(setupNavigationWorld());

    std::string results_file = "results/navigation_stats";
    if (classical)
        results_file += "_classical.json";
    else
        results_file += "_rulebook.json";
    std::ofstream ofs(results_file);
    if (!ofs.is_open()) {
        std::cerr << "Error: cannot open " << results_file << " for writing\n";
        return;
    }

    ofs << "{\n";
    ofs << "  \"runs\": [\n";

    for (size_t i = 0; i < all_iterations.size(); ++i) {
        size_t iterations = all_iterations[i];
        size_t num_runs = 50;
        if (iterations > 2000)
            num_runs = 10;
        auto results = runPlanner(*scenario, num_runs, iterations);
        bool is_last = (i + 1 == all_iterations.size());
        toJsonStreamStats(ofs, results, iterations, is_last);
    }
    ofs << "  ]\n";
    ofs << "}\n";
    ofs.close();

    std::cout << "Saved results to " << results_file << "\n";
}

int main(int argc, char *argv[]) {
    const Arguments args = parse_args(argc, argv);
    if (args.get_stats)
        collectStats(args.classical);
    else
        testNavigation(1000, 10, args.classical);
}
