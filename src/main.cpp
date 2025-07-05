#include <chrono>
#include <fstream>
#include <iostream>

#include "RulebookPlanner.h"
#include "ScenarioAvoidance.h"
#include "ScenarioRandom.h"

using namespace std::chrono;

struct Arguments {
    bool random = false;
    bool change_rule_size = false;
    size_t num_experiments = 5;
    size_t refinement = 0;
};

void addResult(std::unique_ptr<Scenario> scenario, std::ofstream &outdata) {
    RulebookPlanner planner(*scenario);
    auto start = high_resolution_clock::now();
    const auto res = planner.getOptimalPlans();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    // auto duration = duration_cast<microseconds>(stop - start);

    outdata << "{\"psize\": " << planner.getGraphSize() << ", ";
    outdata << "\"rsize\": " << planner.getNumRules() << ", ";
    outdata << "\"time\": " << duration.count() << ", \"plans\": " << res;
    outdata << "}";
}

Arguments parse_args(int argc, char *argv[]) {
    Arguments args;

    args.random = false;
    args.change_rule_size = false;
    // Check if the optional argument --sim is provided
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--rand" || arg == "--random") {
            args.random = true;
        } else if (arg == "--rule") {
            args.change_rule_size = true;
        } else if (arg == "--n" && i + 1 < argc) {
            args.num_experiments = atoi(argv[i + 1]);
            ++i; // Skip the next argument (file path)
        } else if (arg == "--refine" && i + 1 < argc) {
            args.refinement = atoi(argv[i + 1]);
        }
    }
    return args;
}

int main(int argc, char *argv[]) {
    const Arguments args = parse_args(argc, argv);

    const size_t max_grid = 14;
    const size_t max_rules = 10;
    std::string result_file = "results/result_";
    if (args.random) {
        result_file += "random";
        if (!args.change_rule_size)
            result_file += "_graph";
        else
            result_file += "_rule";
    } else
        result_file += "avoidance";
    result_file += ".json";

    std::unique_ptr<Scenario> scenario;
    std::ofstream outdata;
    outdata.open(result_file);
    if (!outdata) {
        std::cerr << "Error: file " << result_file << " could not be opened"
                  << std::endl;
        exit(1);
    }

    std::cout << "random?: " << args.random << std::endl;
    std::cout << "rule?:   " << args.change_rule_size << std::endl;
    std::cout << "num exp: " << args.num_experiments << std::endl;
    std::cout << "refinement: " << args.refinement << std::endl;

    outdata << "[";

    for (size_t i = 0; i < args.num_experiments; ++i) {
        std::cout << "Exp " << i + 1 << std::endl;
        if (args.random) {
            if (!args.change_rule_size) {
                for (size_t num_x = 5; num_x < max_grid; ++num_x) {
                    for (size_t num_y = 5; num_y < max_grid; ++num_y) {
                        scenario =
                            std::make_unique<ScenarioRandom>(num_x, num_y, 5);
                        addResult(std::move(scenario), outdata);
                        if (i + 1 < args.num_experiments ||
                            num_x + 1 < max_grid || num_y + 1 < max_grid)
                            outdata << ", ";
                    }
                }
            } else {
                for (size_t num_rules = 1; num_rules <= max_rules;
                     ++num_rules) {
                    scenario =
                        std::make_unique<ScenarioRandom>(10, 10, num_rules);
                    addResult(std::move(scenario), outdata);
                    if (i + 1 < args.num_experiments ||
                        num_rules + 1 <= max_rules)
                        outdata << ", ";
                }
            }
        } else {
            scenario = std::make_unique<ScenarioAvoidance>(args.refinement);
            addResult(std::move(scenario), outdata);
            if (i + 1 < args.num_experiments)
                outdata << ", ";
        }
    }
    outdata << "]";

    return 0;
}
