import json
import math
import matplotlib.pyplot as plt
import matplotlib as mpl
from collections import defaultdict


class Result:
    def __init__(self, problem_size, rule_size, comp_time, plans):
        self.problem_size = problem_size
        self.rule_size = rule_size
        self.comp_time = comp_time
        self.plans = plans


def read_result(result):
    problem_size = result["psize"]
    rule_size = result["rsize"]
    comp_time = result["time"]
    plans = result["plans"]
    return Result(problem_size, rule_size, comp_time, plans)


def parse_results(filename):
    with open(filename) as result_file:
        result_data = json.load(result_file)

    results = []
    for result in result_data:
        results.append(read_result(result))
    return results


def get_problem_time(results):
    problem_time = []
    for result in results:
        problem_time.append((result.problem_size, result.comp_time))
    return problem_time


def get_rule_time(results):
    rule_time = []
    for result in results:
        rule_time.append((result.rule_size, result.comp_time))
    return rule_time


def plot_results(results_time, xlabel):
    x_vals = [elem[0] for elem in results_time]
    y_vals = [elem[1] / 1000.0 for elem in results_time]

    # Group indices by x-value
    groups = defaultdict(list)
    for i, x in enumerate(x_vals):
        groups[x].append(i)

    # Use a colormap
    cmap = plt.get_cmap("viridis")
    unique_x = sorted(groups.keys())
    colors = {x: cmap(i / max(len(unique_x) - 1, 1)) for i, x in enumerate(unique_x)}

    plt.figure()
    for x in unique_x:
        indices = groups[x]
        plt.scatter(
            [x] * len(indices),
            [y_vals[i] for i in indices],
            color=colors[x],
            s=100,
            label=f"x={x}",
        )

    plt.xlabel(xlabel, size=40)
    plt.ylabel("Computation time (seconds)", size=40)
    plt.yscale("log")
    plt.tick_params(axis="x", labelsize=40)
    plt.tick_params(axis="y", labelsize=40)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Plot grid world stats from JSON.")
    parser.add_argument(
        "result_file",
        nargs="?",  # make it optional
        default="results/result_random_rule.json",  # default file
        help="Path to the statistics JSON file (default: %(default)s)",
    )
    args = parser.parse_args()
    print("Using JSON file:", args.result_file)

    plt.rcParams["text.usetex"] = True
    results = parse_results(args.result_file)

    problem_time = get_problem_time(results)
    plot_results(problem_time, "State space size")
    rule_time = get_rule_time(results)
    plot_results(rule_time, "Number of rules")

    plt.show()
