import json
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict


def plot_navigation_stats(filename: str, show=True, save_prefix=None):
    """Plot timing and path cost statistics from RRT* experiment JSON."""

    cost_labels = ["Collision", "Busy region", "Clearance", "Right-hand rule", "Length"]

    with open(filename, "r") as f:
        data = json.load(f)

    runs = data["runs"]

    # --- Group by iteration count ---
    grouped = defaultdict(lambda: {"elapsed": [], "costs": []})
    for run in runs:
        it = run["iterations"]
        grouped[it]["elapsed"].append(run["elapsed_ms"])
        grouped[it]["costs"].append(run["path_cost"])

    # Sort iteration keys
    iteration_values = sorted(grouped.keys())

    # --- Prepare aggregate data ---
    elapsed_mean = []
    elapsed_std = []
    cost_means = []
    cost_stds = []

    for it in iteration_values:
        elapsed = np.array(grouped[it]["elapsed"]) / 1000.0  # convert ms to s
        costs = np.array(grouped[it]["costs"])

        elapsed_mean.append(elapsed.mean())
        elapsed_std.append(elapsed.std())

        cost_means.append(costs.mean(axis=0))
        cost_stds.append(costs.std(axis=0))

    cost_means = np.array(cost_means)
    cost_stds = np.array(cost_stds)

    # --- Plot elapsed time ---
    fig1, ax1 = plt.subplots(figsize=(6, 4))
    ax1.set_xscale("log")
    ax1.set_xlim(50, max(iteration_values) * 1.2)
    ax1.errorbar(iteration_values, elapsed_mean, yerr=elapsed_std, fmt="-o", capsize=4)
    ax1.set_xlabel("Iterations", fontsize=20)
    ax1.set_ylabel("Computation time (s)", fontsize=20)
    ax1.tick_params(axis="both", which="major", labelsize=16)
    ax1.grid(True, linestyle="--", alpha=0.5)

    if save_prefix:
        fig1.savefig(f"{save_prefix}_time.png", bbox_inches="tight")

    # --- Plot path cost components ---
    fig2, ax2 = plt.subplots(figsize=(7, 5))
    ax2.set_xscale("log")
    ax2.set_xlim(50, max(iteration_values) * 1.2)
    num_components = cost_means.shape[1]

    for j in range(num_components):
        label = ""
        if num_components > 1:
            label = f"{cost_labels[j]}"
        ax2.errorbar(
            iteration_values,
            cost_means[:, j],
            yerr=cost_stds[:, j],
            fmt="-o",
            capsize=4,
            label=label,
        )

    ax2.set_xlabel("Iterations", fontsize=20)
    ax2.set_ylabel("Path cost", fontsize=20)
    _, ymax = ax2.get_ylim()
    ax2.set_ylim(0, ymax)
    ax2.tick_params(axis="both", which="major", labelsize=16)
    ax2.grid(True, linestyle="--", alpha=0.5)
    if num_components > 1:
        ax2.legend(fontsize=16)

    if save_prefix:
        fig2.savefig(f"{save_prefix}_costs.png", bbox_inches="tight")

    if show:
        plt.show()
    else:
        plt.close("all")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Plot navigation planner stats from JSON."
    )
    parser.add_argument("json_file", help="Path to the statistics JSON file")
    parser.add_argument(
        "--save-prefix", help="Prefix to save output plots", default=None
    )
    parser.add_argument(
        "--no-show", action="store_true", help="Don't display the plots"
    )
    args = parser.parse_args()

    plot_navigation_stats(
        args.json_file, show=not args.no_show, save_prefix=args.save_prefix
    )
