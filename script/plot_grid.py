import json
import matplotlib.pyplot as plt
import numpy as np


def get_time_stat(data):
    problem_sizes = sorted(map(int, data.keys()))
    means = []
    std_devs = []

    for size in problem_sizes:
        times = data[str(size)]
        times = [t * 1e-3 for t in times]
        means.append(np.mean(times))
        std_devs.append(np.std(times))

    return (problem_sizes, means, std_devs)


# Load data
with open("results/comp_time_grid_iterated.json") as f1:
    time_iterated = json.load(f1)

with open("results/comp_time_grid_dijkstra.json") as f2:
    time_dijkstra = json.load(f2)

# Process and sort
(problem_sizes_iterated, means_iterated, std_devs_iterated) = get_time_stat(
    time_iterated
)
(problem_sizes_dijkstra, means_dijkstra, std_devs_dijkstra) = get_time_stat(
    time_dijkstra
)

# Plot
plt.rcParams["text.usetex"] = True
plt.figure(figsize=(8, 5))
plt.errorbar(
    problem_sizes_iterated,
    means_iterated,
    yerr=std_devs_iterated,
    fmt="o-",
    capsize=5,
    ecolor="blue",
    label="Algorithm 1",
)
plt.errorbar(
    problem_sizes_dijkstra,
    means_dijkstra,
    yerr=std_devs_dijkstra,
    fmt="x--",
    capsize=5,
    ecolor="red",
    label="Dijkstra's",
)
plt.xlabel("State space size", size=20)
plt.ylabel("Computation time (seconds)", size=20)
plt.tick_params(axis="x", labelsize=20)
plt.tick_params(axis="y", labelsize=20)
plt.grid(True)
plt.legend(fontsize=20)
plt.tight_layout()
plt.savefig("timing_plot.png")
plt.show()
