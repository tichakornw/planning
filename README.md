# Motion Planning with Rulebooks

This repository provides the code base for **Motion Planning with Rulebooks**, a framework that formalizes task specifications and constraints into a structured, interpretable rule-based formulation.

## Installation

On Ubuntu 22.04 or later, you can follow the steps below to install all the necessary requirements.

1. Install the compiler

   ```
   $ sudo apt install cmake build-essential
   ```

2. Install this package

   ```
   $ cd planning
   $ make all
   ```

   This will create an executables `grid`, `navigation`, `avoidance` and `test` in the `build` folder.

## Usage
The repository includes three main example scenarios that demonstrate different planning algorithms:

1. Grid World Example
2. Navigation Example
3. Obstacle Avoidance Example

Each executable corresponds to one of these experiments.

### Grid World Example
The grid world example illustrates single-strategy control synthesis for discrete systems using rulebooks. It compares the proposed iterated algorithm against a baseline Dijkstra-based method.

**Run the example:**

```
./build/grid
```

**Output:**
* `results/comp_time_grid_iterated.json`: Computation time for the proposed iterated algorithm.
* `results/comp_time_grid_dijkstra.json`: Computation time for Dijkstra's algorithm.

**Plot results:**

```
python script/plot_grid.py
```

This script visualizes the performance comparison between the two algorithms.


### Navigation Example
The navigation example illustrates single-strategy control synthesis for continuous-state systemsimplemented using an RRT\*-style planner with rulebooks. It demonstrates how the rulebook framework can integrate with sampling-based motion planning to encode multiple prioritized objectives.

**Command-line options:**
* `--classical`: Run the classical RRT\* algorithm using only path length as the objective.
* `--stats`: Run multiple trials to collect runtime and cost statistics. The results are stored in `results/navigation_stats_<alg>.json`, where <alg> is either classical or rulebook.

These options can be used independently or together:
* Running with `--classical` applies the classical RRT\* algorithm.
* Running with both `--classical` and `--stats` collects statistics for the classical RRT\*.
* Running with only `--stats` collects statistics for the rulebook-based RRT\* algorithm.

**Run the default (rulebook-based) RRT\* algorithm:**
```
./build/navigation
```

**Run the classical RRT\* baseline:**
```
./build/navigation --classical
```

**Collect statistics for rulebook-based RRT\*:**
```
./build/navigation --stats
```

**Collect statistics for classical RRT\*:**
```
./build/navigation --classical --stats
```

**Output:**
Without `--stats`
* `results/world.json`: Environment configuration (obstacles, start, goal)
* `results/navigation_<alg>.json`: Planning results (tree, path, cost, computation time)

With `--stats`
* `results/navigation_stats_<alg>.json`: Runtime and cost statistics over multiple runs


### Obstacle Avoidance Example
The obstacle avoidance example illustrates complete control synthesis.

**Command-line options:**
* `--random`: Generates a random grid world or rulebooks, depending on whether `--rule` is specified,
with varying sizes. When used, this argument creates a grid world instead of the default obstacle avoidance scenario.
* `--rule`: Use in combination with `--random` to randomize rulebooks instead of worlds.
* `-n <num>`: Specifies the number of experiments to run.
If this argument is provided, the avoidance will execute the specified number of experiments for the given scenario.

**Run the default obstacle avoidance scenario:**
```
./build/avoidance
```

**Vary the size of grid world:**
```
./build/avoidance --random
```

**Vary the size of the rulebook:**
```
./build/avoidance --random --rule
```

** Specifies the number of experiments to run:**
```
./build/avoidance -n 5
```
