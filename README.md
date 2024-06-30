# Discrete Motion Planning with Additive Rulebooks

This is the code base for discrete motion planning with additive rulebooks

## Installation

On Ubuntu 22.04, you can follow the following steps to install all the necessary requirements.

1. Install the compiler

   ```
   $ sudo apt install cmake build-essential
   ```

2. Install this package

   ```
   $ cd planning
   $ make all
   ```

   This will create an executable `planner` in the `build` folder.



## Usage
The `planner` takes 3 optional command line arguments: `--random` , `--rule` and `-n`.


### Default Behavior
By default, running the planner without any arguments will execute the obstacle avoidance scenario:

```
./build/planner
```

### Command-Line Arguments

1.  `--random`: Generates a random grid world or rulebooks, depending on whether `--rule` is specified,
with varying sizes.
When used, this argument creates a grid world instead of the default obstacle avoidance scenario.

    Example: The following command vary the size of grid world:

    ```
    ./build/planner --random
    ```

    Example: The following command vary the size of the rulebook:

    ```
    ./build/planner --random --rule
    ```

2.  `-n <number_of_experiments>`: Specifies the number of experiments to run.
If this argument is provided, the planner will execute the specified number of experiments for the given scenario.

    Example:

    ```
    ./build/planner -n 5
    ```
