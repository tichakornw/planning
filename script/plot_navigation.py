#!/usr/bin/env python3
import json
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Polygon
import matplotlib.patches as mpatches
import numpy as np


def plot_world(ax, world_data):
    """Plot the 2D world with bounds, obstacles, and regions."""
    # --- World bounds ---
    world = world_data.get("world", {})
    if len(world) == 0:
        return

    clearance = world_data.get("clearance", 0)

    bounds = world.get("bounds", {})
    xmin, xmax = bounds.get("xmin", 0), bounds.get("xmax", 10)
    ymin, ymax = bounds.get("ymin", 0), bounds.get("ymax", 10)
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)

    # --- Regions (rectangles) ---
    for region in world.get("regions", []):
        rect = Rectangle(
            (region["xmin"], region["ymin"]),
            region["xmax"] - region["xmin"],
            region["ymax"] - region["ymin"],
            facecolor="orange",
            edgecolor="darkorange",
            alpha=0.3,
            label="Busy region",
        )
        ax.add_patch(rect)

    # --- Obstacles (circles) ---
    for obstacle in world.get("obstacles", []):
        cx, cy, r = obstacle["x"], obstacle["y"], obstacle["radius"]

        # Clearance area (slightly transparent ring)
        if clearance > 0:
            clearance_circle = Circle(
                (cx, cy),
                r + clearance,
                facecolor="none",
                edgecolor="purple",
                linestyle="--",
                linewidth=1.2,
                alpha=0.5,
                label="Clearance"
                if "Clearance" not in [t.get_text() for t in ax.texts]
                else "",
            )
            ax.add_patch(clearance_circle)

        # Actual obstacle
        circ = Circle(
            (cx, cy),
            r,
            facecolor="red",
            alpha=0.3,
            edgecolor="none",
            label="Obstacles"
            if "Obstacles" not in [t.get_text() for t in ax.texts]
            else "",
        )
        ax.add_patch(circ)


def plot_rrtstar_result(result_file: str, world_file: str, show=True, save_as=None):
    """Plot the RRT* tree and path from a JSON result file."""
    with open(result_file, "r") as f:
        data = json.load(f)

    tree = data.get("tree", data.get("graph", {}))  # handle both keys
    vertices = tree.get("vertices", [])
    edges = tree.get("edges", [])
    path = data.get("path", [])

    # --- Load world ---
    with open(world_file, "r") as f:
        world_data = json.load(f)

    # Extract vertex coordinates
    xs = [v["x"] for v in vertices]
    ys = [v["y"] for v in vertices]

    # Set up plot
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    # ax.set_title(f"RRT* Result\nTime: {data['elapsed_ms']:.2f} ms, Cost: {data['path_cost']}")

    # Plot world (regions + obstacles)
    plot_world(ax, world_data)

    # Plot tree edges
    for e in edges:
        ax.plot(
            [e["x1"], e["x2"]],
            [e["y1"], e["y2"]],
            color="gray",
            linewidth=0.5,
            alpha=0.6,
            label="RRT* tree"
            if "RRT* tree" not in [t.get_text() for t in ax.texts]
            else "",
        )

    # Plot path edges
    offset = 5.0
    shadow_color = (0.2, 0.2, 0.8, 0.15)
    for e in path:
        x1, y1, x2, y2 = e["x1"], e["y1"], e["x2"], e["y2"]
        ax.plot([x1, x2], [y1, y2], color="blue", linewidth=2.5, label="Optimal path")

        # Compute right-hand side normal
        dx, dy = x2 - x1, y2 - y1
        length = np.hypot(dx, dy)
        if length < 1e-6:
            continue
        nx, ny = dy / length, -dx / length  # Right-hand normal

        # Construct a polygon offset to the right
        shadow = np.array(
            [
                [x1, y1],
                [x2, y2],
                [x2 + nx * offset, y2 + ny * offset],
                [x1 + nx * offset, y1 + ny * offset],
            ]
        )
        patch = Polygon(shadow, closed=True, facecolor=shadow_color, edgecolor=None)
        ax.add_patch(patch)

    # Plot start and goal if provided
    start = data.get("start")
    goal = data.get("goal")

    if start and goal:
        ax.scatter(start["x"], start["y"], c="green", s=80, label="Start", zorder=5)
        ax.scatter(goal["x"], goal["y"], c="blue", s=80, label="Goal", zorder=5)

    # --- Clean aesthetics ---
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xlabel("")
    ax.set_ylabel("")
    ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

    # --- Legend and grid ---
    handles, labels = ax.get_legend_handles_labels()
    shadow_patch = mpatches.Patch(color=shadow_color, label="Right-hand clearance zone")
    handles.append(shadow_patch)
    labels.append("Right-hand clearance zone")
    unique = dict(zip(labels, handles))  # remove duplicate legend items
    ax.legend(unique.values(), unique.keys(), loc="upper left")
    ax.grid(True, linestyle="--", alpha=0.5)

    if save_as:
        plt.savefig(save_as, bbox_inches="tight")
    if show:
        plt.show()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Plot RRT* result from JSON file.")
    parser.add_argument(
        "result_file", help="Path to JSON file produced by savePlanningResultJson()"
    )
    parser.add_argument(
        "world_file", help="Path to JSON file produced by saveWorldJson()"
    )
    parser.add_argument(
        "--save", help="Save plot to file instead of showing", default=None
    )
    parser.add_argument("--no-show", action="store_true", help="Don't display the plot")
    args = parser.parse_args()

    plot_rrtstar_result(
        args.result_file, args.world_file, show=not args.no_show, save_as=args.save
    )
