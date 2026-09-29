#!/usr/bin/env python3
"""Plot CSV paths and dense rollouts produced by the C++ Kino A* demo."""

import argparse
import csv
import math
from pathlib import Path


def read_csv(path):
    metadata = {}
    with path.open(encoding="utf-8") as stream:
        lines = []
        for line in stream:
            if line.startswith("# "):
                key, value = line[2:].strip().split("=", 1)
                metadata[key] = value
            else:
                lines.append(line)
    rows = [{key: float(value) for key, value in row.items()}
            for row in csv.DictReader(lines)]
    if any(not math.isfinite(value) for row in rows for value in row.values()):
        raise ValueError(f"Non-finite CSV value: {path}")
    return metadata, rows


def load_scenario(directory, name):
    meta, path = read_csv(directory / f"{name}_path.csv")
    primitive_meta, primitives = read_csv(directory / f"{name}_primitives.csv")
    rollout_meta, rollout = read_csv(directory / f"{name}_rollout.csv")
    if meta != primitive_meta or meta != rollout_meta:
        raise ValueError(f"CSV metadata mismatch: {name}")
    if not path or not rollout or len(path) != len(primitives) + 1:
        raise ValueError(f"Inconsistent path/primitive sizes: {name}")
    elapsed = 0.0
    for index, primitive in enumerate(primitives):
        if primitive["duration"] <= 0 or not math.isclose(
                primitive["start_time"], elapsed, abs_tol=1e-10):
            raise ValueError(f"Invalid primitive time: {name}")
        elapsed += primitive["duration"]
        if not math.isclose(path[index + 1]["time"], elapsed, abs_tol=1e-10):
            raise ValueError(f"Path time mismatch: {name}")
    if any(a["time"] > b["time"] for a, b in zip(rollout, rollout[1:])):
        raise ValueError(f"Non-monotonic rollout: {name}")
    # Stitch/replay the dense C++ samples, retaining duplicate boundary times
    # because the velocity control can change instantly between primitives.
    for endpoint, sample in ((path[0], rollout[0]), (path[-1], rollout[-1])):
        if any(not math.isclose(endpoint[key], sample[key], abs_tol=1e-10)
               for key in ("time", "x", "y", "yaw")):
            raise ValueError(f"Rollout endpoint mismatch: {name}")
    return meta, path, primitives, rollout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", nargs="?", type=Path,
                        default=Path("/tmp/wbmm_kino_astar_demo"))
    parser.add_argument("--save", type=Path, help="Save the figure to this image path")
    parser.add_argument("--no-show", action="store_true", help="Use a noninteractive backend")
    args = parser.parse_args()
    if args.no_show and args.save is None:
        parser.error("--no-show requires --save")
    if args.no_show:
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 3, figsize=(14, 10), constrained_layout=True)
    for column, name in enumerate(("forward", "reverse", "rotate")):
        meta, path, primitives, rollout = load_scenario(args.directory, name)
        ax = axes[0, column]
        ax.plot([p["x"] for p in rollout], [p["y"] for p in rollout], label="C++ rollout")
        ax.plot([p["x"] for p in path], [p["y"] for p in path], ".", label="Search nodes")
        stride = max(1, len(path) // 10)
        arrows = path[::stride]
        ax.quiver([p["x"] for p in arrows], [p["y"] for p in arrows],
                  [math.cos(p["yaw"]) for p in arrows],
                  [math.sin(p["yaw"]) for p in arrows],
                  angles="xy", scale_units="xy", scale=5, color="tab:orange")
        ax.plot(float(meta["goal_x"]), float(meta["goal_y"]), "rx", label="Requested goal")
        ax.set_title(f"{name} | {meta['frame_id']}")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        xs = [p["x"] for p in rollout] + [float(meta["goal_x"])]
        ys = [p["y"] for p in rollout] + [float(meta["goal_y"])]
        half_span = max(max(xs) - min(xs), max(ys) - min(ys), 0.6) * 0.65
        mid_x, mid_y = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
        ax.set_xlim(mid_x - half_span, mid_x + half_span)
        ax.set_ylim(mid_y - half_span, mid_y + half_span)
        ax.set_aspect("equal", adjustable="box")
        ax.legend(fontsize=8)

        yaw_ax = axes[1, column]
        yaw_ax.plot([p["time"] for p in rollout], [p["yaw"] for p in rollout])
        yaw_ax.axhline(float(meta["goal_yaw"]), color="r", linestyle="--", label="Goal yaw")
        yaw_ax.set_xlabel("Time from start [s]")
        yaw_ax.set_ylabel("Wrapped yaw [rad]")
        yaw_ax.legend(fontsize=8)

        control_ax = axes[2, column]
        if primitives:
            times = [p["start_time"] for p in primitives]
            times.append(times[-1] + primitives[-1]["duration"])
            for key, label in (("v", "v [m/s]"), ("omega", "omega [rad/s]")):
                values = [p[key] for p in primitives]
                control_ax.step(times, values + [values[-1]], where="post", label=label)
        control_ax.set_xlabel("Time from start [s]")
        control_ax.set_ylabel("Piecewise constant controls")
        control_ax.legend(fontsize=8)
        for row in range(3):
            axes[row, column].grid(True, alpha=0.3)
        if meta["collision_checked"] != "1":
            ax.text(0.02, 0.02, "Collision checking DISABLED", transform=ax.transAxes,
                    fontsize=8, color="crimson")
    fig.suptitle("WBMM differential-drive Kino A* | Offline search initial guess")
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=160)
        print(f"Saved figure: {args.save.resolve()}")
    if not args.no_show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
