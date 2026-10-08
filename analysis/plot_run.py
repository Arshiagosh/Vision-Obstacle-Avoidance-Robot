#!/usr/bin/env python3
"""Plot a run from its odometry log.

    python analysis/plot_run.py data/logs/obstacles_200_0.txt
    python analysis/plot_run.py runs/20240906_191209_goal_200_0.txt --goal 200 0 --show

Makes one figure with the path, x/y over time, the heading, and the PWM sent
to each motor. Saved next to the log unless --out says otherwise.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg") if "--show" not in sys.argv else None
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from runlog import Run, load  # noqa: E402

BLUE = "#2a78d6"
ORANGE = "#eb6834"
INK = "#0b0b0b"
INK_SOFT = "#52514e"
GRID = "#e4e3df"

plt.rcParams.update({
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "axes.edgecolor": INK_SOFT,
    "axes.labelcolor": INK,
    "axes.titlesize": 11,
    "axes.titleweight": "bold",
    "axes.titlelocation": "left",
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.8,
    "xtick.color": INK_SOFT,
    "ytick.color": INK_SOFT,
    "font.size": 10,
    "legend.frameon": False,
    "lines.linewidth": 2,
})


def plot(run: Run, goal=None, obstacles=(), title=None):
    fig, axes = plt.subplots(2, 2, figsize=(11, 7.5), constrained_layout=True)
    (ax_path, ax_xy), (ax_phi, ax_pwm) = axes

    # path
    ax_path.plot(run.x, run.y, color=BLUE)
    ax_path.plot(run.x[0], run.y[0], "o", color=INK, ms=7, label="start")
    ax_path.plot(run.x[-1], run.y[-1], "s", color=BLUE, ms=7, label="end")
    if goal is not None:
        ax_path.plot(goal[0], goal[1], "*", color=ORANGE, ms=14, label="goal")
    for i, (ox, oy) in enumerate(obstacles):
        ax_path.plot(ox, oy, "x", color=INK, ms=10, mew=2.5, label="obstacle" if i == 0 else None)
    ax_path.set_aspect("equal", adjustable="datalim")
    ax_path.set_xlabel("x [cm]")
    ax_path.set_ylabel("y [cm]")
    ax_path.set_title("Path (odometry)")
    ax_path.legend(loc="best")

    # x, y over time
    ax_xy.plot(run.t, run.x, color=BLUE, label="x")
    ax_xy.plot(run.t, run.y, color=ORANGE, label="y")
    ax_xy.set_xlabel("time [s]")
    ax_xy.set_ylabel("position [cm]")
    ax_xy.set_title("Position")
    ax_xy.legend(loc="best")

    # heading
    ax_phi.plot(run.t, run.heading_deg, color=BLUE)
    ax_phi.set_xlabel("time [s]")
    ax_phi.set_ylabel("heading [deg]")
    ax_phi.set_title("Heading")

    # motor commands
    ax_pwm.plot(run.t, run.pwm_left, color=BLUE, label="left", lw=1.5)
    ax_pwm.plot(run.t, run.pwm_right, color=ORANGE, label="right", lw=1.5)
    ax_pwm.set_ylim(-5, 265)
    ax_pwm.set_xlabel("time [s]")
    ax_pwm.set_ylabel("PWM duty [0-255]")
    ax_pwm.set_title("Motor commands")
    ax_pwm.legend(loc="best")

    end = f"ended at ({run.x[-1]:.1f}, {run.y[-1]:.1f}) cm, {run.heading_deg[-1]:.1f} deg"
    if goal is not None:
        miss = ((run.x[-1] - goal[0]) ** 2 + (run.y[-1] - goal[1]) ** 2) ** 0.5
        end += f"  |  {miss:.1f} cm from the goal"
    fig.suptitle(title or run.name, x=0.01, ha="left", fontsize=13, fontweight="bold", color=INK)
    # one line of numbers under the plots, in space kept free for it
    fig.get_layout_engine().set(rect=(0, 0.035, 1, 0.965))
    fig.text(0.01, 0.008, f"{run.duration:.1f} s, {run.path_length:.0f} cm driven  |  {end}",
             ha="left", va="bottom", fontsize=9, color=INK_SOFT)
    return fig


def _point(text):
    x, y = text.split(",")
    return float(x), float(y)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot a run from its odometry log.")
    parser.add_argument("log")
    parser.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"), help="mark the goal (cm)")
    parser.add_argument("--obstacle", action="append", type=_point, default=[], metavar="X,Y",
                        help="mark an obstacle (cm), can be repeated")
    parser.add_argument("--title")
    parser.add_argument("--out", help="output image (default: <log>.png)")
    parser.add_argument("--show", action="store_true", help="open a window instead of only saving")
    args = parser.parse_args()

    run = load(args.log)
    fig = plot(run, args.goal, args.obstacle, args.title)
    out = Path(args.out) if args.out else Path(args.log).with_suffix(".png")
    fig.savefig(out, dpi=150)
    print(f"{run.name}: {len(run)} samples, {run.duration:.1f} s -> {out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
