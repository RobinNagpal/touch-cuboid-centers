"""Draws the pictures in TILT_ON_LEGS.md.

Run from v3/figures/:  python tilt_on_legs.py

Sketches, not to scale, except the leg chart, which uses v2's numbers.
"""

from __future__ import annotations

import numpy as np
from matplotlib.patches import Circle, Polygon, Rectangle
from top_from_wall import (
    BAD,
    BG,
    BOARD_T,
    BOARD_W,
    GOOD,
    INK,
    INK2,
    LEG,
    PATH,
    TOP,
    arrow,
    gripper,
    leg,
    note,
    plt,
    save,
    scene,
    unit,
)

# The far legs' tops are the pivot. The board turns about its corner resting there.
PIVOT = np.array([6.8, 1.4])
NEAR_LEG = 5.27


def board_about(ax, corner, deg, alpha=1.0, z=4):
    """A board with one bottom corner at ``corner``, running at ``deg`` from it. Returns its gripped edge."""
    d = unit(deg)
    o = np.array([d[1], -d[0]]) * BOARD_T
    far = corner + d * BOARD_W
    ax.add_patch(Polygon([corner, far, far + o, corner + o], fc=TOP, ec="#b8491c", lw=1, alpha=alpha, zorder=z))
    return far + o / 2, -d


def legs(ax):
    leg(ax, NEAR_LEG)
    leg(ax, PIVOT[0])


def held(ax, corner, deg, alpha=1.0, open_=False, back=0.0):
    edge, u = board_about(ax, corner, deg, alpha=alpha)
    gripper(ax, edge - u * back, u, alpha=alpha, open_=open_)


def eye(ax, xy):
    ax.add_patch(Circle(xy, 0.26, fc="white", ec=INK2, lw=1.5, zorder=8))
    ax.add_patch(Circle(xy, 0.1, fc=INK2, zorder=9))


# ---------------------------------------------------------------- the steps


def steps():
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    axes = axes.ravel()
    titles = ["1. Carry it hanging, over the far legs", "2. Look, then lower it slowly",
              "3. The corner touches the far legs", "4. Tilt about that corner, 5° at a time",
              "5. The near edge lands on the near legs", "6. Let go, pull back out"]
    for ax, title in zip(axes, titles, strict=True):
        scene(ax, 3.2, 8.4, -0.3, 5.6, title)
        legs(ax)

    held(axes[0], PIVOT + np.array([0, 0.5]), 90)
    arrow(axes[0], (3.5, 4.0), (5.6, 4.4), rad=-0.3)
    note(axes[0], (3.35, 2.9), "picked from the wall as in\nTOP_FROM_WALL.md, steps 1–4")

    held(axes[1], PIVOT + np.array([0, 0.5]), 90)
    eye(axes[1], (5.4, 4.6))
    axes[1].plot([4.8, 7.6], [1.4, 1.4], color=PATH, ls="--", lw=1.2)
    note(axes[1], (3.4, 2.1), "measure the leg tops\nand the lower edge")
    arrow(axes[1], (7.5, 3.0), (7.5, 2.2), lw=1.5)

    held(axes[2], PIVOT, 90)
    axes[2].add_patch(Circle(PIVOT, 0.09, fc=GOOD, zorder=9))
    note(axes[2], (3.4, 2.6), "stop when the arm feels\nthe load come off it")

    ax = axes[3]
    for i, deg in enumerate(range(90, 181, 10)):
        board_about(ax, PIVOT, deg, alpha=0.1 + 0.5 * (i / 9) ** 2)
    held(ax, PIVOT, 135)
    ax.add_patch(Circle(PIVOT, 0.09, fc=PATH, zorder=9))
    arrow(ax, PIVOT + unit(95) * 2.15, PIVOT + unit(172) * 2.15, rad=0.35)
    note(ax, (6.95, 0.8), "pivot")

    held(axes[4], PIVOT, 180)
    for x in (NEAR_LEG, PIVOT[0]):
        axes[4].add_patch(Circle((x, 1.4), 0.08, fc=GOOD, zorder=9))

    held(axes[5], PIVOT, 180, open_=True, back=0.9)
    arrow(axes[5], (5.0, 2.4), (4.1, 2.4), lw=1.5)
    note(axes[5], (5.9, 3.4), "then look and\ncheck the table", ha="center")
    fig.suptitle("The human way: rest one edge on the legs, then tilt it down (side view)",
                 x=0.01, ha="left", fontsize=13, fontweight="bold")
    fig.tight_layout()
    save(fig, "tilt_steps.png")


# ---------------------------------------------------------------- who carries the weight


def load_sharing():
    fig, (a, b) = plt.subplots(1, 2, figsize=(13, 5))
    scene(a, 0.5, 6.5, -0.3, 4.8, "Held flat in the air")
    edge = np.array([2.6, 2.6])
    a.add_patch(Polygon([edge + [0, -0.08], edge + [1.8, -0.08], edge + [1.8, 0.08], edge + [0, 0.08]],
                        fc=TOP, ec="#b8491c", zorder=4))
    gripper(a, edge, unit(0))
    arrow(a, (3.5, 2.5), (3.5, 1.3), color=INK, lw=2.2)
    note(a, (3.6, 1.4), "weight, W")
    arrow(a, (2.0, 1.4), (2.0, 2.1), color=BAD, lw=2.2)
    note(a, (0.6, 1.1), "the fingers carry all of W\nand must stop the twist", color=BAD)

    scene(b, 3.2, 9.2, -0.3, 4.8, "Resting on the far legs")
    leg(b, PIVOT[0])
    edge, u = board_about(b, PIVOT, 150)
    gripper(b, edge, u)
    centre = PIVOT + unit(150) * BOARD_W / 2
    arrow(b, centre, centre + np.array([0, -1.1]), color=INK, lw=2.2)
    note(b, (centre[0] + 0.1, centre[1] - 1.0), "W")
    arrow(b, PIVOT + np.array([0.35, -0.8]), PIVOT + np.array([0.35, -0.05]), color=GOOD, lw=2.2)
    note(b, (PIVOT[0] + 0.5, PIVOT[1] - 0.5), "legs: W/2", color=GOOD)
    arrow(b, edge + np.array([-0.2, -1.1]), edge + np.array([-0.2, -0.35]), color=GOOD, lw=2.2)
    note(b, (3.3, edge[1] - 1.0), "fingers: W/2,\nno twist", color=GOOD)
    save(fig, "load_sharing.png")


# ---------------------------------------------------------------- which point to turn about


def tipped_leg(ax, x, h=1.4, angle=22):
    """A leg falling over, about its bottom edge on the far side."""
    t = 0.3
    base = np.array([x + t / 2, 0.0])
    c, s = np.cos(np.radians(-angle)), np.sin(np.radians(-angle))
    turn = np.array([[c, -s], [s, c]])
    pts = [np.array([-t, 0]), np.array([0, 0]), np.array([0, h]), np.array([-t, h])]
    ax.add_patch(Polygon([base + turn @ p for p in pts], fc=LEG, ec=BAD, lw=2, zorder=2))


def pivot_choice():
    fig, (a, b) = plt.subplots(1, 2, figsize=(13, 5.2))
    scene(a, 3.2, 8.6, -0.3, 4.4, "✓  Turn about the corner on the legs")
    a.title.set_color(GOOD)
    legs(a)
    for deg, alpha in ((110, 0.25), (140, 0.5), (170, 1.0)):
        board_about(a, PIVOT, deg, alpha=alpha)
    a.add_patch(Circle(PIVOT, 0.09, fc=PATH, zorder=9))
    note(a, (3.3, 3.9), "The corner stays where it is. Nothing slides.\nThe legs only feel a push straight down.")

    scene(b, 3.2, 8.6, -0.3, 4.4, "✗  Turn about the gripped edge")
    b.title.set_color(BAD)
    leg(b, NEAR_LEG)
    tipped_leg(b, PIVOT[0])
    for x, deg, alpha in ((6.8, 110, 0.25), (7.1, 125, 0.5), (7.4, 140, 1.0)):
        board_about(b, np.array([x, 1.55]), deg, alpha=alpha)
    arrow(b, (6.8, 1.85), (7.9, 1.85), color=BAD, lw=2)
    note(b, (3.3, 3.9), "Turned about the gripped edge, or with the arm a few\nmillimetres off, the corner "
         "slides across the leg tops,\nand friction drags the legs over.", color=BAD)
    save(fig, "pivot_choice.png")


# ---------------------------------------------------------------- when a leg falls over


def leg_tipping():
    fig, (a, b) = plt.subplots(1, 2, figsize=(13, 5), gridspec_kw={"width_ratios": [1, 1.5]})
    scene(a, 0, 5, -0.3, 4.6, "What keeps a leg standing")
    a.add_patch(Rectangle((2.0, 0), 0.8, 3.0, fc=LEG, ec="#1c5cab", zorder=2))
    arrow(a, (2.4, 4.1), (2.4, 3.05), color=INK, lw=2)
    note(a, (2.55, 4.05), "N: the board's weight on it")
    arrow(a, (1.0, 2.9), (1.95, 2.9), color=BAD, lw=2.2)
    note(a, (0.05, 3.25), "F: a push\nfrom the side", color=BAD)
    arrow(a, (2.4, 1.5), (2.4, 0.6), color=INK2, lw=2)
    note(a, (0.5, 1.0), "leg's own\nweight")
    arrow(a, (1.5, 1.0), (2.3, 1.0), color=INK2, lw=0.8, style="-")
    a.add_patch(Circle((2.8, 0), 0.09, fc=BAD, zorder=9))
    note(a, (3.35, 0.5), "it tips over\nabout this edge", color=BAD)
    a.annotate("", xy=(3.1, 3.0), xytext=(3.1, 0.05), arrowprops=dict(arrowstyle="<->", color=INK2, lw=1))
    note(a, (3.2, 1.9), "h")
    a.text(0.1, -0.25, "Tips when  F × h  >  (leg weight + N) × half its thickness", fontsize=9.5, color=INK,
           va="top")

    names = ["Pine, 2.5 cm", "Pine, 3 cm (like v2)", "Oak, 3.5 cm", "Steel, 3 cm"]
    sizes = [(0.025, 500), (0.03, 500), (0.035, 700), (0.03, 7800)]
    n = 0.48 * 9.81 / 4
    h = 0.14
    push = [(t * t * h * rho * 9.81 + n) * (t / 2) / h for t, rho in sizes]
    y = np.arange(len(names))[::-1]
    b.barh(y, push, height=0.45, color="#2a78d6", zorder=3)
    for yi, value in zip(y, push, strict=True):
        b.text(value + 0.03, yi, f"{value:.2f} N", va="center", fontsize=9.5, color=INK2)
    slide = 1.2 * n
    b.axvline(slide, color=BAD, lw=1.5)
    b.text(slide - 0.03, 3.45, f"push from the board\nsliding on it: {slide:.1f} N", ha="right", va="center",
           fontsize=9.5, color=BAD)
    b.set_yticks(y, names)
    b.set_xlim(0, 1.7)
    b.set_ylim(-0.6, 3.9)
    b.set_xlabel("side push at the top that knocks it over (N), with a 0.48 kg top resting on two legs",
                 color=INK2, fontsize=9.5)
    b.grid(axis="x", color="#e1e0d9", lw=1)
    b.set_axisbelow(True)
    for side in ("top", "right"):
        b.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        b.spines[side].set_color("#c3c2b7")
    b.tick_params(colors=INK2)
    b.set_facecolor(BG)
    b.set_title("A sliding board knocks over any wooden leg", loc="left", fontsize=12, fontweight="bold")
    fig.tight_layout()
    save(fig, "leg_tipping.png")


if __name__ == "__main__":
    steps()
    load_sharing()
    pivot_choice()
    leg_tipping()
