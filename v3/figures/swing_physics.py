"""Draws the pictures in SWING_PHYSICS.md.

Run from v3/figures/:  python swing_physics.py

Sketches, not to scale, except the chart, which uses v2's numbers.
"""

from __future__ import annotations

import math

import numpy as np
from matplotlib.patches import Circle, FancyBboxPatch
from top_from_wall import (
    BAD,
    BG,
    BOARD_T,
    BOARD_W,
    GOOD,
    INK,
    INK2,
    PATH,
    arrow,
    board,
    gripper,
    note,
    perp,
    plt,
    save,
    scene,
    unit,
)

# The UR5e, flattened into one plane, in the sketches' units (1 = 10 cm).
SHOULDER = np.array([0.0, 1.6])
UPPER_ARM = 4.25
FOREARM = 3.92
WRIST_OFFSET = 1.0  # wrist 1 to wrist 2
FLANGE = 1.0  # wrist 2 to the tool
ARM = "#c9ccd1"
ARM_EDGE = "#8a9098"
JOINT = "#4f6d8f"


def elbow(wrist1):
    """Where the elbow is for the wrist at ``wrist1``, elbow up."""
    d = wrist1 - SHOULDER
    reach = float(np.linalg.norm(d))
    bend = math.acos((UPPER_ARM**2 + reach**2 - FOREARM**2) / (2 * UPPER_ARM * reach))
    angle = math.atan2(d[1], d[0]) + bend
    return SHOULDER + UPPER_ARM * np.array([math.cos(angle), math.sin(angle)])


def arm(ax, edge, u, alpha=1.0, z=3, wrist1=None):
    """Draw the arm holding a gripper whose fingers are on ``edge``, reaching along ``u``."""
    tool = edge - u * 1.2
    v = -perp(u)
    wrist2 = tool - u * FLANGE
    if wrist1 is None:
        wrist1 = wrist2 - v * WRIST_OFFSET
    e = elbow(wrist1)
    ax.add_patch(plt.Rectangle((-0.8, 0), 1.6, 0.9, fc=ARM_EDGE, alpha=alpha, zorder=z))
    points = [np.array([0, 0.9]), SHOULDER, e, wrist1, wrist2, tool]
    widths = [9, 11, 9, 8, 8]
    for (a, b), w in zip(zip(points, points[1:], strict=False), widths, strict=True):
        ax.plot([a[0], b[0]], [a[1], b[1]], color=ARM_EDGE, lw=w + 2.5, solid_capstyle="round", alpha=alpha, zorder=z)
    for (a, b), w in zip(zip(points, points[1:], strict=False), widths, strict=True):
        ax.plot([a[0], b[0]], [a[1], b[1]], color=ARM, lw=w, solid_capstyle="round", alpha=alpha, zorder=z)
    for p, r in ((SHOULDER, 0.32), (e, 0.28), (wrist1, 0.24), (wrist2, 0.22)):
        ax.add_patch(Circle(p, r, fc=JOINT, alpha=alpha, zorder=z + 0.1))
    return wrist1, e


# ---------------------------------------------------------------- who moves


def who_moves():
    fig, (a, b) = plt.subplots(1, 2, figsize=(14, 6.4))
    pivot = np.array([5.0, 4.0])

    scene(a, -1.2, 10.8, -0.4, 9.0, "Way 1: the whole arm moves, the gripped edge stays put")
    for deg, alpha in ((0, 0.25), (45, 0.45), (90, 1.0)):
        u = unit(-90 + deg)
        arm(a, pivot, u, alpha=alpha)
        board(a, pivot, u, alpha=alpha)
        gripper(a, pivot, u, alpha=alpha, arm=False)
    a.add_patch(Circle(pivot, 0.12, fc=PATH, zorder=9))
    note(a, (7.2, 3.0), "gripped edge:\nstays here")
    arrow(a, (7.1, 3.1), (5.2, 3.9), color=INK2, lw=1, rad=0.2)
    note(a, (-1.0, 8.4), "Shoulder, elbow and wrist all move a little.\n"
         "The board turns about its own edge, so it only\nneeds room as big as the board (about 20 cm).")

    scene(b, -1.2, 10.8, -0.4, 9.0, "Way 2: only the wrist turns, the rest of the arm stays still")
    u0 = unit(-90)
    wrist1 = pivot - u0 * 1.2 - u0 * FLANGE + perp(u0) * WRIST_OFFSET
    arm(b, pivot, u0, wrist1=wrist1)
    far = []
    for deg, alpha in ((0, 0.25), (30, 0.4), (60, 0.6), (90, 1.0)):
        u = unit(-90 + deg)
        v = -perp(u)
        tool = wrist1 + v * WRIST_OFFSET + u * FLANGE
        edge = tool + u * 1.2
        b.plot([wrist1[0], tool[0] - u[0] * FLANGE, tool[0]], [wrist1[1], tool[1] - u[1] * FLANGE, tool[1]],
               color="#8a9098", lw=8, alpha=alpha, solid_capstyle="round", zorder=3)
        board(b, edge, u, alpha=alpha)
        gripper(b, edge, u, alpha=alpha, arm=False)
        far.append(edge + u * BOARD_W)
    radius = float(np.linalg.norm(far[0] - wrist1))
    b.add_patch(plt.matplotlib.patches.Arc(wrist1, 2 * radius, 2 * radius, theta1=-110, theta2=5,
                                           color=PATH, ls="--", lw=1.5, zorder=2))
    b.add_patch(Circle(wrist1, 0.14, fc=PATH, zorder=9))
    note(b, (wrist1[0] + 0.3, wrist1[1] + 0.45), "wrist 1: the only\njoint that turns")
    note(b, (-1.0, 8.4), "One joint, one simple move, easy to keep slow.\n"
         "But the board turns about the wrist, so it swings\nround a circle about 40 cm across.")
    save(fig, "who_moves.png")


# ---------------------------------------------------------------- forces on the grip


def forces():
    fig, (a, b) = plt.subplots(1, 2, figsize=(12, 5))
    scene(a, 0, 5, -0.3, 5.2, "Hanging: friction carries it (easy)")
    edge = np.array([2.5, 4.0])
    board(a, edge, unit(-90))
    gripper(a, edge, unit(-90))
    centre = edge + unit(-90) * BOARD_W / 2
    arrow(a, centre, centre + np.array([0, -1.2]), color=INK, lw=2.2)
    note(a, (2.65, centre[1] - 1.0), "weight")
    for side in (1, -1):
        x = edge[0] + side * 0.28
        arrow(a, (x, 3.2), (x, 3.9), color=GOOD, lw=2)
    note(a, (0.2, 1.2), "Its weight pulls along the fingers.\nFriction at the pads holds it.\n"
         "Good for up to about 6 kg.", color=INK2)

    scene(b, 0.2, 6.4, -0.3, 5.2, "Flat: the fingers must stop it twisting (hard)")
    edge = np.array([2.2, 3.0])
    board(b, edge, unit(0))
    gripper(b, edge, unit(0))
    centre = edge + unit(0) * BOARD_W / 2
    arrow(b, centre + np.array([0, -0.08]), centre + np.array([0, -1.3]), color=INK, lw=2.2)
    note(b, (centre[0] + 0.12, centre[1] - 1.1), "weight")
    b.annotate("", xy=(centre[0], 1.95), xytext=(edge[0] + 0.27, 1.95),
               arrowprops=dict(arrowstyle="<->", color=INK2, lw=1.2))
    note(b, ((centre[0] + edge[0]) / 2 + 0.1, 1.72), "lever", ha="center")
    b.plot([edge[0] + 0.27] * 2, [1.85, 2.9], color=INK2, lw=0.8, ls=":")
    arrow(b, (edge[0] + 0.15, 3.7), (edge[0] + 0.15, 3.0 + BOARD_T / 2 + 0.05), color=BAD, lw=2)
    arrow(b, (edge[0] + 0.39, 2.25), (edge[0] + 0.39, 3.0 - BOARD_T / 2 - 0.05), color=BAD, lw=2)
    note(b, (3.3, 4.4), "The weight, times the lever, tries to\ntwist the board down out of the fingers.\n"
         "The two rows of pads push back (red).\nLike pinching a book by one edge.", color=INK2)
    save(fig, "forces.png")


# ---------------------------------------------------------------- what fails first


def what_fails_first():
    fig, ax = plt.subplots(figsize=(9, 4.8))
    mass = np.linspace(0, 4, 400)
    g = 9.81
    lines = [
        ("Fingers stop it twisting (held flat)", mass * g * 0.073 / (25 * 0.024), "#2a78d6"),
        ("Arm's rated payload, 5 kg with the gripper", (mass + 0.9) / 5.0, "#eb6834"),
        ("Wrist 1 torque, 28 N·m", (mass * 0.32 + 0.135) * g / 28.0, "#1baf7a"),
        ("Friction holds it up (hanging)", mass * g / (2 * 1.2 * 25), "#eda100"),
    ]
    for label, ratio, color in lines:
        ax.plot(mass, ratio, color=color, lw=2.2, label=label)
    ax.axhline(1.0, color=INK2, lw=1)
    ax.text(3.98, 1.04, "limit", ha="right", fontsize=9.5, color=INK2)
    ax.axvspan(0.25, 0.48, color="#f0efec", zorder=0)
    ax.text(0.365, 2.25, "v2's\ntop", ha="center", fontsize=9.5, color=INK2)
    cross = 25 * 0.024 / (g * 0.073)
    ax.plot([cross], [1.0], "o", color="#2a78d6", ms=8, mec=BG, mew=2, zorder=5)
    ax.annotate(f"slips at about {cross:.1f} kg", xy=(cross, 1.0), xytext=(1.25, 1.6), fontsize=9.5, color=INK2,
                arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
    ax.set_xlim(0, 4)
    ax.set_ylim(0, 2.5)
    ax.set_xlabel("weight of the table top (kg), 20 cm wide", color=INK2)
    ax.set_ylabel("how much of the limit is used", color=INK2)
    ax.set_yticks([0, 0.5, 1, 1.5, 2, 2.5], ["0", "50%", "100%", "150%", "200%", "250%"])
    ax.grid(axis="y", color="#e1e0d9", lw=1)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color("#c3c2b7")
    ax.tick_params(colors=INK2)
    ax.set_facecolor(BG)
    ax.legend(frameon=False, loc="upper right", fontsize=9.5)
    ax.set_title("As the top gets heavier, the grip on a flat board gives out first", loc="left",
                 fontsize=12.5, fontweight="bold")
    save(fig, "what_fails_first.png")


# ---------------------------------------------------------------- where the numbers live


def box(ax, x, y, w, h, title, body, color):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.12",
                                fc="white", ec=color, lw=1.8, zorder=2))
    ax.text(x + 0.15, y + h - 0.2, title, fontsize=10.5, fontweight="bold", va="top", zorder=3)
    ax.text(x + 0.15, y + h - 0.62, body, fontsize=9, color=INK2, va="top", zorder=3, linespacing=1.35)


def gazebo_chain():
    fig, ax = plt.subplots(figsize=(13, 6.2))
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 6.6)
    ax.axis("off")
    ax.add_patch(FancyBboxPatch((0.05, 3.35), 12.9, 3.15, boxstyle="round,pad=0,rounding_size=0.2",
                                fc="#f6f5f1", ec="none", zorder=0))
    ax.add_patch(FancyBboxPatch((0.05, 0.1), 12.9, 3.1, boxstyle="round,pad=0,rounding_size=0.2",
                                fc="#f1f4fa", ec="none", zorder=0))
    ax.text(0.25, 6.3, "The simulator's side (world/). Decides the truth.", fontsize=11, fontweight="bold", color=INK2)
    ax.text(0.25, 0.3, "The robot's side. Never reads the numbers above.", fontsize=11, fontweight="bold", color=INK2)

    orange, grey, blue = "#eb6834", "#8a9098", "#2a78d6"
    box(ax, 0.3, 3.7, 2.8, 2.3, "spec.py", "TOP_DENSITY = 400 kg/m³\nTOP_LENGTH, WIDTH,\nTHICKNESS ranges\n"
        "LEG_DENSITY = 500", orange)
    box(ax, 3.5, 3.7, 2.8, 2.3, "spawn.py", "mass = density × L × W × T\ninertia from mass\nand size", orange)
    box(ax, 6.7, 3.7, 2.8, 2.3, "part.sdf", "<mass>, <inertia>\nfriction mu = 1.2\ncontact stiffness kp", orange)
    box(ax, 9.9, 3.7, 2.8, 2.3, "Gazebo physics", "gravity pulls on the mass\none step every 4 ms\n"
        "contacts + friction\n(cell.sdf)", grey)
    box(ax, 0.3, 0.7, 2.8, 2.3, "gripper.urdf.xacro", "finger push: 25 N max\npad friction mu = 1.6\n"
        "4 pads per finger", blue)
    box(ax, 3.5, 0.7, 2.8, 2.3, "UR5e limits", "big joints: 150 N·m\nwrist joints: 28 N·m\n"
        "(ur_description,\njoint_limits.yaml)", blue)
    box(ax, 9.9, 0.7, 2.8, 2.3, "What the robot senses", "joint angles + efforts\nfingertip contacts\ncamera", blue)
    box(ax, 6.7, 0.7, 2.8, 2.3, "Robot's code", "measures the top's size\ncan measure its weight\n"
        "from the joint efforts", blue)
    for x in (3.1, 6.3, 9.5):
        arrow(ax, (x + 0.02, 4.85), (x + 0.38, 4.85), color=INK2, lw=1.5)
    for x in (1.7, 4.9):
        ax.plot([x, x], [3.02, 3.28], color=INK2, lw=1.5)
    ax.plot([1.7, 10.6], [3.28, 3.28], color=INK2, lw=1.5)
    arrow(ax, (10.6, 3.27), (10.6, 3.68), color=INK2, lw=1.5)
    arrow(ax, (11.8, 3.68), (11.8, 3.02), color=INK2, lw=1.5)
    arrow(ax, (9.88, 1.85), (9.52, 1.85), color=INK2, lw=1.5)
    ax.set_title("Where the weight comes from, and how it reaches the robot", loc="left", fontsize=13,
                 fontweight="bold")
    save(fig, "gazebo_chain.png")


if __name__ == "__main__":
    who_moves()
    forces()
    what_fails_first()
    gazebo_chain()
