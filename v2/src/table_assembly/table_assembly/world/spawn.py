"""Drawing this run's room: the wall, the table top leaning on it, and the legs.

This is the simulator's side. It knows exactly where everything is, because it
is the one putting it there, and the robot never gets to ask it.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ..transforms import WORLD_Z, rotation_z, rpy_from_matrix
from . import spec

PART_TEMPLATE = Path(__file__).parent / "part.sdf"
WALL_TEMPLATE = Path(__file__).parent / "wall.sdf"


@dataclass(frozen=True)
class SpawnedBox:
    """One box, as the simulator will be told to create it."""

    name: str
    size: tuple[float, float, float]
    centre: np.ndarray
    rotation: np.ndarray
    colour: tuple[float, float, float]
    density: float  # zero for something bolted down


@dataclass(frozen=True)
class Room:
    wall: SpawnedBox
    top: SpawnedBox
    legs: tuple[SpawnedBox, ...]
    lean: float  # how far the top leans back from upright, in radians

    def boxes(self) -> list[SpawnedBox]:
        return [self.wall, self.top, *self.legs]


def random_room(seed: int) -> Room:
    rng = random.Random(seed)
    colours = rng.sample(spec.PALETTE, 2)

    azimuth = math.radians(rng.uniform(*spec.WALL_AZIMUTH_DEG))
    outward = np.array([math.cos(azimuth), math.sin(azimuth), 0.0])
    along = np.array([-outward[1], outward[0], 0.0])
    distance = rng.uniform(*spec.WALL_DISTANCE)

    # A top and wall that leave the top's upper edge standing clear of the
    # wall, so the fingers have room either side of it. Redrawn until they do.
    while True:
        size = (rng.uniform(*spec.TOP_LENGTH), rng.uniform(*spec.TOP_WIDTH), rng.uniform(*spec.TOP_THICKNESS))
        wall_height = rng.uniform(*spec.WALL_HEIGHT)
        lean = math.radians(rng.uniform(*spec.TOP_LEAN_DEG))
        if size[1] - wall_height / math.cos(lean) >= spec.TOP_FREE_EDGE:
            break

    front = outward * distance
    wall = SpawnedBox(
        name="wall",
        size=(spec.WALL_THICKNESS, spec.WALL_LENGTH, wall_height),
        centre=front + outward * spec.WALL_THICKNESS / 2.0 + WORLD_Z * wall_height / 2.0,
        rotation=rotation_z(azimuth),
        colour=spec.WALL_COLOUR,
        density=0.0,
    )
    top = _leaning_top(
        size,
        lean,
        wall_height,
        front + along * rng.uniform(-spec.TOP_SLIDE, spec.TOP_SLIDE),
        outward,
        colours[0],
    )
    legs = _lying_legs(rng, colours[1])
    return Room(wall=wall, top=top, legs=legs, lean=lean)


def _leaning_top(size, lean, wall_height, foot, outward, colour) -> SpawnedBox:
    """The top standing on its long edge, leaning back onto the wall's top corner.

    Worked out in the vertical plane through the arm and the wall. The board's
    back face touches the wall's top front corner, and the bottom corner of
    that back face rests on the floor, so the back face is the line through
    those two points at the lean angle.
    """
    length, width, thickness = size
    along = np.array([-outward[1], outward[0], 0.0])
    up = outward * math.sin(lean) + WORLD_Z * math.cos(lean)  # up the face
    back = outward * math.cos(lean) - WORLD_Z * math.sin(lean)  # out of the back face, towards the wall
    heel = foot - outward * wall_height * math.tan(lean)  # bottom corner of the back face
    centre = heel - back * thickness / 2.0 + up * width / 2.0
    return SpawnedBox(
        name="table_top",
        size=size,
        # A millimetre of air, so the board settles onto the floor rather than
        # starting inside it.
        centre=centre + WORLD_Z * 0.001,
        rotation=np.column_stack((along, up, back)),
        colour=colour,
        density=spec.TOP_DENSITY,
    )


def _lying_legs(rng: random.Random, colour) -> tuple[SpawnedBox, ...]:
    length = rng.uniform(*spec.LEG_LENGTH)
    thickness = rng.uniform(*spec.LEG_THICKNESS)
    placed: list[SpawnedBox] = []
    for index in range(spec.LEG_COUNT):
        for _ in range(2000):
            azimuth = math.radians(rng.uniform(*spec.LEG_AZIMUTH_DEG))
            distance = rng.uniform(*spec.LEG_DISTANCE)
            leg = SpawnedBox(
                name=f"leg_{index}",
                size=(length, thickness, thickness),
                centre=np.array(
                    [distance * math.cos(azimuth), distance * math.sin(azimuth), thickness / 2.0 + 0.001]
                ),
                rotation=rotation_z(rng.uniform(-math.pi / 2, math.pi / 2)),
                colour=colour,
                density=spec.LEG_DENSITY,
            )
            if all(_gap(leg, other) >= spec.LEG_GAP for other in placed):
                placed.append(leg)
                break
        else:
            raise RuntimeError(f"could not lay out {spec.LEG_COUNT} legs without them touching")
    return tuple(placed)


def _gap(a: SpawnedBox, b: SpawnedBox) -> float:
    """Clear floor between two lying legs, near enough.

    Each leg is treated as its centre line, sampled finely, and the gap is the
    closest two samples minus the legs' half thicknesses.
    """
    ts = np.linspace(-0.5, 0.5, 25)[:, None]
    line_a = a.centre[:2] + ts * a.size[0] * a.rotation[:2, 0]
    line_b = b.centre[:2] + ts * b.size[0] * b.rotation[:2, 0]
    closest = float(np.min(np.linalg.norm(line_a[:, None, :] - line_b[None, :, :], axis=2)))
    return closest - (a.size[1] + b.size[1]) / 2.0


def box_sdf(box: SpawnedBox) -> str:
    """One box as the simulator's own model format."""
    length, width, height = box.size
    roll, pitch, yaw = rpy_from_matrix(box.rotation)
    red, green, blue = box.colour
    x, y, z = box.centre
    fields = dict(
        name=box.name,
        x=x,
        y=y,
        z=z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        length=length,
        width=width,
        height=height,
        red=red,
        green=green,
        blue=blue,
    )
    if box.density == 0.0:
        template = WALL_TEMPLATE
    else:
        template = PART_TEMPLATE
        mass = box.density * length * width * height
        # A solid box, about its own centre.
        fields.update(
            mass=mass,
            ixx=mass * (width**2 + height**2) / 12.0,
            iyy=mass * (length**2 + height**2) / 12.0,
            izz=mass * (length**2 + width**2) / 12.0,
        )
    return template.read_text().split("-->\n", 1)[1].format(**fields)
