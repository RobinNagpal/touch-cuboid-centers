"""Drawing this run's room: the table top lying on its stands, and the legs standing.

This is the simulator's side. It knows exactly where everything is, because it
is the one putting it there, and the robot never gets to ask it.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ..transforms import rotation_z, rpy_from_matrix
from . import spec

PART_TEMPLATE = Path(__file__).parent / "part.sdf"
STAND_TEMPLATE = Path(__file__).parent / "stand.sdf"


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
    stands: tuple[SpawnedBox, ...]
    top: SpawnedBox
    legs: tuple[SpawnedBox, ...]

    def boxes(self) -> list[SpawnedBox]:
        return [*self.stands, self.top, *self.legs]


def random_room(seed: int) -> Room:
    rng = random.Random(seed)
    colours = rng.sample(spec.PALETTE, 2)

    azimuth = math.radians(rng.uniform(*spec.TOP_AZIMUTH_DEG))
    distance = rng.uniform(*spec.TOP_DISTANCE)
    size = (rng.uniform(*spec.TOP_LENGTH), rng.uniform(*spec.TOP_WIDTH), rng.uniform(*spec.TOP_THICKNESS))
    stand_height = rng.uniform(*spec.STAND_HEIGHT)
    # The long side runs across the line from the arm, turned a little.
    rotation = rotation_z(azimuth + math.pi / 2.0 + math.radians(rng.uniform(*spec.TOP_TURN_DEG)))
    centre = np.array([distance * math.cos(azimuth), distance * math.sin(azimuth), 0.0])

    stands = tuple(
        SpawnedBox(
            name=f"stand_{index}",
            size=(spec.STAND_WIDTH, size[1] - 2.0 * spec.STAND_INSET, stand_height),
            centre=centre
            + rotation[:, 0] * end * (size[0] / 2.0 - spec.STAND_INSET - spec.STAND_WIDTH / 2.0)
            + np.array([0.0, 0.0, stand_height / 2.0]),
            rotation=rotation,
            colour=spec.STAND_COLOUR,
            density=0.0,
        )
        for index, end in enumerate((-1.0, 1.0))
    )
    top = SpawnedBox(
        name="table_top",
        size=size,
        # A millimetre of air, so the board settles onto the stands rather
        # than starting inside them.
        centre=centre + np.array([0.0, 0.0, stand_height + size[2] / 2.0 + 0.001]),
        rotation=rotation,
        colour=colours[0],
        density=spec.TOP_DENSITY,
    )
    legs = _standing_legs(rng, colours[1])
    return Room(stands=stands, top=top, legs=legs)


def _standing_legs(rng: random.Random, colour) -> tuple[SpawnedBox, ...]:
    length = rng.uniform(*spec.LEG_LENGTH)
    thickness = rng.uniform(*spec.LEG_THICKNESS)
    placed: list[SpawnedBox] = []
    for index in range(spec.LEG_COUNT):
        for _ in range(2000):
            azimuth = math.radians(rng.uniform(*spec.LEG_AZIMUTH_DEG))
            distance = rng.uniform(*spec.LEG_DISTANCE)
            leg = SpawnedBox(
                name=f"leg_{index}",
                size=(thickness, thickness, length),
                # A millimetre of air, so the leg settles onto the floor.
                centre=np.array(
                    [distance * math.cos(azimuth), distance * math.sin(azimuth), length / 2.0 + 0.001]
                ),
                rotation=rotation_z(rng.uniform(-math.pi / 4, math.pi / 4)),
                colour=colour,
                density=spec.LEG_DENSITY,
            )
            if all(np.linalg.norm(leg.centre[:2] - other.centre[:2]) >= spec.LEG_SPACING for other in placed):
                placed.append(leg)
                break
        else:
            raise RuntimeError(f"could not stand {spec.LEG_COUNT} legs far enough apart")
    return tuple(placed)


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
        template = STAND_TEMPLATE
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
