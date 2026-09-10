"""Putting the world file together.

The room, the lighting and the floor never change, and they live in
``cell.sdf``. The wall, the table top and the legs change every run. Rather
than edit a world file by hand before each run, the world is assembled here:
the fixed room is read from disk and this run's boxes are dropped into the
marker line ``cell.sdf`` leaves for them. What comes out is a complete world
file, which is what Gazebo is started on.
"""

from __future__ import annotations

from pathlib import Path

from .spawn import Room, box_sdf

PARTS_MARKER = "<!-- PARTS -->"


def build_world(world_template: str, room: Room) -> str:
    """The finished world: the fixed room, with this run's boxes in it."""
    if PARTS_MARKER not in world_template:
        raise ValueError(f"the world template has no {PARTS_MARKER} line to fill in")
    return world_template.replace(PARTS_MARKER, "".join(box_sdf(box) for box in room.boxes()))


def read_template(share: Path) -> str:
    """The fixed part of the world, as it sits on disk."""
    return (share / "world" / "cell.sdf").read_text()
