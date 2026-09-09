"""Putting the world file together.

The room, the lighting and the ground never change, and they live in
``cell.sdf``. The table never changes either, and it lives in
``table/table.sdf``. The cuboids change every run.

Rather than have three files that must agree, or one file that has to be
edited by hand before each run, the world is assembled here: the fixed parts
are read from disk and the cuboids are dropped into the two marker lines
``cell.sdf`` leaves for them. What comes out is a complete world file written
to a temporary path, which is what Gazebo is started on.
"""

from __future__ import annotations

from pathlib import Path

from ..cuboids.spawn import SpawnedCuboid, cuboid_sdf

TABLE_MARKER = "<!-- TABLE -->"
CUBOIDS_MARKER = "<!-- CUBOIDS -->"


def build_world(world_template: str, table: str, cuboids: list[SpawnedCuboid]) -> str:
    """The finished world: the room, with the table and the cuboids in it."""
    for marker in (TABLE_MARKER, CUBOIDS_MARKER):
        if marker not in world_template:
            raise ValueError(f"the world template has no {marker} line to fill in")

    filled = world_template.replace(TABLE_MARKER, table)
    return filled.replace(CUBOIDS_MARKER, "".join(cuboid_sdf(c) for c in cuboids))


def read_parts(share: Path) -> tuple[str, str]:
    """The two fixed pieces of the world, as they sit on disk."""
    return (share / "world" / "cell.sdf").read_text(), (share / "table" / "table.sdf").read_text()
