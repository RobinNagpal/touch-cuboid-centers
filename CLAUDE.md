# CLAUDE.md

## What this repo is

One task, done properly: a UR5e in Gazebo measures the cuboids on a table and
touches the middle of the biggest face of each one. Read
[`README.md`](README.md) first, then [`ARCHITECTURE.md`](ARCHITECTURE.md) for
the file layout and [`IMPLEMENTATION_NOTES.md`](IMPLEMENTATION_NOTES.md) for
why it is built the way it is.

## Before changing anything

`make test` runs the tests that need no simulator. They cover the geometry and
the box fitting, which is where most of the logic is.

`make run GUI=false` runs the whole thing end to end. It takes about four
minutes. Nothing is really verified until this has passed.

Kill leftovers between runs. A simulator that was killed rather than shut down
leaves its ROS endpoints registered and the next run will stall on them.

## Where things belong

- Numbers describing the cell go in `cuboid_cell/cell.py`, nowhere else. The
  world file, the spawner and the task all read them from there.
- `geometry.py` and `perception.py` must not import ROS. That is what lets the
  tests run them directly, and it is worth keeping.
- `task.py` is the only module that knows what order things happen in. Keep
  sequencing there and capability in the modules it calls.

## Writing style

Simple English, short sentences, no filler. Comment the *why*, not the *what* —
a comment that restates the line below it is noise. The conceptual background
belongs in the three docs, not in the code.

Same for docs: plain English, pinpoint details, no marketing.
