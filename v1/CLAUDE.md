# CLAUDE.md

## What this repo is

One task, done properly: a UR5e in Gazebo measures the cuboids on a table and
touches the middle of the biggest face of each one. Read
[`README.md`](README.md) first, then [`PSEUDOCODE.md`](PSEUDOCODE.md) for what
every file and function is for, [`ARCHITECTURE.md`](ARCHITECTURE.md) for the
layout, and [`IMPLEMENTATION_NOTES.md`](IMPLEMENTATION_NOTES.md) for why it is
built the way it is.

## Before changing anything

`make test` runs the tests that need no simulator. They cover the geometry and
the box fitting, which is where most of the logic is.

`make run GUI=false` runs the whole thing end to end. It takes about three
minutes. Nothing is really verified until this has passed.

Kill leftovers between runs. A simulator that was killed rather than shut down
leaves its ROS endpoints registered and the next run will stall on them.

## Where things belong

One folder per thing in the room. Everything about the arm is in `arm/`, and
the same for `table/`, `cuboids/` and `world/` — the model the simulator loads,
the settings, and the code, together. A new file goes with its subject, not
with other files of its type.

- Numbers live with their subject, and nowhere else: the table and the two
  zones in `table/layout.py`, the arm's reach and working heights in
  `arm/dimensions.py`, the cuboid size range in `cuboids/spec.py`.
- `cuboids/geometry.py` and `cuboids/perception.py` must not import ROS. That
  is what lets the tests run them directly, and it is worth keeping.
- `task.py` is the only module that knows what order things happen in. Keep
  sequencing there and capability in the modules it calls.

## Writing style

Simple English, short sentences, no filler. Comment the *why*, not the *what* —
a comment that restates the line below it is noise. The conceptual background
belongs in the docs, not in the code.

Same for docs: plain English, pinpoint details, no marketing.
