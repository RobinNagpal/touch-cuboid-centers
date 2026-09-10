# CLAUDE.md

## What this repo is

One task, done properly: a UR5e in Gazebo finds a table top leaning against a
wall and four legs lying on the floor, measures them, stands the legs up to
fit the top, and puts the top on them. Read [`README.md`](README.md) first,
then [`PSEUDOCODE.md`](PSEUDOCODE.md) for what every file and function is for,
[`ARCHITECTURE.md`](ARCHITECTURE.md) for the layout, and
[`IMPLEMENTATION_NOTES.md`](IMPLEMENTATION_NOTES.md) for why it is built the
way it is.

This folder is a project on its own. It shares nothing with `../v1` and must
not import from it or point at it.

## Before changing anything

`make test` runs the tests that need no simulator. They cover the fitting, the
reading of a whole room, the planning and the grasp geometry, which is where
most of the logic is.

`make run GUI=false` runs the whole thing end to end. It takes about ten
minutes. Nothing is really verified until this has passed.

Kill leftovers between runs. A simulator that was killed rather than shut down
leaves its ROS endpoints registered and the next run will stall on them.

## The one rule that matters

The robot is told nothing about the room. It knows where its own base is and
how its own tooling is built (`arm/dimensions.py`), and everything else — the
floor height, the wall, the top's size and pose, the legs — it measures.

- `world/` is the simulator's side. It decides the sizes and positions of
  everything, and **nothing outside `world/` may import from it.**
  `test_world.py` fails if anything does.
- A new number the robot needs is either a fact about the robot (it goes in
  `arm/dimensions.py`) or a choice about how to do the job (it goes next to
  the code that uses it, with the reason). It is never a fact about the room.

## Where things belong

- `perception/` and `assembly/` must not import ROS. That is what lets the
  tests run them directly, and it is worth keeping.
- `task.py` is the only module that knows what order things happen in. Keep
  sequencing there and capability in the modules it calls.

## Writing style

Simple English, short sentences, no filler. Comment the *why*, not the *what* —
a comment that restates the line below it is noise. The conceptual background
belongs in the docs, not in the code.

Same for docs: plain English, pinpoint details, no marketing.
