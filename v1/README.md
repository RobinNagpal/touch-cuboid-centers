# touch-cuboid-centers

A robot arm looks at cuboids on a table, works out which face of each one is
the biggest, and touches the middle of that face with its fingertip.

Everything runs in simulation. One command starts it.

```
make run
```

The first run downloads the environment, which is a few gigabytes.

## The problem

Cuboids of different sizes sit on a table. For each of them the arm has to:

- pick it up and set it down on the other half of the table;
- measure its length, width and height using a camera;
- work out the area of its three different faces — length x width, length x
  height, width x height — and find the biggest one;
- reach out and touch the centre of that face.

The first step is the interesting one, and it is worth saying why it is there.
The arm has no memory of which box is which. Every time it looks at the table
it sees a fresh set of coloured shapes, and nothing in a picture says whether
this is a box it has already dealt with. Moving a box across the table solves
that without any bookkeeping at all: the near half holds the boxes still to do,
the far half holds the ones that are done, and the arm can tell them apart by
looking. The to-do list is the table itself.

One thing is assumed: the cuboids are set apart from each other, not touching
and not stacked.

## How it works

Each cuboid goes through the same five steps.

**1. Look at the near half of the table.** The arm carries the camera to three
viewpoints above it and takes a picture at each one. Three rather than one,
because a tall box can hide a short one from a single angle.

**2. Turn the pictures into boxes.** The cuboids are the only strongly coloured
things in the cell, so colour alone separates them from the grey table. Those
pixels have a depth reading, and depth plus the arm's own pose puts each pixel
at a point in the room. Grouping the points that sit near each other gives one
cloud per box, and the smallest rectangle that encloses a cloud seen from above
is that box's footprint.

**3. Carry one box to the far half.** The arm takes the nearest one, closes its
fingers on the shorter of the two horizontal sides — the only one that fits
between them — lifts it across and sets it down.

**4. Measure it properly, on its own.** The arm looks again from three closer
viewpoints. This second look is the one the answer is worked out from, for two
reasons. The box is now alone, with nothing beside it to confuse the grouping.
And a box that has been picked up and put down does not always land on the face
it started on, so its old measurements may describe a box that no longer exists.

**5. Work out the biggest face and touch it.** Six faces, but only three
different areas, because opposite faces match. The biggest one wins, except
that the face lying on the table is ignored, since the arm cannot get
underneath it. The arm then lines up a short distance out from the centre of
that face and moves straight in until its fingertips press against it.

## What it looks like when it runs

```
[cuboid_task]: 3 cuboid(s) waiting on the pending side
[cuboid_task]:   8 x 6 x 9 cm at [0.49, -0.162, 0.793]
[cuboid_task]:   8 x 5 x 8 cm at [0.44, -0.294, 0.791]
[cuboid_task]:   8 x 4 x 4 cm at [0.324, -0.174, 0.772]
[cuboid_task]: --- cuboid 1 of 3 ---
[cuboid_task]: picking up a 8 x 4 x 4 cm box and moving it to the done side
[cuboid_task]: fingers closed to 41 mm on a 41 mm side
[cuboid_task]: measured 7.8 x 4.1 x 4.5 cm
[cuboid_task]:   length x width     32.4 cm^2
[cuboid_task]:   length x height    35.1 cm^2
[cuboid_task]:   width x height     18.6 cm^2
[cuboid_task]:   biggest reachable face is +width, centre [0.428, 0.157, 0.772]
[cuboid_task]: touched it
...
[cuboid_task]: finished: 3 cuboid(s) measured and touched
[cuboid_task]:   1. 7.8 x 4.1 x 4.5 cm, biggest reachable face +width (35.1 cm^2), contact yes
[cuboid_task]:   2. 8.7 x 6.1 x 8.6 cm, biggest reachable face +width (74.5 cm^2), contact yes
[cuboid_task]:   3. 7.7 x 4.7 x 8.3 cm, biggest reachable face -width (63.7 cm^2), contact yes
```

The three cuboids in that run were really 7.9 x 4.1 x 4.5, 7.8 x 4.7 x 8.2 and
6.1 x 8.6 x 8.6 cm, so the camera reads them to about a millimetre.

`contact yes` is the fingertip sensor reporting that it felt the face. Without
it, a measurement that was two centimetres wrong would still be logged as a
success, because the arm would have gone exactly where it calculated. The
sensor is what makes the last line a fact rather than a claim.

## The cell

| Piece | What it is |
| --- | --- |
| Arm | **UR5e**, a 6-axis arm from Universal Robots. The model comes from their own `ur_description` package. |
| Gripper | A two-finger parallel gripper, defined in this repo. Closed, the two fingers meet and act as the single blunt tip that touches a face. |
| Camera | An RGB-D camera on the wrist, next to the gripper, looking the way the fingers point. It moves with the arm. |
| Table | 1.4 m by 1.2 m, top at 75 cm. The arm stands on it. The half at negative y is where cuboids start, the half at positive y is where they end up. |
| Cuboids | Boxes between 4 and 9 cm on a side, in random sizes, colours, positions and orientations. `SEED` picks the arrangement. |

Everything in it is open source:

| Layer | Tool |
| --- | --- |
| Simulator | [Gazebo](https://gazebosim.org) Harmonic |
| Middleware | [ROS 2](https://docs.ros.org) Jazzy |
| Motion planning | [MoveIt 2](https://moveit.ai) |
| Joint control | [ros2_control](https://control.ros.org) |
| Perception | [OpenCV](https://opencv.org) and NumPy |
| Environment | [pixi](https://pixi.sh), with ROS packages from [RoboStack](https://robostack.github.io) |

## Commands

```
make            # list everything you can run
make run        # build if needed, then start the cell and run the task
make cell       # start the cell but leave the arm alone, for poking at by hand
make test       # run the tests
make doctor     # print versions of everything that matters
```

`make run` takes settings:

```
make run CUBOIDS=4 SEED=12   # four cuboids, a different arrangement
make run GUI=false           # no Gazebo window, for a machine without a display
make run RVIZ=true           # also open RViz to see what MoveIt is planning against
```

Three is the number the table is really sized for, and those runs go through
cleanly. Four fits and finishes, but leans on the arm retrying the odd box.
Above four the cuboids cannot be spaced far enough apart to be sure they are
not touching, and the run stops before it starts rather than measure two boxes
as one.

## Reading further

- [`docs/`](docs/) — a walk through one run, one document per step, with the
  pictures and the arithmetic for the parts that are hard to see in the code.
- [`PSEUDOCODE.md`](PSEUDOCODE.md) — what every file and function is for, and
  what calls what when you type `make run`.
- [`ARCHITECTURE.md`](ARCHITECTURE.md) — the folder layout, and why the pieces
  are split up the way they are.
- [`IMPLEMENTATION_NOTES.md`](IMPLEMENTATION_NOTES.md) — why each choice was
  made, the maths behind the measuring, and what breaks it.
