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

- pick it up and set it down on the other half of the table, so the side it
  started on always holds exactly the boxes that are still to do;
- measure its length, width and height using a camera;
- work out the area of its three different faces — length x width, length x
  height, width x height — and find the biggest one;
- reach out and touch the centre of that face.

The cuboids are assumed to be set apart from each other, not touching and not
stacked.

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
6.1 x 8.6 x 8.6 cm. The first survey reads them off the pending side to the
nearest millimetre or so. Each one is then measured again once it has been moved,
which is the number the face areas are worked out from, because a box that has
been picked up and set down does not always land on the face it started on.

`contact yes` at the end means the fingertip sensor felt the face. It is the
difference between the arm reaching the place it calculated and the arm
actually touching the box.

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
make run CUBOIDS=5 SEED=12   # five cuboids, a different arrangement
make run GUI=false           # no Gazebo window, for a machine without a display
make run RVIZ=true           # also open RViz to see what MoveIt is planning against
```

## Reading further

- [`ARCHITECTURE.md`](ARCHITECTURE.md) — what each file is for and how they fit together.
- [`IMPLEMENTATION_NOTES.md`](IMPLEMENTATION_NOTES.md) — why it is built this way, the
  maths behind the measuring, and what breaks it.
