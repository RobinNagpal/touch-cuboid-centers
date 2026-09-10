# assemble-table

A robot arm builds a small table. It finds a table top leaning against a wall
and four legs lying on the floor, measures the top, stands the legs up where a
top that size needs them, then lifts the top and lays it on the legs.

Everything runs in simulation. One command starts it.

```
make run
```

The first run downloads the environment, which is a few gigabytes.

This folder is a project of its own. It shares no code or configuration with
`../v1`; it only uses the same tools.

## The problem

In the room there are:

- a UR5e arm, bolted to the floor;
- a low wall somewhere in reach of it;
- a table top — a thin board — standing on its long edge and leaning back
  against the wall;
- four table legs lying on the floor, in no particular arrangement.

The arm has to:

1. find all of that with its camera;
2. measure the table top: its length, width and thickness;
3. work out where each leg has to stand for a top that size;
4. stand the four legs up on those spots;
5. pick the top up and put it on the legs.

**The arm is told nothing about the room.** It knows where it is bolted down
and how its own gripper and camera are built, and that is all. It does not
know how high the floor is, where the wall is, how big the top is, how far it
leans, or where the legs are or how long they are. Every one of those is drawn
at random for each run, and the arm has to measure them. The only way the
simulator's numbers could leak to the robot is through the code, and a test
fails if any robot code so much as imports them.

The one thing that is assumed: the parts are coloured and the room is grey.
That is how the camera tells a part from the floor. Which part is which is
decided by shape alone — the top is the one broad thin plate, a leg is a stick.

## How it works

**1. Look round the room.** The arm swings the camera round itself in eight
steps, looking down and out at the floor. The floor is found as the biggest
level surface in all those pictures. Everything coloured is a part, everything
grey standing up off the floor is an obstacle.

**2. Measure the top up close.** The arm goes to the top and looks at it from
four angles. A plane fitted to its face gives its lean, the spread of points
across the face gives length and width, and a look down on its upper edge
gives its thickness.

**3. Plan the table.** The table is built with the edge the arm picks the top
up by facing the arm. The legs go just in from the top's corners. The arm
picks a patch of floor within comfortable reach with nothing on it, measured
against everything it saw.

**4. Stand up each leg.** The arm grips a leg round its middle from above,
lifts it, turns it upright about its own centre, carries it to its spot and
sets it down. Then it looks to check the leg is really standing there, and
where exactly. A leg that falls over is found again and tried again.

**5. Lay the top on.** The arm grips the top by the middle of its upper edge,
with a finger either side of the board, lifts it off the wall, turns it level
and lowers it onto the legs — onto where the legs actually are, not where they
were planned to be.

**6. Check the table.** The top and the legs now touch, so the camera sees them
as one lump, and a box fitted to it is the finished table. The arm reports its
size, its height against what it should be, and how far from level the top is.

## What it looks like when it runs

That is seed 1, headless, about seven minutes from start to finish:

```
[assembly_task]: waiting for the cell to come up
[assembly_task]: looking round the room
[assembly_task]: floor at z = 0.1 mm
[assembly_task]:   obstacle, 44.4 x 9.7 x 7.2 cm, at [-0.165, 0.528]
[assembly_task]:   table top, 26.0 x 18.0 x 1.8 cm, at [-0.11, 0.49, 0.087]
[assembly_task]:   leg, 13.1 x 3.3 x 3.4 cm, at [-0.002, -0.452]
[assembly_task]:   leg, 13.1 x 3.3 x 3.3 cm, at [-0.351, -0.382]
[assembly_task]:   leg, 13.1 x 3.3 x 3.4 cm, at [0.386, -0.459]
[assembly_task]:   leg, 13.0 x 3.3 x 3.4 cm, at [-0.059, -0.565]
[assembly_task]: table top: 26.1 x 18.1 cm, 1.8 cm thick
[assembly_task]:   standing on its edge, leaning 20.6 degrees back from upright
[assembly_task]: building a 26.1 x 18.1 cm table, 14.9 cm high, centred at [0.57, 0.0]
[assembly_task]:   a leg goes at [0.631, -0.102]
[assembly_task]:   a leg goes at [0.631, 0.102]
[assembly_task]:   a leg goes at [0.509, -0.102]
[assembly_task]:   a leg goes at [0.509, 0.102]
[assembly_task]: --- leg 1 of 4 ---
[assembly_task]: picking up a 13.1 x 3.3 x 3.4 cm leg
[assembly_task]: fingers closed to 33 mm on 33 mm
[assembly_task]: leg standing, 13.1 cm tall, 3 mm from its spot
[assembly_task]: --- leg 2 of 4 ---
[assembly_task]: picking up a 13.1 x 3.3 x 3.4 cm leg
[assembly_task]: fingers closed to 33 mm on 33 mm
[assembly_task]: turned the leg 96% of the way upright
[assembly_task]: leg standing, 13.1 cm tall, 12 mm from its spot
[assembly_task]: --- leg 3 of 4 ---
[assembly_task]: picking up a 13.1 x 3.3 x 3.4 cm leg
[assembly_task]: fingers closed to 33 mm on 33 mm
[assembly_task]: turned the leg 98% of the way upright
[assembly_task]: leg standing, 13.1 cm tall, 8 mm from its spot
[assembly_task]: --- leg 4 of 4 ---
[assembly_task]: picking up a 13.1 x 3.3 x 3.4 cm leg
[assembly_task]: fingers closed to 33 mm on 33 mm
[assembly_task]: turned the leg 96% of the way upright
[assembly_task]: leg standing, 13.1 cm tall, 2 mm from its spot
[assembly_task]: --- table top ---
[assembly_task]: fingers closed to 18 mm on 18 mm
[assembly_task]: table built: 26.0 x 18.3 x 14.9 cm, top 0.0 degrees off level
[assembly_task]: finished
[assembly_task]:   table top measured at 26.1 x 18.1 x 1.8 cm
[assembly_task]:   4 legs standing, 13.1 cm long
[assembly_task]:   table as built: 26.0 x 18.3 x 14.9 cm
[assembly_task]:   height 14.9 cm, expected 15.0 cm
[assembly_task]:   top 0.0 degrees off level
[assembly_task]:   centre 6 mm from where it was planned
[assembly_task]:   verdict: a table
```

What the simulator had actually put there: a top 26.04 x 17.98 x 1.78 cm
leaning 20.5°, and legs 13.09 cm long and 3.34 cm square. Afterwards, asked
directly, Gazebo had the top lying dead level on four upright legs, its
underside 13.1 cm off the floor.

"12 mm from its spot" is what the camera saw after letting go: legs land a
few millimetres off where they were aimed, and one that tips a little as the
fingers open lands further. That is why the top is put down over where the
legs actually are, rather than over where they were meant to be — "centre
6 mm from where it was planned" is that correction showing.

"turned the leg 96% of the way upright" is the leg being stood up where it lay
before being carried; see IMPLEMENTATION_NOTES.md for why it does not need to
finish the turn exactly.

## The cell

| Piece | What it is |
| --- | --- |
| Arm | **UR5e**, a 6-axis arm from Universal Robots, standing on the floor. The model comes from their own `ur_description` package. |
| Gripper | A two-finger parallel gripper, defined in this repo. Each finger grips through four small pads. |
| Camera | An RGB-D camera on the wrist, beside the gripper, looking the way the fingers point. |
| Wall | A low wall, 45 cm long, 6 to 8 cm high, somewhere to the arm's left. |
| Table top | A light board, 24–32 cm by 16–20 cm, 1.6–2.0 cm thick, standing on its long edge and leaning back 15–22° onto the wall. |
| Legs | Four identical sticks, 13–16 cm long and 2.5–3.5 cm square, lying anywhere to the arm's right. |

`SEED` picks the room. The finished table is about 15 to 18 cm tall.

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
make run        # build if needed, then start the cell and build the table
make cell       # start the cell but leave the arm alone, for poking at by hand
make test       # run the tests
make doctor     # print versions of everything that matters
```

`make run` takes settings:

```
make run SEED=7          # a different room
make run GUI=false       # no Gazebo window, for a machine without a display
make run RVIZ=true       # also open RViz to see what MoveIt is planning against
```

To keep every picture the camera takes, with the pose it was taken from:

```
TABLE_ASSEMBLY_VIEWS=/tmp/views make run
```

## Reading further

- [`PSEUDOCODE.md`](PSEUDOCODE.md) — what every file and function is for, and
  what calls what when you type `make run`.
- [`ARCHITECTURE.md`](ARCHITECTURE.md) — the folder layout, the line between the
  simulator and the robot, and why the pieces are split up the way they are.
- [`IMPLEMENTATION_NOTES.md`](IMPLEMENTATION_NOTES.md) — why each choice was
  made, the maths and the physics behind it, and what breaks it.
