# assemble-table

A robot arm builds a small table. It finds a table top lying on two stands
and four legs standing on the floor, measures the top, moves the legs to
where a top that size needs them, then lifts the top and lays it on the legs.

Everything runs in simulation. One command starts it.

```
make run
```

The first run downloads the environment, which is a few gigabytes. On macOS
the first run after that is slow too — minutes before the arm moves — because
every newly installed library is checked the first time it loads. After that
the cell is up in seconds.

This folder is a project of its own. It shares no code or configuration with
`../v1`; it only uses the same tools.

## The problem

In the room there are:

- a UR5e arm, bolted to the floor;
- a table top — a thin board — lying flat on two low grey stands, one under
  each end, to the arm's left;
- four table legs standing on end on the floor to the arm's right, in no
  particular arrangement.

The arm has to:

1. find all of that with its camera;
2. measure the table top: its length, width and thickness;
3. work out where each leg has to stand for a top that size;
4. move the four legs onto those spots, standing;
5. pick the top up and put it on the legs.

**The arm is told nothing about the room.** It knows where it is bolted down
and how its own gripper and camera are built, and it always builds the table
on the same patch of floor in front of it. That is all. It does not know how
high the floor is, where the top is or how big, or where the legs are or how
long they are. Every one of those is drawn
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

**2. Measure the top up close.** The arm looks down on the top from four
angles on its own side. The spread of points across the upper face gives
length and width, and the near side face gives its thickness.

**3. Plan the table.** The table is always built in the same place, 60 cm
straight in front of the arm, with the top's long edge facing the arm. The
legs go just in from the top's corners, so where they go depends only on the
top's measured size. The arm checks the spot is empty first.

**4. Move each leg.** Before touching a leg, the arm checks it can reach the
pose that sets it down on its spot. Then it grips the leg from straight above
by its top end, lifts it, carries it round above the other legs, lowers it
onto its spot and lifts off. The leg hangs straight down from the fingers the
whole way, so nothing tips or turns. Then the arm looks to check the leg is
really standing there, and where exactly. If an attempt fails, it looks round
again and tries whichever standing leg it finds.

**5. Lay the top on.** The arm reaches in level over the middle of the top's
near edge, one finger above the board and one below it, between the stands.
It lifts the top off, carries it round level, and lowers it onto the legs —
onto where the legs actually are, not where they were planned to be.

**6. Check the table.** The top and the legs now touch, so the camera sees them
as one lump, and a box fitted to it is the finished table. The arm reports its
size, its height against what it should be, and how far from level the top is.

## What it looks like when it runs

That is seed 1, headless, about four and a half minutes from start to finish:

```
[assembly_task]: waiting for the arm's joint states
[assembly_task]: waiting for the cell to come up
[assembly_task]: looking round the room
[assembly_task]: floor at z = 0.1 mm
[assembly_task]:   table top, 25.9 x 18.5 x 1.8 cm, at [-0.07, 0.583, 0.095]
[assembly_task]:   leg, 2.5 x 2.5 x 13.3 cm, at [0.155, -0.462]
[assembly_task]:   leg, 2.5 x 2.5 x 13.3 cm, at [-0.239, -0.539]
[assembly_task]:   leg, 2.6 x 2.5 x 13.3 cm, at [-0.312, -0.376]
[assembly_task]:   leg, 2.5 x 2.5 x 13.3 cm, at [-0.073, -0.4]
[assembly_task]: table top: 25.6 x 18.0 cm, 1.8 cm thick
[assembly_task]:   lying 0.0 degrees off level
[assembly_task]: building a 25.6 x 18.0 cm table, 15.1 cm high, centred at [0.6, 0.0]
[assembly_task]:   a leg goes at [0.666, -0.103]
[assembly_task]:   a leg goes at [0.666, 0.103]
[assembly_task]:   a leg goes at [0.534, -0.103]
[assembly_task]:   a leg goes at [0.534, 0.103]
[assembly_task]: --- leg 1 of 4 ---
[assembly_task]: picking up a 2.6 x 2.5 x 13.3 cm leg
[assembly_task]: fingers closed to 25 mm on 26 mm
[assembly_task]: leg standing, 13.3 cm tall, 2 mm from its spot
[assembly_task]: --- leg 2 of 4 ---
[assembly_task]: picking up a 2.6 x 2.5 x 13.3 cm leg
[assembly_task]: fingers closed to 25 mm on 26 mm
[assembly_task]: leg standing, 13.3 cm tall, 2 mm from its spot
[assembly_task]: --- leg 3 of 4 ---
[assembly_task]: picking up a 2.6 x 2.5 x 13.3 cm leg
[assembly_task]: fingers closed to 25 mm on 26 mm
[assembly_task]: leg standing, 13.3 cm tall, 3 mm from its spot
[assembly_task]: --- leg 4 of 4 ---
[assembly_task]: picking up a 2.6 x 2.5 x 13.3 cm leg
[assembly_task]: fingers closed to 25 mm on 26 mm
[assembly_task]: leg standing, 13.3 cm tall, 2 mm from its spot
[assembly_task]: --- table top ---
[assembly_task]: fingers closed to 18 mm on 18 mm
[assembly_task]: table built: 25.5 x 18.0 x 15.1 cm, top 0.0 degrees off level
[assembly_task]: finished
[assembly_task]:   table top measured at 25.6 x 18.0 x 1.8 cm
[assembly_task]:   4 legs standing, 13.3 cm long
[assembly_task]:   table as built: 25.5 x 18.0 x 15.1 cm
[assembly_task]:   height 15.1 cm, expected 15.1 cm
[assembly_task]:   top 0.0 degrees off level
[assembly_task]:   centre 3 mm from where it was planned
[assembly_task]:   verdict: a table
```

What the simulator had actually put there: a top 25.53 x 17.98 x 1.78 cm,
and legs 13.28 cm long and 2.53 cm square. Afterwards, asked directly, Gazebo
had the top level to within 0.02°, centred at [0.602, -0.001], its underside
13.29 cm off the floor, resting on four upright legs.

The survey's first look at the top (25.9 x 18.5 cm) is from far off and at a
slant; the close look (25.6 x 18.0 cm) is the one the plan uses. The stands
are not listed as obstacles: the top hides most of them, and what shows is
taken for the top's own shadowed sides.

"2 mm from its spot" is what the camera saw after letting go. The top is put
down over where the legs actually are, rather than over where they were meant
to be — "centre 3 mm from where it was planned" is that correction showing.

## The cell

| Piece | What it is |
| --- | --- |
| Arm | **UR5e**, a 6-axis arm from Universal Robots, standing on the floor. The model comes from their own `ur_description` package. |
| Gripper | A two-finger parallel gripper, defined in this repo. Each finger grips through four small pads. |
| Camera | An RGB-D camera on the wrist, beside the gripper, looking the way the fingers point. |
| Stands | Two grey blocks, 8–9 cm high, one under each end of the top. |
| Table top | A light board, 24–30 cm by 16–20 cm, 1.6–2.0 cm thick, lying flat on the stands 55–60 cm to the arm's left, its long side facing the arm, give or take 8°. |
| Legs | Four identical sticks, 13–16 cm long and 2.5–3.5 cm square, standing on end anywhere 40–60 cm to the arm's right, at least 12 cm apart. |

`SEED` picks the room: it is the seed of the random numbers every size and
position above is drawn from, so the same seed always gives the same room.
The finished table is about 15 to 18 cm tall.

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
