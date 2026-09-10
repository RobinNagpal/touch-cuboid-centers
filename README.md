# Robot arm projects

A learning repo. It holds two robot arm projects, `v1` and `v2`, each written to
showcase a different set of the concepts you run into when a 6-axis arm has to
look at something and then do something about it: where to put a camera, how
pixels become millimetres, what a plan is worth once a part lands somewhere
other than where it was aimed.

Both run entirely in simulation — Gazebo, ROS 2, MoveIt — so neither needs
hardware to try. The repo is named after `v1`, which was the first thing in it.

## v1 — touch the centre of the biggest face

Cuboids of random sizes sit on a table. For each one the arm measures it with a
wrist camera, works out which of its faces has the largest area, and presses a
fingertip into the middle of that face.

It is the smallest complete example of an arm that looks before it moves:
carrying a camera to a viewpoint, separating coloured pixels from a grey world,
turning those pixels into points in the room, fitting a box to the points, and
finishing with a contact sensor so the last line of the log is a fact rather
than a calculation. The pick-and-place in the middle is there to serve
perception, not the other way round — moving a box to the far half of the table
is how the arm remembers which boxes it has already done, without keeping a
list.

→ [`v1/README.md`](v1/README.md)

## v2 — build a table

A table top leans against a wall and four legs lie on the floor. The arm
measures the top, works out where legs have to stand to hold up a top that
size, stands the four of them up, and lays the top on them.

→ [`v2/README.md`](v2/README.md)

## Why there are two

`v2` is not `v1` cleaned up. It exists to push on the two things `v1` deliberately
made easy.

**How much the robot is told.** In `v1` the table is a known quantity: its
height, its edges and the two halves it is divided into are numbers in the code,
and only the cuboids have to be measured. In `v2` the robot knows where it is
bolted down and how its own gripper and camera are built, and nothing else — the
floor height, the wall, the top's size and lean, and every leg's size and place
are all drawn at random per run and have to be measured. That is the harder and
more realistic arrangement, and it is easy to break by accident, so a test
fails if any robot code so much as imports the simulator's side.

**One object, or several that have to agree.** `v1` measures a box and acts on
that one box; a millimetre of error costs a millimetre. `v2` has to make four
legs and a top into a single thing that stands up, where being right depends on
every part at once. Parts land a few millimetres off where they were aimed and
sometimes tip over, so the arm has to look again after each placement and put
the top down over where the legs actually are. Measure, act, look again, correct
— that loop is the whole point of the second project.

Each folder is a project of its own. They share no code and no configuration,
only the same tools, and each has its own environment and its own `make run`.
Start with the README inside whichever one you are reading.
