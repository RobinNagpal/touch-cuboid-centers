# Implementation notes

Why it is built this way, what the physics taught along the way, and what
breaks it.

## The rule everything follows

The robot measures the room; it is never told it. The only numbers it starts
with are about itself, in `arm/dimensions.py` — where its base is, how its
tooling is built — and one choice about how it works: it always builds the
table on the same patch of floor in front of it (`SITE` in `assembly/plan.py`).
Everything else — the floor height, the stands, where the top lies and its
size, the legs' size and positions — is drawn at random by `world/spawn.py`
for each run and has to be found with the camera.

That is enforced by a test, not by good intentions. `test_world.py` reads
every module outside `world/` and fails if any of them imports from it. The
tests are the one place the two sides meet, and there they do the job a
referee would: the robot's measurements on one side, the simulator's truth on
the other.

## When it goes wrong

Nothing here is quiet about failure, because an arm that quietly does the
wrong thing is worse than one that stops.

- **A part with nowhere to go.** Before a part is picked up, the arm checks it
  can reach the pose that puts it down, with its wrist the way it will be
  then. A grip with no such pose is not used. Found out with the part in the
  air, the only thing left to do is let go of it wherever the arm is.
- **A move that did not arrive.** The planner and the controller both report
  success when they have done their part, which is not the same as the arm
  being where it was asked to be. After every move the arm reads where its
  tool really is and refuses to carry on if it is more than 5 mm or 1.7° out.
- **A grasp that closed on nothing**, and a part that worked loose on the way,
  are caught by the fingertip contact sensors at the moment they happen.
- **Fingers that did not open.** Before the gripper is lowered over a part,
  each finger is checked to be where it was sent — not just the gap between
  them.
- **A leg that is not where it was put.** After a leg is put down the arm
  looks, and only counts it as done if a standing leg of the right height is
  within 3 cm of its spot. Otherwise it lifts clear, looks round the room, and
  tries whichever standing leg it finds, up to three times.
- **Two legs touching.** They look like one lump twice the size, and a lump
  that is not the size of one leg is never picked up.
- **Something on the building spot.** The run says so before moving anything.
- **Letting go after a failure.** The gripper is lifted clear first, with
  collision checking on, and only then opened. Fingers told to open while the
  arm is pressing them down on a part stay where they are; and an unchecked
  move, tried once, folded the arm into itself so badly that MoveIt would plan
  nothing afterwards.
- **A move MoveIt refuses** says why: no joint angles reach the pose, or only
  with the elbow or wrist the other way, or every way of reaching it hits
  something, with the contacts in the log.
- **Anything else.** A run that cannot carry on ends with one line saying
  why, not a traceback.

The run ends with what was built: its size, its height against what it should
be, how far the top is from level, and a verdict.

## Choices

### The arm on the floor, the parts around it

The problem puts the legs on the floor, so the arm stands on the floor too.
A UR5e reaches 85 cm; standing at floor level, it can reach the floor around
it from about 35 cm out to 70 cm, and that ring is where everything happens.
The top lies on its stands to the arm's left, the legs stand to its right,
and the table is built straight in front.

### The legs start standing

The legs used to start lying on the floor, and the arm stood each one up. That
meant gripping it, turning it upright in the air, and setting it down with the
gripper held level beside it a few centimetres off the floor. It was the one
part of the task that never became reliable, for two reasons, both found by
running it:

- **The wrist.** A 6-axis arm can reach most poses with its wrist flipped
  either way, and no straight-line move can turn one into the other: it would
  have to pass through the point where two wrist axes line up. A leg picked up
  from above left the wrist one way, and in that shape a level gripper
  pointing out from the base near the floor puts the forearm into the floor.
  The legs furthest out could not be set down at all.
- **The planner's answers.** Asked for joint angles for those low, level
  poses, the solver came back with a different shape of arm from one run to
  the next — the base turned backwards, the elbow bent the wrong way — so the
  same leg would go down in one run and not the next.

With the legs standing, the arm grips a leg by its top end from straight
above, carries it hanging straight down, and sets it down on its spot. The
gripper points down the whole time and stays well above the floor, and the
wrist never needs to change shape. What the problem is about is unchanged:
the arm still has to find the legs, measure them, and work out where they go
from the size of the top.

### Two stands for the top

The top lies flat on two low grey stands, one under each end. The arm picks it
up by the middle of its near edge, from the side, with one finger above the
board and one below — so the middle of that edge has to be open underneath,
which is why there are two stands at the ends rather than one in the middle.
The stands are 8 to 9 cm high so that the gripper, held level at the board
with its fingers one above the other, keeps its 11 cm body clear of the floor.
`test_grasps.py` checks, for twenty rooms, that the lower finger misses both
stands and the gripper misses the floor.

Supported under both ends, the board does not tip if a finger touches it
first as the fingers close. A single narrow stand under its middle would let
a push at the edge tip it off.

### The gripper: pads, not plates

The two-finger gripper is written here rather than borrowed: two fingers
commanded to the same number need nothing special from the simulator, where a
mimic joint does.

The pads are the most important thing this project learned about simulated
physics. The first version had plain flat fingers. They squeezed at their full
25 N, and still a part held a few millimetres off its middle swung round in
the fingers as soon as it was lifted and fell out. The contact messages showed
why: each finger touched the part at **one point**. Pressed flat against a
flat face, the simulator resolves the contact to a single point, and a single
point can stop sliding but cannot stop turning.

So each finger grips through four small pads, two at the fingertip and two
further up, standing 1.5 mm proud of the finger's face. Four pads touch at four
points, and the 2 cm between them is what resists the turn. Real grippers do
the same with ridged or rubber pads.

### The fingers: short of their end stops, and one at a time

Two things about the simulated fingers cost a run each before they were
understood.

- **A finger can lag.** One finger sometimes sets off a second or more after
  the other. `set_gripper()` used to decide the fingers were done when the gap
  between them stopped changing, and a finger that has not started yet leaves
  the gap unchanged. The arm then came down over a standing leg while the late
  finger was still closing in, and knocked the leg over. Now each finger is
  watched on its own, and `open_gripper()` refuses to go on unless both are
  where they were sent.
- **A finger can stick at its end stop.** After the arm has been moving, a
  finger sent right to the end of its travel has stayed there when told to
  close, for good. So the fingers are never opened past 7.4 cm of their 8.

### Where to hold each part

Both parts are held so that their weight does not try to turn them in the
fingers at all, or as little as possible.

- **A leg is gripped by its top end, from above.** It hangs straight down from
  the fingers, so its weight pulls along them.
- **The top is gripped at the middle of its near edge, from the side.** Held
  level, its weight tries to tip it down about the edge, and that is resisted
  by the fingers pressing on both faces — top finger down at the edge, bottom
  finger up at the tips — which the two rows of pads give a lever to do. The
  fingers reach 5 cm in, so both rows are on the board.

### Carrying a part round the base

A part in hand is carried up first, then round the base on an arc at a fixed
height (`carry_round()`): distance from the base and bearing both change
evenly, so the path never cuts in close to the base, and the tool turns about
the vertical only, so a hanging leg stays hanging and a level top stays level.
It is a slow straight-line path, checked against everything in the room. If it
is refused, the planner is asked for a way to its end with the arm kept in its
usual shape.

Legs are carried high enough that a leg's lower end passes 4 cm above every
leg standing; the top high enough that it passes above the legs it is going
onto.

All moves with a part in hand run at a tenth of the arm's speed.

### The hold

Once a part is in the gripper, part and tool move as one rigid body. That
relationship — the tool's pose written in the part's own frame — is recorded
at the moment of the grasp as the *hold*. Putting the part anywhere is then
one line: the tool goes to *where the part should be* times *the hold*.
Standing a leg on its spot and laying the top level are just a new pose for
the part, and `grasps.py` never has to reason about fingers at all.

### Reaching a pose the same way every time

A 6-axis arm can usually reach a pose in up to eight ways — elbow up or down,
shoulder forward or back, wrist flipped or not — and each joint that can turn
a full circle each way also has two angles, a full turn apart, that put the
arm in the same place. Given only a pose, the planner picks among all of these
at random. Each of these happened because of it:

- the camera found the arm's own elbow filling half the picture, hiding the
  legs behind it;
- a leg was carried all the way round the base, a full circle, to end up
  where it started;
- a carry flipped the whole arm over from elbow-up to elbow-down with a leg in
  the fingers.

So `Arm.move_to()` works out the joint angles itself before asking the
planner for a path:

- The search always starts from the same posture: the ready one — elbow up,
  upper arm upright, forearm out level — turned on the base to face the pose.
  Starting from wherever the arm happened to be let one odd posture breed the
  next.
- An answer with the elbow bent the other way from that posture is a
  different shape of arm, and is thrown away. With a part in hand, so is one
  with the wrist flipped the other way from how it is now.
- Each joint is then set to whichever of its equivalent angles is nearest
  where it is now, so nothing turns further than it needs to — but never
  within half a radian of a full turn. Left to take the nearest angle every
  time, the last wrist joint crept, leg by leg, to exactly a full turn, and
  the straight line in to the top's edge that followed stopped a third of
  the way: the joint had nowhere left to turn.
- The answer is checked by working out where it really puts the tool, because
  the solver was once seen to report success with something else; and checked
  for collisions, with the fingers as open as they really are and the part in
  hand, because a colliding answer handed to the planner only fails after ten
  seconds and a screenful of errors.

If none of that works, the planner may be given the bare pose — but not with
a part in hand. Then the move simply fails, and the next pose on the list is
tried.

On top of that, MoveIt is told the upper arm may never point below level
(`joint_limits.yaml`). The arm stands on the floor; lower than that, the elbow
is at the floor.

### Where the table is built

Always in the same place: centred 60 cm straight in front of the arm, with the
top's long edge facing it. That is the arm's choice of where to work, not
knowledge of the room — the arm still checks nothing it saw is on or within
8 cm of that patch, and the simulator never puts anything there
(`test_plan.py` checks thirty rooms).

It is 60 cm rather than closer because the top is let go of by pulling the
gripper back out from under it, towards the base. With the table at 55 cm the
arm was already folded up to reach the near edge, and pulling back 7 cm folded
its upper arm into its wrist. At 60 cm, pulling back 6 cm is comfortable. The
far legs, at about 67 cm, are gripped from straight above, which the arm
reaches easily.

### Two pictures of the room

MoveIt keeps its own picture of the room to plan against, and in this setup
there are two copies of it: one in `move_group`, which straight-line moves
are checked against, and one inside the task node, which free moves are
planned against.

They do not keep each other up to date. The task node's copy listens for
changes on `/planning_scene`, which nothing publishes to — whatever the
`monitored_planning_scene_topic` setting's name suggests. For a long time
every obstacle went only to `move_group`, and free moves were being planned in
an empty room, until the wrist was driven into a leg on the floor. Now
`scene.py` applies every change to both, one through `move_group`'s service
and one directly.

### The camera: pictures and poses from the same moment

A frame from a wrist camera means nothing without the pose the camera was at
when it was taken. Two things can pair a picture with the wrong pose, and both
happened:

- **A frame from before the arm stopped** can arrive after it has. So
  `capture()` only accepts frames stamped after it was called.
- **The latest pose is not the pose at the frame's moment.** The arm's joint
  states and the images travel separately, and the pose is looked up at the
  image's own timestamp.

### Starting up: waiting rather than guessing

The task waits for the arm's joint states, the planning scene, the
controllers and the camera themselves rather than trusting a delay.

The first run after an install is very slow on macOS: every newly installed
library is checked the first time it loads, and in one such run Gazebo took
over four minutes to load the world. Two timeouts were too short for that:

- **MoveIt gives up after ten seconds** without joint states, for good, and
  the task node died with it. So `main.py` waits for the first joint state
  before MoveIt is started inside the task.
- **A controller spawner** found the controller manager, never heard back
  from it, and gave up after three minutes, while a fresh spawner started
  afterwards was answered at once. So a failed spawner is started again, up to
  three times, and the next controller only starts once the one before it has
  really loaded — the old chain started the next one regardless, which left
  the arm with no joint states.

## How the measuring works

### The floor

All grey points from the survey go into a histogram of height. The floor is by
far the biggest level surface in view, so the height most points share is the
floor, refined to under a millimetre with the median of the points near it. It
is found once, from the survey, and used from then on: in close-up views a
part can fill more of the picture than the floor does.

### Which pixels to trust

Pixels are split by colour: saturated is a part, grey is everything else.
Then pixels whose range jumps away from their neighbours of the same kind are
dropped (`flying_pixels()`). A depth camera looking at an edge can return a
range between the object and whatever is behind it, and such a pixel lands in
mid-air.

The first version instead shaved a pixel off every edge of every mask, which
is the usual quick fix, and every part measured one to two pixels small — 2 to
3 mm, enough that the fingers were told to squeeze 6 mm instead of 4. Comparing
depth only against neighbours *of the same kind* keeps a part's true edge
pixels, because the floor next to a leg is not in the leg's mask. With that
change the measurements came to within a millimetre.

### Back-projection

Masked pixels become points in the room through the camera intrinsics and the
camera's pose:

```
X = (u - cx) * depth / fx
Y = (v - cy) * depth / fy
Z = depth
```

then through `camera_to_world`, the pose of `wrist_camera_optical_frame` at the
frame's timestamp.

### Splitting and fitting

Points are dropped into a 12 mm voxel grid and neighbouring occupied voxels are
flood-filled into clusters, one per object. That works because the parts are
set apart; `spec.py` keeps legs at least 12 cm apart, centre to centre.

Two fits, for the two shapes in the room:

- **A box resting on the floor** — a standing leg, a stand, the finished
  table. Height is the highest point above the floor, because the camera
  always sees the top. Length and width come from the smallest rectangle
  enclosing the points seen from above, which *is* the footprint.
- **A flat board at any angle** — the top. A plane is fitted to the points by
  principal component analysis: the direction they spread least in is the
  plane's normal. The strip of side face seen at a slant drags that first
  plane off true, so the fit is repeated with only the points close to the
  plane, tightening the band each time, until only the face is left. In the
  face's own axes, the spread across it gives length and width, and how far
  the side face reaches below it gives the thickness. That is why the top is
  looked at from the arm's side and above: straight down, no side face shows.

### Telling the parts apart

By shape alone. The table top is the broadest cluster whose plate fit is thin
— thinner than a quarter of its shorter side. A leg is a cluster at least
twice as long as it is thick, and it is standing if it is taller than it is
wide. Anything coloured that is neither is reported and left alone.

Grey clusters standing up off the floor are obstacles, with two filters: grey
points within 3 cm of a part are that part's own shadowed sides, and a grey
cluster that does not reach down to the floor is the arm catching sight of
itself. The first filter also takes most of the stands: the top hides them,
and what shows is close under it. So the stands are usually not in MoveIt's
picture of the room. Nothing depends on them being there — the top is lifted
straight up off them, unchecked, before anything else moves.

### Accuracy

Against the sizes the simulator was told to spawn, from seed 1:

| | Measured | True |
| --- | --- | --- |
| Floor | 0.1 mm | 0 |
| Table top | 25.6 x 18.0 x 1.8 cm | 25.53 x 17.98 x 1.78 cm |
| Legs | 2.5 x 2.5 x 13.3 cm | 2.53 x 2.53 x 13.28 cm |

## Planning the table

The table goes at `SITE`. It is turned so the edge the top is gripped by — a
long edge — faces the arm. The legs stand 1.2 cm in from the top's edges. They
go in far ones first, so the gripper never has to reach past a leg it has
already put down.

The top goes on where the legs *are*, not where they were planned: its centre
over the middle of the four as measured after each was put down, its underside
3 mm above the tallest.

## What breaks it

- **Parts touching.** Two legs standing against each other are one lump to
  the camera. The arm will not pick up a lump the wrong size, but it cannot
  separate them either.
- **A leg that falls over.** The arm only picks legs up standing. A leg
  knocked over is left where it is, and if that leaves fewer than four, the
  run stops.
- **A top too thick or a leg too fat for the gripper.** The gripper opens to
  7.4 cm and the task stops if a part needs more than 6.5.
- **Parts outside the survey ring.** The survey sees the floor from about 35 cm
  to a metre out. A leg closer in than that is not found.
- **A top that is not lying flat**, or not with a long edge towards the arm.
  It is gripped from the side by the middle of its near long edge, and the arm
  stops if it is tipped more than 5°.
- **Something on the building spot.** The arm does not look for another.
- **Coloured obstacles.** Anything saturated is taken for a part.

## What is checked, and how

`make test` runs about 240 tests that need no simulator:

- **Fitting.** A floor, lying and standing legs at many yaws, leaning boards
  and level boards at several sizes are turned into the points a camera would
  see — only the faces towards it — and the fits must give the numbers back.
- **Reading a room.** Rooms drawn by the real spawner, for several seeds, are
  turned into survey points, and `read_room()` must find the floor, the top to
  2 mm, all four legs standing, and the two stands, and nothing else.
- **Pixels.** Edge pixels of a part are kept; a pixel hanging between a part and
  what is behind it is dropped; a floor seen at a slant is left alone.
- **Planning.** The legs stand inside the top's footprint, inset; far legs go
  first; the top ends up level at the right height; something on or beside
  the spot is noticed; and no room the simulator draws puts anything there.
- **Grasps.** The hold puts the tool back where it was; a standing leg is
  gripped from above by its top end and set down upright on its spot; a leg
  or the top carried round the base stays upright or level the whole way; the
  top is gripped level across its near edge and laid level; the finger under
  the top misses the stands in twenty rooms.
- **The world.** The same seed gives the same room; the top lies level on
  both stands with its middle open underneath; the legs stand upright and
  well apart; and nothing outside `world/` imports from it.

Beyond that, the only real test is running it.

## Debugging

```
make cell                                   # bring the cell up without running the task
gz model -m leg_0 -p                        # where a leg really is
gz model -m table_top -p                    # where the top really is
ros2 topic echo --once /joint_states        # finger positions and efforts
ros2 topic hz /wrist_camera/depth_image     # 15 Hz, or the transport is dropping frames
TABLE_ASSEMBLY_VIEWS=/tmp/views make run    # keep every picture, with its pose
make run RVIZ=true                          # see what MoveIt is planning against
```

The saved views are `.npz` files with the colour image, the depth image, the
intrinsics and the camera pose, so the whole perception step can be replayed
offline with the functions in `perception/`.

Gazebo prints `Unable to load Ogre Plugin ... Rendering will not be possible`
on macOS. Rendering does in fact work; the camera topics carry real images.

Kill leftovers between runs. A simulator killed rather than shut down leaves
its ROS endpoints registered, and MoveIt's Python node sometimes ignores a
polite kill altogether — check with `pgrep -fl assemble_table`.
