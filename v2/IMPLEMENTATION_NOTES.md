# Implementation notes

Why it is built this way, what the physics taught along the way, and what
breaks it.

## The rule everything follows

The robot measures the room; it is never told it. The only numbers it starts
with are about itself, in `arm/dimensions.py`: where its base is, how its
tooling is built, how far it reaches comfortably. Everything else — the floor
height, the wall, the table top's size and lean, the legs' size and positions —
is drawn at random by `world/spawn.py` for each run and has to be found with
the camera.

That is enforced by a test, not by good intentions. `test_world.py` reads
every module outside `world/` and fails if any of them imports from it. The
tests are the one place the two sides meet, and there they do the job a
referee would: the robot's measurements on one side, the simulator's truth on
the other.

## When it goes wrong

Nothing here is quiet about failure, because an arm that quietly does the
wrong thing is worse than one that stops.

- **A move that did not arrive.** The planner and the controller both report
  success when they have done their part, which is not the same as the arm
  being where it was asked to be. After every move the arm reads where its
  tool really is and refuses to carry on if it is more than 5 mm or 1.7° out.
- **A grasp that closed on nothing**, and a part that worked loose on the way,
  are caught by the fingertip contact sensors at the moment they happen.
- **Fingers that did not open.** Before the gripper is lowered over a part,
  both fingers are checked to be where they were sent.
- **A leg that fell over.** After a leg is put down the arm looks, and only
  counts it as done if a standing leg of the right height is within 3 cm of
  its spot. Otherwise it lifts clear, looks round the room, finds the leg
  wherever it ended up, and tries again, up to three times.
- **Two legs touching.** They look like one lump twice the size, and a lump
  that is not the size of one leg is never picked up.
- **No room.** If there is no clear patch of floor in reach big enough for
  the table, the run says so before moving anything.
- **Letting go after a failure.** The gripper is lifted clear first, with
  collision checking on, and only then opened. Fingers told to open while the
  arm is pressing them down on a part stay where they are; and an unchecked
  move, tried once, folded the arm into itself so badly that MoveIt would plan
  nothing afterwards.
- **Anything else.** A run that cannot carry on ends with one line saying
  why, not a traceback.

The run ends with what was built: its size, its height against what it should
be, how far the top is from level, and a verdict.

## Choices

### The arm on the floor, the parts around it

The problem puts the legs on the floor, so the arm stands on the floor too.
A UR5e reaches 85 cm; standing at floor level, it can reach the floor around
it from about 35 cm out to 70 cm, and that ring is where everything happens.
The arm looks round that ring, the wall and the parts are somewhere on it,
and the table is built on a clear patch of it.

### A low wall

The top leans against a wall, as asked. The wall is a low one — 6 to 8 cm —
and that is not for looks.

The top is picked up by its highest edge, with a finger either side of the
board. Against a full-height wall the board's upper edge rests *on* the wall,
and there is no room for the finger on the wall's side. Against a low wall the
board rests on the wall's top corner and its upper part stands clear above it,
so both fingers fit round the edge. `spec.py` only draws rooms where at least
9 cm of the board, measured up its face, stands clear of the wall, and
`test_grasps.py` checks, for twenty rooms, that the finger behind the board
passes above the wall with a centimetre to spare.

It is also stable. A board leaning on a corner is the classic ladder problem:
its weight, the push from the wall's corner, and the floor's support and
friction have to balance. With the corner pushing square to the board, the
friction needed at the floor is under a third of the board's weight at these
lean angles, well inside what the floor gives.

### The gripper: pads, not plates

The two-finger gripper is written here rather than borrowed, for the same
reason as in v1: two fingers commanded to the same number need nothing special
from the simulator, where a mimic joint does.

What is new is the pads, and they are the most important thing this project
learned about simulated physics. The first version had plain flat fingers.
They squeezed at their full 25 N, and still a leg held a few millimetres off
its middle swung round in the fingers as soon as it was lifted and fell out.
The contact messages showed why: each finger touched the leg at **one point**.
Pressed flat against a flat face, the simulator resolves the contact to a
single point, and a single point can stop sliding but cannot stop turning —
there is no spread for friction to act across. Any turning force at all, from
the part's own weight or from the arm swinging, turns the part.

So each finger now grips through four small pads, two at the fingertip and two
further up, standing 1.5 mm proud of the finger's face. Four pads touch at four
points, and the 2 cm between them is what resists the turn. Real grippers do
the same with ridged or rubber pads, and for a related reason: a flat plate on
a flat part really touches at a few high spots, not everywhere.

### Where to hold each part

The pads are the insurance; where the part is gripped is the first line of
defence. Both parts are held so that their weight does not try to turn them in
the fingers at all.

- **A leg is gripped round its middle.** Its weight then acts through the grip
  whether the leg is lying or standing.
- **The top is gripped at the middle of its upper edge.** Hanging there, its
  weight is straight below the grip. Held level, its weight tries to tip it
  forward about the edge, and that is resisted by the fingers pressing on
  both faces — top finger down at the edge, bottom finger up at the tips —
  which the two rows of pads give a lever to do.

Gripping a leg round its middle has a cost: stood up, the gripper is level and
only half a leg's length off the floor. That is why the legs are 13 to 16 cm
long, and why the gripper's body is slim on the side that faces the floor.

### Standing a leg up in stages

The first version picked a leg up and let the planner find its own way from
"lying, gripped from above" to "standing, gripped from the side" in one move.
The planner found ways — through the air a metre up, spinning the wrist —
and legs held only by friction were flung across the room.

Now it goes in short, slow stages, each doing one thing:

1. **Lift** the leg straight up, until its lower end will stay 8 cm off the
   floor as it turns — clear of any leg still lying nearby.
2. **Turn it upright where it is**, about its own centre, as a slow straight
   path in small steps (`turned_upright()`), so it neither swings out nor
   drops. The turn also swivels the leg so the gripper ends up pointing out
   along the arm's reach, with back towards the base as the second choice.
   Left to end pointing whichever way the leg happened to lie, the gripper
   can end level and square across the arm's reach, which is a *wrist
   singularity*: two of the wrist's axes line up and the arm loses a
   direction it can move in, so no path into that pose can be followed
   steadily.
3. **Carry** it, upright, to above its spot.
4. **Lower** it straight down to 3 mm off the floor, open the fingers all the
   way, and **lift straight up** off it. Up, not back: pulling back towards
   the base folds an arm already folded tight for the near legs until it
   cannot move.

The turn is allowed to stop short, and often does — the logs say "turned the
leg 96% of the way upright". MoveIt's straight-line service refuses the last
few percent of many turns without saying why, and that was not pinned down.
It does not need to be: the turn's job is only to get the leg roughly upright
round its own middle without swinging it about, and the carry that follows
goes to the exact standing pose anyway. Asking the turn to finish exactly
would fail the whole leg for the sake of its last few degrees.

All moves with a part in hand run at a tenth of the arm's speed.

### The hold

Once a part is in the gripper, part and tool move as one rigid body. That
relationship — the tool's pose written in the part's own frame — is recorded
at the moment of the grasp as the *hold*. Putting the part anywhere is then
one line: the tool goes to *where the part should be* times *the hold*. Standing
a leg up, turning it, and laying the top level are all just a new pose for the
part, and `grasps.py` never has to reason about fingers at all.

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
  different shape of arm, and is thrown away.
- Each joint is then set to whichever of its equivalent angles is nearest
  where it is now, so nothing turns further than it needs to.
- The answer is checked by working out where it really puts the tool, because
  the solver was once seen to report success with something else.

If none of that works, the planner may be given the bare pose — but not with
a part in hand. Then the move simply fails, and the next pose on the list is
tried.

On top of that, MoveIt is told the upper arm may never point below level
(`joint_limits.yaml`). The arm stands on the floor; lower than that, the elbow
is at the floor. MoveIt's model once judged a pose with the upper arm leaning
back past level to clear the floor by a few millimetres; in the simulator the
elbow ground into it and the shoulder stalled at full torque.

### Two pictures of the room

MoveIt keeps its own picture of the room to plan against, and in this setup
there are two copies of it: one in `move_group`, which straight-line moves
are checked against, and one inside the task node, which free moves are
planned against.

They do not keep each other up to date. The task node's copy listens for
changes on `/planning_scene`, which nothing publishes to — whatever the
`monitored_planning_scene_topic` setting's name suggests. For most of this
project's development every obstacle went only to `move_group`, and free
moves were being planned in an empty room: no wall, no legs, no floor. That
went unnoticed until the wrist was driven into a leg lying on the floor. Now
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

One frame taken 8 cm off where the camera had been sent is what exposed this.
Every point in it was misplaced by the same amount.

### Starting up: waiting rather than guessing

As in v1, the task waits for the planning scene, the controllers and the
camera themselves rather than trusting a delay. The controller spawners are
given five minutes, because on a cold start — the first run after an install —
the controller manager took well over a minute to appear, and a spawner that
gave up first left the arm with no joint states.

## How the measuring works

### The floor

All grey points from the survey go into a histogram of height. The floor is by
far the biggest level surface in view, so the height most points share is the
floor, refined to under a millimetre with the median of the points near it. It
is found once, from the survey, and used from then on: in close-up views a
part or the wall can fill more of the picture than the floor does.

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
set apart; `spec.py` keeps legs at least 4 cm from each other.

Two fits, for the two shapes in the room:

- **A box resting on the floor** — a lying leg, a standing leg, the wall, the
  finished table. Height is the highest point above the floor, because the
  camera always sees the top. Length and width come from the smallest
  rectangle enclosing the points seen from above, which *is* the footprint.
- **A flat board at any angle** — the leaning top. A plane is fitted to the
  points by principal component analysis: the direction they spread least in
  is the plane's normal. The strip of the top edge seen from above sits behind
  the face and drags that first plane off true, so the fit is repeated with
  only the points close to the plane, tightening the band each time, until
  only the face is left. In the face's own axes, the spread across it gives
  length and width, and how far the edge strip reaches behind it gives the
  thickness. The lean is the angle between "up the face" and straight up.

### Telling the parts apart

By shape alone. The table top is the broadest cluster whose plate fit is thin
— thinner than a quarter of its shorter side. A leg is a cluster at least
twice as long as it is thick. Anything coloured that is neither is reported
and left alone.

Grey clusters standing up off the floor are obstacles, with two filters: grey
points within 3 cm of a part are that part's own shadowed sides, and a grey
cluster that does not reach down to the floor is the arm catching sight of
itself.

### Accuracy

Against the sizes the simulator was told to spawn, from seed 1:

| | Measured | True |
| --- | --- | --- |
| Floor | 0.1 mm | 0 |
| Table top | 26.1 x 18.0 x 1.8 cm, leaning 20.6° | 26.04 x 17.98 x 1.78 cm, 20.5° |
| Legs | 13.1 x 3.3 x 3.4 cm | 13.09 x 3.34 x 3.34 cm |
| Wall | 44.4 x 9.7 x 7.2 cm | 45.0 x 10.0 x 7.3 cm |

The wall reads short because the top hides part of it. Nothing depends on the
wall being measured exactly; it is only there to be avoided.

## Planning the table

The table goes wherever the arm has room. Candidate centres are tried on
three rings, 52 to 62 cm from the base, every 5°. A candidate is kept only if
nothing the camera saw is within 12 cm of the table's footprint — room for the
gripper to reach in beside each leg. Of the ones kept, the one straight in
front of the arm at 58 cm wins. Much closer in than that, the arm has to fold
up so tightly to reach the near legs and the top's near edge that it runs out
of room to move. That preference is the arm's comfort, not
knowledge of the room: every candidate still has to be proved empty.

The table is turned so the edge the top was gripped by faces the arm. The legs
stand 1.2 cm in from the top's edges. They go in far ones first, so the gripper
never has to reach past a leg it has already stood up.

The top goes on where the legs *are*, not where they were planned: its centre
over the middle of the four as measured after each was put down, its underside
3 mm above the tallest.

## What breaks it

- **Parts touching.** Two legs lying against each other are one lump to the
  camera. The arm will not pick up a lump the wrong size, but it cannot
  separate them either.
- **A top too thick or a leg too fat for the gripper.** The gripper opens to
  8 cm and the task stops if a part needs more than 6.5.
- **Short legs.** Under about 12 cm, the wrist would be too close to the floor
  when a leg is stood up.
- **Parts outside the survey ring.** The survey sees the floor from about 35 cm
  to a metre out. A leg closer in than that is not found.
- **Parts too close to the arm.** The near legs and the top's near edge are
  reached with the gripper held level and the arm folded; move the table much
  closer than 52 cm and the arm cannot fold that far.
- **A board that is not leaning on something.** The top is expected to be
  standing on an edge. A top lying flat on the floor cannot be gripped by an
  edge at all, and a different grasp would be needed.
- **Coloured obstacles.** Anything saturated is taken for a part.

## What is checked, and how

`make test` runs about 180 tests that need no simulator:

- **Fitting.** A floor, lying and standing legs at many yaws, and leaning boards
  at several angles and sizes are turned into the points a camera would see —
  only the faces towards it — and the fits must give the numbers back.
- **Reading a room.** Rooms drawn by the real spawner, for several seeds, are
  turned into survey points, and `read_room()` must find the floor, the top to
  2 mm, all four legs, and the wall, and nothing else.
- **Pixels.** Edge pixels of a part are kept; a pixel hanging between a part and
  what is behind it is dropped; a floor seen at a slant is left alone.
- **Planning.** The legs stand inside the top's footprint, inset; far legs go
  first; the top ends up level at the right height; the site keeps clear of
  obstacles; no room is reported rather than squeezed.
- **Grasps.** The hold puts the tool back where it was; a leg is gripped over
  its centre of mass, turned upright about its own middle — ending pointing
  where asked — and stood upright;
  the top is gripped on its upper edge and laid level; the finger behind the
  top clears the wall in twenty rooms.
- **The world.** The same seed gives the same room; the top touches both the
  floor and the wall's corner; legs never touch; and nothing outside `world/`
  imports from it.

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
