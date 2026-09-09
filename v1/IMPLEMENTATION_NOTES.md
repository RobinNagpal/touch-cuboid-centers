# Implementation notes

Why it is built this way, and what breaks it.

## When it goes wrong

Nothing here is quiet about failure, because a robot arm that quietly does the
wrong thing is worse than one that stops.

- A grasp that closed on nothing, and a box that worked loose part way across
  the table, are both caught by the fingertip contact sensors, at the moment
  they happen rather than later.
- A viewpoint the arm cannot reach is skipped, and the measurement is made from
  the ones it could reach.
- A cuboid the arm cannot manage is reported by name, the gripper is opened,
  and the run moves on to the next one. The box stays on the pending side, so
  the next pass tries it again.
- A touch that reached the face but felt nothing is reported as exactly that,
  not as a success.

The run ends with a list of what was measured, which face was chosen, and
whether the arm actually felt it.

## Choices

### The arm: UR5e

The problem asks for a 6-axis arm. The UR5e is the obvious pick: Universal
Robots publish `ur_description` themselves, so the link lengths, masses and
joint limits in this repo are the manufacturer's numbers rather than something
approximated. It has 850 mm of reach, which comfortably covers a table the arm
is standing on. Nothing in the code is UR-specific; `ur_type` in the xacro would
take `ur3e` or `ur10e` just as well, though the zones in `table/layout.py`
would need to shrink or grow to match the new reach.

### The gripper: written here, not borrowed

Universal Robots do not make a gripper, and the open-source models of the ones
people bolt onto a UR (the Robotiq 2F-85, for example) use *mimic* joints,
where one finger is defined as following the other. Simulated hardware has to
understand mimic joints for that to work, and support for them varies. Two
independent finger joints commanded to the same number needs nothing special
from anybody, so that is what this is.

The fingers are 12 cm long, which is longer than they need to be for gripping.
The reason is touching. To reach the side of a tall box the arm comes in from
above at an angle, and with short fingers the body of the gripper reaches the
top of the box before the fingertip reaches the face.

### The camera: on the wrist

The problem says the camera moves with the arm, which is also the more useful
arrangement: pointing the tool at something is the same as pointing the camera
at it, and the arm can walk the camera around a box to see sides that were
hidden from the first viewpoint. The cost is that a frame is only meaningful
together with the arm pose it was taken at, which is why the camera hands back
the image and the camera-to-world transform together rather than separately.

### The planner: MoveIt, mostly in-process

Free moves are planned by OMPL inside the task node, through MoveIt's Python
API. That keeps the planner's picture of the world and the code that changes it
in one process.

Two things still go through `move_group`, because MoveIt only offers them as
services: straight-line Cartesian paths, and edits to the planning scene. The
in-process planner is configured to watch the scene `move_group` publishes, so
adding the table in one place makes it visible in both.

### Gripping: how hard to squeeze

The fingers are told to close 4 mm narrower than the box was measured to be.
That number cost more experiments than any other in the project, because it is
squeezed from both sides.

It cannot be much smaller. The measured width is only good to a millimetre or
two, so on a box read slightly too wide, a 2 mm squeeze has the fingers stop
before they ever reach it. That failure looks exactly like a successful grasp
until the arm lifts an empty gripper.

It cannot be much larger either, and the reason is worth understanding. The
finger joints are position controlled: they are told where to be, not how hard
to push. Telling a finger to be somewhere the box already is does not squeeze
harder, it asks the physics engine to resolve two solid objects occupying the
same space, and the way it resolves that is by shooting the box out sideways.
A 6 mm squeeze lost boxes mid-carry for exactly this reason.

The joints also carry a 25 N effort limit. A wooden block needs a few newtons
to hold, so 25 N is generous, while a finger driven at full strength into
something it cannot squash is what flings it.

That leaves the question of how the arm knows it is holding anything, since
4 mm of finger travel is not a difference worth trusting. The fingertip contact
sensors answer it directly. They are read when the fingers close, and again
before the box is released at the far side. A grasp that missed and a box that
worked loose on the way over are both caught the moment they happen, and both
are reported, rather than being noticed later as a box that went missing.

### Starting up: waiting rather than guessing

A simulated robot cell does not come up all at once. Gazebo starts, the robot
is spawned into it, the controllers claim the joints, MoveIt loads its planners,
and the camera begins publishing — in that order, over tens of seconds.

The first version of this simply waited 25 seconds before starting the task,
which is the wrong shape of answer. A fixed delay is a guess about someone
else's machine. It wastes time when the guess is too long, and fails when it is
too short, and it is silently too short in exactly the case you notice last:
opening the Gazebo window slows startup down, so a delay tuned on headless runs
was not enough once there was something to watch.

So the task waits for the things themselves. Before it does anything it blocks
until the planning scene service answers, both controllers accept goals, and
the first camera frames have arrived. The launch file still gives the cell a
ten second head start, but only so the logs are not full of waiting; the run
is correct whether or not that head start was long enough.

## How the measuring works

### Finding the cuboids

The cuboids are the only saturated colours in the cell — the table, the floor
and the robot are all grey — so a threshold on saturation in HSV separates them
from everything else. That is the whole segmentation step. It is not a
general-purpose object detector, and it is not meant to be: the assumption is
stated up front, and it means the measurement is being tested rather than the
detector.

The mask is then eroded by one pixel. At the silhouette of an object, a depth
pixel is a blend of the object and whatever is behind it, and those blended
pixels back-project into mid-air, where they stretch the fitted box.

### Getting to 3D

Masked pixels are back-projected through the camera intrinsics into the
camera's optical frame, then through the camera pose into the world:

```
X = (u - cx) * depth / fx
Y = (v - cy) * depth / fy
Z = depth
```

`camera_to_world` comes from TF, for the frame `wrist_camera_optical_frame`.
Gazebo renders along a link's x axis and ROS projects images along z, so the
model carries two frames a quarter turn apart, and this is the one that matches
the images.

### Splitting the cloud up

Points are dropped into a 12 mm voxel grid and neighbouring occupied voxels are
flood-filled together. This is cheap — linear in the number of occupied voxels,
not quadratic in the number of points — and it is enough because the problem
states the cuboids are set apart. Boxes touching each other would come back as
one cluster and be measured as one large box.

### Fitting a box

Height is the highest point above the table. The camera always looks down on a
box, so it always sees the top face, and the top face is the height.

Length and width come from `cv2.minAreaRect` on the same points seen from
above. The smallest rectangle enclosing the top face *is* the box's footprint,
so this needs no iteration and no initial guess. Two details:

- The points are converted to millimetres first. OpenCV's convex hull maths is
  tuned for pixel-sized numbers, and in metres every coordinate in this cell
  sits inside a 1.0 box.
- The longer of the two sides is called the length, and the yaw is turned a
  quarter turn to match when they swap. Without that, a box would change its
  reported orientation as it rotated past 45 degrees.

The table height is taken from `table/layout.py` rather than fitted from the
data. The table is a fixed part of the cell — it is the surface the arm is
bolted to — so its height is known in the same way the arm's own dimensions
are known.

Measured against the sizes the simulator was told to spawn, the fits come out
within about a millimetre.

### Picking the face

A cuboid has six faces in three matching pairs, so there are three areas to
compare: length x width, length x height, width x height. The largest is taken,
with two rules:

- The face lying on the table is dropped. The arm cannot get underneath it.
- Opposite faces always tie on area, so the tie is broken by whichever face
  centre is nearer the robot's base. That picks the side the arm can approach
  without reaching over the box.

Areas are rounded to a square millimetre before being compared, so that
measurement noise cannot decide a tie between two faces that are really equal.

## Touching

The fingertip is driven 4 mm past where the face was measured to be. Stopping
exactly on the measured surface would mean that a face measured a millimetre
too far away never gets touched at all.

The last few centimetres are run with collision checking turned off. The
fingertip is being driven into the box deliberately, and the straight line
starts from a standoff pose that was itself reached with collision checking on,
so nothing else can be in the way.

Contact sensors on both fingertips report whether the arm actually felt the
face. That is what makes this a touch rather than a claim: without it, a
measurement that was 2 cm wrong would still be reported as a success.

## What breaks it

- **Cuboids touching or stacked.** The clustering merges them and the fit
  returns one box spanning both. This is the assumption the problem states, and
  the code does not try to work around it.
- **A cuboid too wide to grip.** The gripper opens to 80 mm, and the spawner
  will not place a box unless one of its horizontal sides is 65 mm or less.
  The 15 mm of margin is not spare capacity: a box almost as wide as the
  gripper opens has to be approached with its yaw right to within a degree or
  two, or a corner catches a finger. A real cell would need a plan for boxes it
  cannot pick up at all.
- **A viewpoint the arm cannot reach.** Skipped with a warning. The extra
  angles exist to fill in what one view misses, so losing one costs accuracy
  rather than the measurement. Losing all of them is an error.
- **A box that shifts when touched.** Pressing on the side of a box moves it a
  little, so its recorded position goes stale. That only matters for planning
  around it afterwards, and the boxes are far enough apart that it has not
  caused a collision.
- **A crowded done side.** Three cuboids is what the table comfortably holds,
  and those runs go through cleanly. Four fits, but the extra slots sit further
  round to the side of the arm, and the odd pick or carry there does not come
  off first time: a grasp closes a millimetre wide of a box, or a box works
  loose part way over. Nothing about that is silent. Each failure says what
  went wrong, the box stays on the pending side, and the next pass tries it
  again, up to twice as many attempts as there are cuboids. In the run this was
  measured on, one of the four took four attempts and all four finished.
- **Too many cuboids.** Above four, the spawner cannot place boxes far enough
  apart to be sure none of them touch, and it stops with an error before the
  run starts. That is deliberate. Two boxes touching would be grouped into one
  cloud and measured as a single large box, and a wrong answer delivered
  confidently is worse than a refusal.
- **Large ROS messages.** A 320x240 float depth image is around 300 kB, which
  is past the default DDS socket buffers. Without `world/fastdds.xml` the
  colour images arrive and the depth images mostly do not, and the failure is
  silent — the topic simply runs at a fraction of its rate.

## What is checked, and how

`make test` runs 50 tests that need no simulator, covering the two modules
where the logic lives:

- **Geometry.** That a cuboid has six faces in three matching pairs; that the
  three areas are the products of the side pairs; that a flat box is touched on
  top and a tall one on its side; that the face against the table is never
  chosen; that a tie goes to the face nearer the arm; and that yaw turns the
  side normals with the box.
- **Box fitting.** A cuboid of a known size and yaw is turned into the points a
  camera looking down on it would see — top face and two visible sides, nothing
  more — and the fit has to give the numbers back. Six yaws and three shapes,
  including the ones that make the two horizontal sides swap over. Two boxes
  set apart have to come back as two clusters and two independent measurements.
- **Transforms.** That a rotation survives the round trip through a quaternion,
  including the rotations whose quaternion has a near-zero scalar part; that a
  tool orientation really points where it was told; and that two viewpoints a
  few centimetres apart do not come out a quarter turn apart.

Beyond that, the only real test is running it. The measurements above were
checked against the sizes and positions the simulator was told to spawn, which
`gz model -m cuboid_0 -p` will print.

## Debugging

```
make cell                          # bring the cell up without running the task
ros2 topic hz /wrist_camera/depth_image   # 15 Hz, or the transport is dropping frames
ros2 control list_controllers             # all three should be active
gz model -m cuboid_0 -p                   # where a cuboid really is, to check a measurement
make run RVIZ=true                        # see what MoveIt is planning against
```

Gazebo prints `Unable to load Ogre Plugin ... Rendering will not be possible`
on macOS. Rendering does in fact work; the camera topics carry real images.

The simulator is always started as a server on its own, and `GUI=true` adds a
second process that connects to it. On macOS `gz sim` refuses to be both at
once, because the window has to own the main thread; it exits immediately with
a message saying so. Running them apart works on every platform, so there is no
per-platform branch in the launch file.

If nothing at all reaches the ROS side, check for leftover processes from an
earlier run. A simulator killed rather than shut down leaves its ROS endpoints
registered, and the publishers that are still alive stall waiting on them.
