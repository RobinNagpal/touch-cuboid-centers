# Pseudo code

A walk through the project at the level of files and functions. It says what
each thing is for and who calls it, and stops short of what happens inside a
function. For the reasoning behind the choices, read
[`IMPLEMENTATION_NOTES.md`](IMPLEMENTATION_NOTES.md).

## The two sides

The project is split down the middle.

**The simulator's side** is `table_assembly/world/`. It makes up this run's
room and tells Gazebo about it. It knows exactly how big everything is,
because it decides.

**The robot's side** is everything else. It knows where the arm is bolted
down and how the arm is built, and it finds everything else out with the
camera. Nothing on this side imports anything from `world/`, and
`test/test_world.py` checks that.

## The simulator's side: `table_assembly/world/`

- `spec.py` is the ranges a room is drawn from: where the top lies and how
  big it is, how tall its two stands are, how long the legs are and where
  they may stand.
- `spawn.py` draws one room. `random_room(seed)` picks the top, puts a stand
  under each end of it, and draws the four legs. `_standing_legs()` scatters
  the legs, redrawing any that would stand too close to another. `box_sdf()` turns a box into the
  simulator's model format using `part.sdf` or `stand.sdf`.
- `cell.sdf` is the fixed room: physics, lights, the floor, the window's view,
  and a marker line where the boxes go. `build.py`'s `build_world()` fills the
  marker in.
- `gz_bridge.yaml` lists the topics that cross from Gazebo to ROS: the clock,
  the camera, and the fingertip contact sensors. Nothing about the parts.
- `fastdds.xml` raises the message size limits so depth images arrive.
- `view.rviz` is the RViz layout, used only with `RVIZ=true`.

## The robot's side

### What the robot knows about itself: `table_assembly/arm/`

- `arm.urdf.xacro` is the whole robot: the UR5e from Universal Robots' own
  package, stood on the floor, with the gripper and the camera bolted on.
- `gripper.urdf.xacro` is the two-finger gripper, the four pads on each
  finger, and a contact sensor on each fingertip pad.
- `controllers.yaml` says which controllers run the joints.
- `dimensions.py` is every fixed number the robot has: where its base is,
  its ready posture, how far the fingertips and the camera are from the
  flange, and where the camera goes when it looks round the room.
- `motion.py` is the `Arm` class, which is everything the arm can be asked to
  do.
  - `move_to()` plans a free move to a pose. It first works out joint angles
    for the pose (`_solve()`), searching from the ready posture turned to
    face the pose, so the arm always reaches in the same shape, and taking
    each joint's equivalent angle nearest where it is. With a part in hand
    (`any_shape=False`) it will not let the planner pick any other shape.
    After the move it checks the arm really got there (`_check_arrival()`).
  - `can_reach()` says whether the arm could get to a pose without hitting
    anything, without moving. It is asked before a part is picked up.
    `wrist_side()` says which way the wrist would be flipped at a pose; a
    part in hand is never moved through a wrist flip.
  - `move_to_first()` tries a list of poses and stops at the first that works.
  - `move_linear()` moves in straight lines through one or more poses, and
    says how much of the way it got.
  - `looking_towards()` is the ready posture turned to face a direction.
  - `set_gripper()` moves the fingers and waits until they arrive or stop.
    `open_gripper()` also checks both fingers really opened.
  - `gripper_gap` says how far apart the fingers ended up, `in_contact` whether
    a fingertip is touching something, and `tool_pose()` where the tool really
    is.
- `camera/wrist_camera.urdf.xacro` is the RGB-D sensor.
- `camera/wrist_camera.py` is the `WristCamera` class. `capture()` returns a
  frame taken after it was asked for, together with the pose the camera was
  at when that frame was taken.

### Seeing: `table_assembly/perception/`

No ROS in here. Numpy arrays in, numpy arrays out.

- `pixels.py` goes from pixels to points. `colour_masks()` splits a picture
  into the pixels that show a part and the pixels that show everything else,
  `flying_pixels()` drops pixels whose range jumps away from their
  neighbours', and `back_project()` turns masked depth pixels into points in
  the room.
- `fitting.py` goes from points to shapes. `cluster()` groups points into one
  lump per object. `floor_height()` finds the floor. `fit_resting_box()` fits
  a box sitting on the floor — a standing leg, a stand, the
  finished table. `fit_plate()` fits a thin board at any angle — the top,
  lying on its stands. `surface_tilt()` says how far a surface is from level.
- `room.py` puts it together. `read_room()` takes the pooled points of
  several pictures and returns a `Room`: the floor height, the table top,
  the legs, the obstacles, and anything coloured that is neither. `leg_length()`,
  `leg_thickness()` and `is_standing()` answer questions about a leg.

### Planning: `table_assembly/assembly/`

No ROS in here either.

- `plan.py` decides the table. `SITE` is where the arm always builds it.
  `table_footprint()` says which way round the table goes: the top's long
  edge faces the arm. `plan_table()` returns a `TablePlan`: where the top
  ends up and where each leg stands, furthest from the arm first.
  `in_the_way()` lists anything seen on or beside that patch of floor.
- `grasps.py` says where the tool goes. `hold()` records how a part sits in the
  gripper, and `carried_tool_pose()` uses that to put the part anywhere.
  `leg_pick_poses()` grips a standing leg from above by its top end.
  `leg_place_pose()` stands it on a spot. `top_pick_poses()`
  grips the top from the side by the middle of its near edge.
  `level_top_pose()` holds it level over the legs. `carry_round()` carries a
  part round the base on an arc without tipping it. `shifted()` and
  `backed_off()` move a pose up or back.

### Shared pieces: `table_assembly/`

- `geometry.py` is the `Box` everything is described with: a centre, a full
  rotation, and three side lengths.
- `transforms.py` is rotations and poses as plain numpy.
- `messages.py` turns them into ROS messages and back.
- `scene.py` is the `PlanningSceneClient`, which tells MoveIt what is in the
  room: `set_floor()`, `set_objects()`, `attach()` and `detach()`. Every
  change goes to both of MoveIt's copies of the room — `move_group`'s and the
  task's own planner's.
- `task.py` is the workflow, and the only file that knows what order things
  happen in.
- `main.py` wires the pieces together and prints the result.

## The tools, and where they are configured

| Tool | What it does here | Where it is configured |
| --- | --- | --- |
| Gazebo | Simulates the room, the physics and the camera | `world/cell.sdf`, `world/spec.py` |
| ROS 2 | Carries messages between the simulator and the code | `world/gz_bridge.yaml`, `world/fastdds.xml` |
| ros2_control | Drives the joints | `arm/controllers.yaml` |
| MoveIt 2 | Plans the arm's movements | `src/table_assembly_moveit_config/config/` |
| pixi | Installs everything, at pinned versions | `pixi.toml`, `pixi.lock` |

## What happens when you run it

You type:

```
make run
```

The Makefile builds if it has to, then starts `launch/run.launch.py`.

**`run.launch.py`** includes `cell.launch.py`, starts MoveIt's `move_group`,
and ten seconds later starts the task. The task does not rely on ten seconds
being enough; it waits for what it needs.

**`cell.launch.py`**, in `setup()`:

1. `write_world()` asks `random_room()` for this run's room and
   `build_world()` for a finished world file.
2. It points Gazebo's search paths at the ROS install folders.
3. It starts the Gazebo server, `robot_state_publisher` on the robot from
   `robot_description()`, and spawns the robot.
4. It starts the Gazebo window, if one was asked for, as its own process.
5. It starts the bridge.
6. `_chain_spawners()` starts the three controllers one after another, and
   starts a spawner again if it fails.

**`main.py`**, in `main()`, waits for the arm's first joint states, because
MoveIt gives up if they are not there within ten seconds of it starting. Then
it builds `Arm`, `WristCamera`, `PlanningSceneClient`
(given the arm's own planning scene to keep up to date) and
`AssembleTableTask`, starts a thread that keeps ROS messages flowing, and calls
`task.run()`. A run that cannot finish ends with one line saying why.

**`task.py`**, in `AssembleTableTask.run()`:

1. **Wait** for the planning scene, the controllers and the camera.
2. **Survey.** `_survey()` sends the camera to eight views round the base and
   calls `_look()`, which for each view calls `_point_camera()`,
   `camera.capture()`, `colour_masks()` and `back_project()`, then hands all
   the points to `read_room()`. The floor goes into MoveIt with
   `scene.set_floor()` and everything else with `_publish()`.
3. **Measure the top.** `_measure_top()` looks at it from four close views
   and fits it again with `read_room()`.
4. **Plan.** `_plan()` calls `plan_table()` at `SITE` and stops if
   `in_the_way()` finds anything there.
5. **Legs.** For each spot, `_install_leg()`:
   - picks the nearest leg still standing where it started
     (`_waiting_legs()`), that is the size of one leg (`_is_one_leg()`);
   - `_inspect_leg()` measures it again from close above;
   - `_move_leg()` first pairs each of the four grips with the pose that
     would stand the leg on its spot (`leg_place_pose()`, `_reachable()`),
     and drops any grip that cannot reach it. Then it opens the fingers
     (`open_gripper()`), comes down over the top of the leg
     (`leg_pick_poses()`, `_reach()`), takes it out of MoveIt's scene
     (`_forget()`), grips it (`_grip()`), tells MoveIt it is held
     (`scene.attach()`), lifts it and carries it round over its spot
     (`_carry()`), lowers it (`_lower()`), lets go (`_release()`), and lifts
     straight up off it;
   - `_check_standing()` looks to see the leg standing on its spot.

   A failed attempt calls `_let_go()`, looks round the room again, and tries
   whichever standing leg it finds, up to three times.
6. **Top.** `_install_top()` works out where the top goes from where the legs
   really are, keeps only grips that can lay it there (`level_top_pose()`,
   `_reachable()`), opens the fingers, reaches in level over the top's near
   edge (`top_pick_poses()`, `_approach()`), grips it, lifts it off the
   stands, carries it round level (`_carry()`), lowers it onto the legs,
   lets go, and pulls back out from under it (`_pull_out()`).
7. **Check.** `_check_table()` looks down on the table, fits a box to what it
   sees with `fit_resting_box()`, and measures the top's tilt with
   `surface_tilt()`.
8. **Park** the arm, and return a `Result` for `main.py` to print.
