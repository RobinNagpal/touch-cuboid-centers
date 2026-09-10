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

- `spec.py` is the ranges a room is drawn from: how long the wall is and how
  far away, how big the top can be and how far it leans, how long the legs
  are and where they may lie.
- `spawn.py` draws one room. `random_room(seed)` picks the wall, the top and
  the four legs. `_leaning_top()` works out where a board of that size has to
  be to stand on the floor and rest on the wall's top corner. `_lying_legs()`
  scatters the legs, redrawing any that would touch. `box_sdf()` turns a box
  into the simulator's model format using `part.sdf` or `wall.sdf`.
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
  flange, the patch of floor it builds on, and where the camera goes when it
  looks round the room.
- `motion.py` is the `Arm` class, which is everything the arm can be asked to
  do.
  - `move_to()` plans a free move to a pose. It first works out joint angles
    for the pose (`_solve()`), searching from the ready posture turned to
    face the pose, so the arm always reaches in the same shape, and taking
    each joint's equivalent angle nearest where it is. With a part in hand
    (`any_shape=False`) it will not let the planner pick any other shape.
    After the move it checks the arm really got there (`_check_arrival()`).
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
  a box sitting on the floor — a lying leg, a standing leg, the wall, the
  finished table. `fit_plate()` fits a thin board at any angle — the top,
  leaning on the wall. `surface_tilt()` says how far a surface is from level.
- `room.py` puts it together. `read_room()` takes the pooled points of
  several pictures and returns a `Room`: the floor height, the table top,
  the legs, the obstacles, and anything coloured that is neither. `leg_length()`,
  `leg_thickness()` and `is_standing()` answer questions about a leg.

### Planning: `table_assembly/assembly/`

No ROS in here either.

- `plan.py` decides the table. `gripped_axis()` and `table_footprint()` work
  out which edge of the top the arm will hold, and so which way round the
  table goes. `choose_site()` picks a patch of floor the table fits on with
  nothing near it. `plan_table()` returns a `TablePlan`: where the top ends
  up and where each leg stands, furthest from the arm first.
- `grasps.py` says where the tool goes. `hold()` records how a part sits in the
  gripper, and `carried_tool_pose()` uses that to put the part anywhere.
  `leg_pick_poses()` grips a lying leg round its middle. `turned_upright()`
  turns a held leg upright about its own middle, ending with the tool
  pointing whichever way it is asked. `standing_leg_poses()` stands
  it on a spot. `top_pick_poses()` grips the top by its upper edge.
  `level_top_poses()` holds it level over the legs. `shifted()` and
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
6. `_chain_spawners()` starts the three controllers one after another.

**`main.py`**, in `main()`, builds `Arm`, `WristCamera`, `PlanningSceneClient`
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
4. **Plan.** `_plan()` calls `choose_site()` and `plan_table()`.
5. **Legs.** For each spot, `_install_leg()`:
   - picks the nearest lying leg that is the size of one leg (`_is_one_leg()`);
   - `_inspect_leg()` measures it again from close above;
   - `_stand_up()` opens the fingers (`open_gripper()`), comes down over the
     leg (`leg_pick_poses()`, `_reach()`), takes it out of MoveIt's scene
     (`_forget()`), grips it (`_grip()`), tells MoveIt it is held
     (`scene.attach()`), lifts it, turns it upright where it is
     (`_turn_upright()`, which tries `turned_upright()` pointing out along
     the arm's reach, then back towards the base, then as it lay), carries it
     over its spot (`standing_leg_poses()`), lowers it (`_lower()`), lets go
     (`_release()`), and lifts straight up off it (`_lift_off()`);
   - `_check_standing()` looks to see the leg standing on its spot.

   A failed attempt calls `_let_go()`, looks round the room again, and tries
   whichever lying leg it finds, up to three times.
6. **Top.** `_install_top()` works out where the top goes from where the legs
   really are, opens the fingers, comes down onto the top's upper edge
   (`top_pick_poses()`, `_approach()`), grips it, lifts it off the wall, holds
   it level over the legs (`level_top_poses()`), lowers it, lets go, and
   pulls back out from under it (`_pull_out()`).
7. **Check.** `_check_table()` looks down on the table, fits a box to what it
   sees with `fit_resting_box()`, and measures the top's tilt with
   `surface_tilt()`.
8. **Park** the arm, and return a `Result` for `main.py` to print.
