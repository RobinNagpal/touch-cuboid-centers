# Pseudo code

A walk through the project at the level of files and functions. It says what
each thing is for and who calls it, and stops short of what happens inside a
function. For the reasoning behind the choices, read
[`IMPLEMENTATION_NOTES.md`](IMPLEMENTATION_NOTES.md).

## What is in the cell, and where each thing is described

The project involves a table, an arm, and a few cuboids. Each of those has one
folder, and that folder holds everything about it: the model the simulator
loads, the settings it needs, and the code that works with it.

**The table** is in `work_cell/table/`.

- `table.sdf` is the table itself — a top and four legs, grey on purpose.
- `layout.py` is the numbers: the table top is at 75 cm, the arm stands on it
  at one edge, and the top is split into two zones. Cuboids start in the
  pending zone and end up in the done zone. `zone_centre()` gives the middle of
  a zone, for pointing the camera at. `zone_slots()` gives the resting places
  in it, for putting boxes down.

**The arm** is in `work_cell/arm/`.

- `arm.urdf.xacro` is the whole robot. It pulls the UR5e in from Universal
  Robots' own package, bolts the gripper and the camera on, and lists which
  joints may be driven.
- `gripper.urdf.xacro` is the two-finger gripper, including the fingertip
  contact sensors.
- `controllers.yaml` says which controllers run the joints.
- `dimensions.py` is the numbers the model does not carry: how far the
  fingertips reach past the flange, where the camera sits, and the heights the
  arm works at.
- `motion.py` is the `Arm` class, which is everything the arm can be asked to
  do. `move_to_pose()` plans a free move. `move_linear()` moves in a straight
  line. `move_to_first_reachable()` tries a few orientations and keeps the one
  that works. `set_gripper()` opens and closes the fingers, `gripper_gap` says
  how far apart they ended up, and `in_contact` says whether a fingertip is
  touching something.

**The arm's camera** is in `work_cell/arm/camera/`, because it is bolted to the
arm and moves with it.

- `wrist_camera.urdf.xacro` is the RGB-D sensor.
- `wrist_camera.py` is the `WristCamera` class. `capture()` returns one frame
  together with where the camera was when it took it, which is the only way a
  picture from a moving camera means anything.

**The cuboids** are in `work_cell/cuboids/`.

- `cuboid.sdf` is one box, with blanks in it.
- `spec.py` is what a cuboid may be: the size range, the density, the colours,
  and how far apart two of them must sit.
- `spawn.py` invents them. `random_cuboids()` draws the sizes and positions for
  a run, and `cuboid_sdf()` fills in the blanks in `cuboid.sdf`.
- `perception.py` turns camera frames back into cuboids. `object_mask()` picks
  out the coloured pixels, `back_project()` turns them into points in the room,
  `cluster()` groups the points into one lump per box, and `fit_cuboid()` fits
  a box to a lump. `find_cuboids()` does the last two together.
- `geometry.py` is the maths of a box. `Cuboid.faces()` gives all six faces,
  `distinct_face_areas()` gives the three areas that are actually different,
  and `largest_touchable_face()` picks the one to touch.

**The world** is in `work_cell/world/`.

- `cell.sdf` is the room: physics, lighting, the ground, and the window's
  opening view. It has two marker lines where the table and the cuboids go.
- `build.py` puts the pieces together. `read_parts()` reads the room and the
  table off disk, and `build_world()` drops them and the cuboids into one
  finished world file.
- `gz_bridge.yaml` lists every topic that has to cross from the simulator into
  ROS.
- `fastdds.xml` raises the message size limits, without which depth images do
  not arrive.
- `view.rviz` is the RViz layout, used only when `RVIZ=true`.

Three files sit outside any one subject, because they are about the job rather
than about a thing in the room.

- `work_cell/task.py` is the workflow — the only file that knows what order
  things happen in.
- `work_cell/scene.py` tells MoveIt what is in the room, so it can plan around
  it.
- `work_cell/transforms.py` is shared maths: turning a direction the arm should
  point into an orientation it can be given.

## The tools, and where they are configured

| Tool | What it does here | Where it is configured |
| --- | --- | --- |
| Gazebo | Simulates the room, the physics and the camera | `work_cell/world/cell.sdf` |
| ROS 2 | Carries messages between the simulator and the code | `work_cell/world/gz_bridge.yaml`, `work_cell/world/fastdds.xml` |
| ros2_control | Drives the joints | `work_cell/arm/controllers.yaml` |
| MoveIt 2 | Plans the arm's movements | `src/work_cell_moveit_config/config/` |
| pixi | Installs everything, at pinned versions | `pixi.toml`, `pixi.lock` |

`work_cell_moveit_config` is the second package. It holds only what MoveIt
needs to be told about the robot: which joints are the arm and which are the
gripper (`work_cell.srdf`), which pairs of links are allowed to touch, and the
planner settings. It points back at `work_cell/arm/arm.urdf.xacro`, so the
robot is described in exactly one place and the simulator and the planner
cannot disagree about it.

## What happens when you run it

You type:

```
make run
```

The Makefile builds if it has to, then starts `launch/run.launch.py`.

**`run.launch.py`** starts three things and gets out of the way.

1. It includes `launch/cell.launch.py`, which brings the cell up.
2. It starts MoveIt's `move_group`, the planner other processes can call.
3. It starts the task, ten seconds later. The task does not rely on ten seconds
   being enough; it checks for itself.

**`cell.launch.py`** brings up the cell, in `setup()`.

1. `write_world()` asks `random_cuboids()` for this run's boxes and
   `build_world()` for a finished world file, and writes it to a temporary path.
2. It points Gazebo's search paths at the ROS install folders, so Gazebo can
   find the arm's meshes.
3. It starts the Gazebo server on that world file, and `robot_state_publisher`
   on the robot from `robot_description()`, which runs the xacro.
4. It spawns the robot into the running world.
5. If a window was asked for, it starts one as a separate process.
6. It starts the bridge between Gazebo and ROS.
7. `_chain_spawners()` starts the three controllers, each one waiting for the
   one before it to finish.

**`work_cell/main.py`**, in `main()`, is the task process.

1. It builds the four objects the job needs: `Arm`, `WristCamera`,
   `PlanningSceneClient` and `TouchCuboidsTask`.
2. It starts a thread that keeps ROS messages flowing, because the workflow
   itself spends most of its time blocked waiting for them.
3. It calls `task.run()`, and prints the summary at the end.

**`work_cell/task.py`**, in `TouchCuboidsTask.run()`, is the job itself.

1. It waits for the cell: `scene.wait_until_ready()`, `arm.wait_until_ready()`
   and `camera.wait_until_ready()`.
2. `scene.add_table()` tells MoveIt the table is there, so it never plans a
   path through it.
3. `_survey(PENDING_ZONE)` looks at the near half of the table and returns the
   cuboids on it. Then, for each one:
   - `_relocate()` carries it to the done side;
   - `_inspect()` measures it again, now that it is on its own;
   - `largest_touchable_face()` picks the face;
   - `_touch()` presses the fingertips into the middle of it.
4. It ends by parking the arm and returning what it found.

The four steps in the middle are worth one line each.

**`_survey()` and `_inspect()`** both call `_look()`, which is where a
measurement is actually made. For each viewpoint it calls `_point_camera()` to
put the camera there, `camera.capture()` to take a frame, then
`object_mask()` and `back_project()` from `perception.py` to turn that frame
into points in the room. It pools the points from every viewpoint and hands
them to `find_cuboids()`, which returns one measured box per lump of points.

**`_relocate()`** does the pick and place. It works out a grasp orientation
with `_grasp_rotation()`, opens the fingers with `arm.set_gripper()`, moves
above the box, comes down with `_reach()`, closes on it, and checks
`arm.wait_for_contact()` to be sure it really has it. It tells MoveIt the box
is in the gripper with `scene.attach()`, carries it over, sets it down, opens
the fingers, and calls `scene.detach()`.

**`_touch()`** works out where the wrist has to be with `_approach_direction()`,
closes the fingers so they act as one blunt tip, moves to a standoff pose, then
runs `arm.move_linear()` straight in until the fingertips are pressed into the
face. `arm.wait_for_contact()` says whether the arm actually felt it.

**`_publish()`** keeps MoveIt's picture of the room up to date after every
change, by calling `scene.set_cuboids()` with the boxes that are on the table
now.
