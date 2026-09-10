# Architecture

Two ROS 2 packages under `src/`.

```
v2/
├── Makefile                         the only entry point people use
├── pixi.toml                        every dependency, pinned in pixi.lock
└── src/
    ├── table_assembly/              the cell and the task that runs in it
    │   ├── launch/                  how it all gets started
    │   ├── test/                    tests that need no simulator
    │   └── table_assembly/
    │       ├── arm/                 the robot: model, controllers, motion, what it knows about itself
    │       │   └── camera/          the camera bolted to its wrist
    │       ├── perception/          camera pixels to floor, parts and obstacles (no ROS)
    │       ├── assembly/            where the table goes, and how to hold each part (no ROS)
    │       ├── world/               the simulator's side: the room, and this run's random parts
    │       ├── task.py              the workflow
    │       ├── scene.py             what MoveIt is told about the room
    │       ├── geometry.py          the Box every object is described with
    │       ├── transforms.py        rotations and poses, as plain numpy
    │       └── messages.py          numpy poses to ROS messages and back
    └── table_assembly_moveit_config/ what MoveIt needs to know about the robot
```

## The line down the middle

The most important boundary in the project is between `world/` and
everything else.

`world/` is the simulator's side. It decides how big the table top is, how far
it leans, where the wall is and where each leg lies, and it writes that into
the world file Gazebo loads. It knows the truth because it makes it up.

Everything else is the robot's side, and the robot is told nothing about the
room. It knows where it is bolted down and how its own gripper and camera are
built — that is `arm/dimensions.py`, and that is all. The floor height, the
wall, the top's length, width, thickness and lean, and every leg's size and
position it has to measure with the camera.

That boundary is enforced, not just intended. `test_world.py` parses every
module outside `world/` and fails if any of them imports from it. The only
place the two sides meet is the tests, which is where the robot's measurements
are compared with the simulator's truth.

## Why the folders are shaped like this

One folder per subject. A question about the arm is answered in `arm/`: the
model the simulator loads, the controller settings, the code that moves it,
and the numbers it knows about itself. `.xacro` and `.yaml` files sit next to
`.py` files, which is unusual in a ROS package, but `setup.py` installs them to
the places ROS expects, so nothing downstream notices.

`perception/` and `assembly/` are the two folders where the logic lives, and
they import no ROS at all. They are given numpy arrays and hand numpy arrays
back. That is why the tests can run them directly, in under a second, with no
simulator: the part of the code most likely to be wrong is the part that is
easiest to check.

## Why two packages

`table_assembly_moveit_config` holds only configuration: which joints are the
arm and which the gripper, which pairs of links may touch, the planner
settings, and one limit the robot model does not carry — the upper arm may
not point below level, because the arm stands on the floor. That is the
layout MoveIt's own tooling expects. It points back at
`table_assembly/arm/arm.urdf.xacro`, so the robot is described in exactly one
place and the simulator and the planner cannot disagree about it.

## table_assembly

### The arm

| File | What it owns |
| --- | --- |
| `arm/arm.urdf.xacro` | The whole robot. Pulls the UR5e in from `ur_description`, stands it on the floor, adds the gripper and camera, and declares which joints `ros2_control` may drive. |
| `arm/gripper.urdf.xacro` | The two-finger gripper, the four gripping pads on each finger, their friction, and a contact sensor on each fingertip pad. |
| `arm/controllers.yaml` | The three `ros2_control` controllers: joint states, the arm, the gripper. |
| `arm/dimensions.py` | Everything the robot knows in advance: where its base is, its ready posture, its tooling offsets, its comfortable reach, where it looks from when surveying. |
| `arm/motion.py` | The `Arm` class: working out joint angles in a consistent shape, planning, straight lines, checking it arrived, gripping and checking the fingers opened, feeling. |
| `arm/camera/wrist_camera.urdf.xacro` | The RGB-D sensor, and the two frames a ROS camera needs. |
| `arm/camera/wrist_camera.py` | The `WristCamera` class: a frame taken after it was asked for, plus the pose the camera was at when that frame was taken. Saves every frame when `TABLE_ASSEMBLY_VIEWS` is set. |

### Perception

| File | What it owns |
| --- | --- |
| `perception/pixels.py` | Pixels to points: the colour masks, the flying-pixel filter, back-projection. |
| `perception/fitting.py` | Points to shapes: clustering, the floor height, a box resting on the floor, a board at any angle, how level a surface is. |
| `perception/room.py` | Pooled points to a `Room`: the floor, the table top, the legs, the obstacles. Tells a top from a leg by shape alone. |

### Assembly

| File | What it owns |
| --- | --- |
| `assembly/plan.py` | Where to build the table, where each leg stands, and where the top ends up. |
| `assembly/grasps.py` | Tool poses to pick each part up, turn a leg upright, and put each part down, all through the *hold*. |

### The world

| File | What it owns |
| --- | --- |
| `world/cell.sdf` | Physics, lighting, the floor, and the window's opening view. One marker line says where the parts go. |
| `world/spec.py` | The ranges each run's room is drawn from. |
| `world/spawn.py` | Drawing this run's wall, top and legs, and writing them as models. |
| `world/part.sdf`, `world/wall.sdf` | One box model each, with blanks in it. |
| `world/build.py` | Putting the room and the parts into one world file. |
| `world/gz_bridge.yaml` | Every topic that crosses from Gazebo into ROS. None of them says anything about the parts. |
| `world/fastdds.xml` | Middleware buffer sizes, without which depth images do not arrive. |
| `world/view.rviz` | The RViz layout, used only with `RVIZ=true`. |

### The code

Each module has one job, and the dependencies only point one way.

| Module | Depends on |
| --- | --- |
| `transforms.py` | nothing |
| `geometry.py` | `transforms.py` |
| `arm/dimensions.py` | nothing |
| `perception/pixels.py` | nothing |
| `perception/fitting.py` | `geometry.py`, `transforms.py` |
| `perception/room.py` | `perception/fitting.py`, `geometry.py` |
| `assembly/plan.py` | `geometry.py` |
| `assembly/grasps.py` | `assembly/plan.py`, `arm/dimensions.py`, `geometry.py`, `transforms.py` |
| `messages.py` | `transforms.py` |
| `arm/camera/wrist_camera.py` | `perception/pixels.py`, `messages.py`, `arm/dimensions.py` |
| `arm/motion.py` | `messages.py`, `arm/dimensions.py` |
| `scene.py` | `geometry.py`, `messages.py`, `arm/dimensions.py` |
| `task.py` | all of the above |
| `main.py` | `task.py` |
| `world/*` | `transforms.py` — and nothing on the robot's side depends on `world/` |

Only one module knows what order things happen in. `task.py` holds the
workflow; everything below it offers a capability and has no opinion about
when it is used. `grasps.py` knows how to stand a leg up, not that the legs go
in before the top. So a change to the workflow is a change to one file.

### The launch files

`launch/cell.launch.py` brings up the cell:

1. draws this run's room and writes a world file with it in;
2. points Gazebo's search paths at the ROS install prefixes, because Gazebo
   does not know about ROS packages and would not find the arm's meshes;
3. starts the Gazebo server, `robot_state_publisher`, and spawns the robot;
4. starts the Gazebo window, if one was asked for, as a separate process;
5. starts the ROS <-> Gazebo bridge;
6. starts the three controllers, one after another, each willing to wait.

The window is a second process because macOS will not have a window share a
process with the simulator. The controllers go one at a time because three
spawners racing into a controller manager that is still waking up is enough
for one of them to trip over another. They are told to wait up to five
minutes, because on a cold start the controller manager took well over a
minute to appear, and a spawner that gives up leaves the arm without joint
states.

`launch/run.launch.py` adds MoveIt's `move_group` and the task node, and is
what `make run` calls. It gives the cell a ten second head start, but the task
does not depend on that being long enough — it waits for the controllers, the
planning scene service and the first camera frames itself.

## Nodes and topics

```
                      /wrist_camera/image
  Gazebo ──────────►  /wrist_camera/depth_image  ──────────►  assembly_task
   │      ros_gz      /wrist_camera/camera_info                 │      │
   │       bridge     /fingertip_contacts                       │      │
   │                  /clock                                    │      │
   │                                                            │      │
   │  ◄── /arm_controller/follow_joint_trajectory ──────────────┘      │
   │  ◄── /gripper_controller/follow_joint_trajectory ─────────────────┘
   │                                                            │
   └──► /joint_states ──► robot_state_publisher ──► /tf ────►  move_group
                                                                ▲      │
                          /compute_cartesian_path, /apply_planning_scene
```

Everything the robot learns about the room comes in on the three camera
topics. Nothing else crossing the bridge mentions the wall or the parts.

The task node holds MoveIt's planner inside itself, so free moves are planned
in process. Straight-line moves go through `move_group`, because MoveIt only
offers them as a service.

That means there are two copies of MoveIt's picture of the room: one in
`move_group`, which straight lines are checked against, and one in the task
node, which free moves are planned against. They do not keep each other up to
date. The in-process one listens for changes on `/planning_scene`, which
nothing publishes to, whatever `moveit_cpp.yaml`'s setting names suggest. So
`scene.py` makes every change twice: to `move_group` through its
`/apply_planning_scene` service, and to the in-process scene directly. Before
that was found, free moves were being planned in an empty room, and the wrist
was driven into a leg lying on the floor.
