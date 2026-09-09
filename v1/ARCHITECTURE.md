# Architecture

Two ROS 2 packages under `src/`.

```
touch-cuboid-centers/
├── Makefile                     the only entry point people use
├── pixi.toml                    every dependency, pinned in pixi.lock
└── src/
    ├── work_cell/               the cell and the task that runs in it
    │   ├── launch/              how it all gets started
    │   ├── test/                tests that need no simulator
    │   └── work_cell/
    │       ├── arm/             the arm: model, controllers, motion
    │       │   └── camera/      the camera bolted to its wrist
    │       ├── table/           the table, and the two halves of it
    │       ├── cuboids/         the boxes: inventing, measuring, geometry
    │       ├── world/           the room, and the bridge into ROS
    │       ├── task.py          the workflow
    │       ├── scene.py         what MoveIt is told about the room
    │       └── transforms.py    shared maths
    └── work_cell_moveit_config/ what MoveIt needs to know about the robot
```

## Why the folders are shaped like this

One folder per thing in the room. `arm/` holds everything about the arm — the
model the simulator loads, the controller settings, the code that moves it —
and `table/`, `cuboids/` and `world/` do the same for theirs.

The alternative, and the more common ROS layout, is one folder per file type:
all the models together, all the YAML together, all the Python together.
That reads well when you already know the project, because you know what kind
of file you are looking for. It reads badly when you do not, because answering
"how is the gripper set up?" means opening three folders and knowing in advance
which three.

Grouping by subject means a question about the arm is answered in one place. It
costs one thing: `.xacro` and `.yaml` files sit next to `.py` files, which is
unusual to look at. `setup.py` installs them to the same places ROS expects, so
nothing downstream notices.



## Why two packages

`work_cell_moveit_config` holds only configuration: a semantic description of the
robot (which joints are the arm, which are the gripper, which pairs of links
are allowed to touch) and the planner settings. That is the layout MoveIt's own
tooling expects, and keeping it separate means the robot itself is described in
exactly one place — `work_cell/arm/arm.urdf.xacro` — and both the simulator
and MoveIt read that same file.

## work_cell

### The arm

| File | What it owns |
| --- | --- |
| `arm/arm.urdf.xacro` | The whole robot. Pulls the UR5e in from `ur_description`, adds the gripper and camera, and declares which joints `ros2_control` may drive. |
| `arm/gripper.urdf.xacro` | The two-finger gripper, its friction, and the fingertip contact sensors. |
| `arm/controllers.yaml` | The three `ros2_control` controllers: joint states, the arm, the gripper. |
| `arm/dimensions.py` | The measurements the model does not carry: fingertip reach, camera offset, working heights. |
| `arm/motion.py` | The `Arm` class: planning, straight lines, gripping, feeling. |
| `arm/camera/wrist_camera.urdf.xacro` | The RGB-D sensor, and the two frames a ROS camera needs. |
| `arm/camera/wrist_camera.py` | The `WristCamera` class: a frame plus the pose the camera was at. |

### The table

| File | What it owns |
| --- | --- |
| `table/table.sdf` | The table itself, spliced into the world at run time. |
| `table/layout.py` | Where the table is, and the two zones on it. The frame everything else is measured in. |

### The cuboids

| File | What it owns |
| --- | --- |
| `cuboids/cuboid.sdf` | One box, with blanks in it. |
| `cuboids/spec.py` | What a cuboid may be: size range, density, colours, spacing. |
| `cuboids/spawn.py` | Inventing this run's boxes and writing them as models. |
| `cuboids/perception.py` | RGB-D frames to cuboids: masking, back-projection, clustering, box fitting. |
| `cuboids/geometry.py` | A cuboid, its six faces, their areas, and which face to touch. Pure maths. |

### The world

| File | What it owns |
| --- | --- |
| `world/cell.sdf` | Physics, lighting, the ground, and the window's opening view. Two marker lines say where the table and the cuboids go. |
| `world/build.py` | Assembling the room, the table and the cuboids into one world file. |
| `world/gz_bridge.yaml` | Every topic that has to cross from Gazebo into ROS. |
| `world/fastdds.xml` | Middleware buffer sizes, without which depth images do not arrive. |
| `world/view.rviz` | The RViz layout, used only with `RVIZ=true`. |

### The code

Each module has one job, and the dependencies only point downwards: nothing in
the table below depends on anything beneath it.

That ordering is not tidiness for its own sake. It buys two specific things.

The first is testing. The modules at the top — the cuboid geometry and the box
fitting — import no ROS at all. They are given numbers and hand numbers back.
That is why 50 tests can run in under a second with no simulator, and why the
part of the code most likely to be wrong is also the part that is easiest to
check.

The second is that only one module knows what order things happen in. `task.py`
holds the workflow; everything below it offers a capability and has no opinion
about when it is used. `arm.py` knows how to move the arm somewhere, not that a
survey comes before a grasp. So a change to the workflow is a change to one
file.

| Module | Depends on |
| --- | --- |
| `table/layout.py` | nothing |
| `arm/dimensions.py` | nothing |
| `cuboids/spec.py` | nothing |
| `cuboids/geometry.py` | nothing |
| `cuboids/perception.py` | `cuboids/geometry.py` |
| `cuboids/spawn.py` | `cuboids/spec.py`, `arm/dimensions.py`, `table/layout.py` |
| `world/build.py` | `cuboids/spawn.py` |
| `transforms.py` | nothing |
| `arm/camera/wrist_camera.py` | `cuboids/perception.py`, `transforms.py`, `table/layout.py` |
| `arm/motion.py` | `transforms.py`, `table/layout.py` |
| `scene.py` | `cuboids/geometry.py`, `transforms.py`, `table/layout.py` |
| `task.py` | all of the above |
| `main.py` | `task.py` |

The one arrow that looks backwards is the camera depending on the cuboids: it
hands back the camera's intrinsics, and those are defined in `perception.py`
because that is what consumes them. Moving them into the camera would drag ROS
into `perception.py` and cost the tests their simulator-free run, which is a
worse trade than one odd-looking arrow.

### The launch files

`launch/cell.launch.py` brings up the cell:

1. writes a world file with freshly generated cuboids in it;
2. points Gazebo's search paths at the ROS install prefixes, because Gazebo
   does not know about ROS packages and would not find the arm's meshes;
3. starts the Gazebo server, `robot_state_publisher`, and spawns the robot;
4. starts the Gazebo window, if one was asked for, as a separate process;
5. starts the ROS <-> Gazebo bridge;
6. starts the three controllers, one after another.

Two of those steps are ordered the way they are for a reason.

The window is a second process rather than part of the server because macOS
will not have it any other way: a window has to own the main thread, and
`gz sim` exits rather than try. Running them apart works everywhere, so there
is no per-platform branch.

The controllers are started one after another rather than all at once because
three spawners racing into a controller manager that is still waking up is
enough for one of them to try to configure a controller another has already
started.

`launch/run.launch.py` adds MoveIt's `move_group` and the task node, and is what
`make run` calls. It gives the cell a ten second head start, but the task does
not depend on that being long enough — it waits for the controllers, the
planning scene service and the first camera frames itself.

## Nodes and topics

```
                      /wrist_camera/image
  Gazebo ──────────►  /wrist_camera/depth_image  ──────────►  cuboid_task
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

The task node holds MoveIt's planner inside itself, so free moves are planned
in process. Straight-line moves and changes to the planning scene go through
`move_group`, because MoveIt only offers those as services.

That split means there are two copies of the planning scene in play, which
would be a bug waiting to happen. It is avoided by making one of them the
authority: the in-process planner is configured to watch the scene `move_group`
publishes, so the table is added in one place and both see it.
