# Architecture

Two ROS 2 packages under `src/`.

```
touch-cuboid-centers/
├── Makefile                     the only entry point people use
├── pixi.toml                    every dependency, pinned in pixi.lock
└── src/
    ├── cuboid_cell/             the work cell and the task that runs in it
    │   ├── urdf/                the robot: arm, gripper, camera
    │   ├── worlds/cell.sdf      the room, the light and the table
    │   ├── config/              controllers, the ROS <-> Gazebo bridge, RViz
    │   ├── launch/              how it all gets started
    │   ├── cuboid_cell/         the Python that does the work
    │   └── test/                tests that need no simulator
    └── cuboid_moveit_config/    what MoveIt needs to know about the robot
```

## Why two packages

`cuboid_moveit_config` holds only configuration: a semantic description of the
robot (which joints are the arm, which are the gripper, which pairs of links
are allowed to touch) and the planner settings. That is the layout MoveIt's own
tooling expects, and keeping it separate means the robot itself is described in
exactly one place — `cuboid_cell/urdf/cell.urdf.xacro` — and both the simulator
and MoveIt read that same file.

## cuboid_cell

### The model

| File | What it owns |
| --- | --- |
| `urdf/cell.urdf.xacro` | The whole robot. Pulls the UR5e in from `ur_description`, adds the gripper and camera, and declares which joints `ros2_control` may drive. |
| `urdf/gripper.urdf.xacro` | The two-finger gripper, its friction, and the fingertip contact sensors. |
| `urdf/wrist_camera.urdf.xacro` | The RGB-D camera and the two frames a ROS camera needs. |
| `worlds/cell.sdf` | Physics, lighting, the ground and the table. Has a `<!-- CUBOIDS -->` line that the launch fills in. |

### The configuration

| File | What it owns |
| --- | --- |
| `config/controllers.yaml` | The three `ros2_control` controllers: joint states, the arm, the gripper. |
| `config/gz_bridge.yaml` | Every topic that has to cross from Gazebo into ROS. |
| `config/fastdds.xml` | Middleware buffer sizes, without which depth images do not arrive. |
| `config/view.rviz` | The RViz layout, used only with `RVIZ=true`. |

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

| Module | What it owns | Depends on |
| --- | --- | --- |
| `cell.py` | Every number describing the cell: table height, the two zones, gripper offsets, working heights. | nothing |
| `geometry.py` | A cuboid, its six faces, their areas, and which face to touch. Pure maths. | `cell.py` |
| `perception.py` | RGB-D frames to cuboids: masking, back-projection, clustering, box fitting. | `geometry.py` |
| `transforms.py` | Conversions between ROS poses and 4x4 matrices, and building a tool orientation from a direction. | nothing |
| `world.py` | Generating the random cuboids and writing them into the world file. | `cell.py` |
| `camera.py` | The wrist camera as one object: a frame plus the pose the camera was at. | `perception.py`, `transforms.py` |
| `arm.py` | The arm, the gripper and the contact sensors. Planning, straight-line moves, gripping, feeling. | `transforms.py` |
| `scene.py` | What MoveIt is told about the table, the cuboids, and the box currently in the gripper. | `geometry.py`, `transforms.py` |
| `task.py` | The workflow. The only module that knows the order things happen in. | all of the above |
| `main.py` | Wiring, and the executor thread the workflow blocks against. | `task.py` |

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
