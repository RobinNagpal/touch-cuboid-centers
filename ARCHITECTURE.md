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

Each module has one job, and they only depend downwards.

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

`geometry.py` and `perception.py` import no ROS at all, which is why the tests
can run them directly.

### The launch files

`launch/cell.launch.py` brings up the cell:

1. writes a world file with freshly generated cuboids in it;
2. points Gazebo's search paths at the ROS install prefixes;
3. starts Gazebo, `robot_state_publisher`, and spawns the robot into the world;
4. starts the ROS <-> Gazebo bridge;
5. starts the three controllers, one after another.

`launch/run.launch.py` adds MoveIt's `move_group` and the task node, and is what
`make run` calls.

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
`move_group`, because those are services rather than library calls.
