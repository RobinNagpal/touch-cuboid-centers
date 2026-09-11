"""Start the simulated cell: Gazebo, the robot, its controllers and the bridge.

The world file is written fresh on every start, because the room in it — the
stands, the table top and the legs — is random. ``seed`` makes any one room
repeatable.
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from table_assembly.world.build import build_world, read_template
from table_assembly.world.spawn import random_room

PACKAGE = "table_assembly"

# How many times a controller spawner is started before giving up on it.
SPAWNER_ATTEMPTS = 3


def robot_description(share: Path) -> str:
    """Run the xacro and hand back the finished URDF."""
    import xacro

    return xacro.process_file(
        str(share / "arm" / "arm.urdf.xacro"),
        mappings={"controllers_file": str(share / "arm" / "controllers.yaml")},
    ).toxml()


def write_world(share: Path, seed: int) -> str:
    """Put this run's stands and parts into the room and write out the world file."""
    sdf = build_world(read_template(share), random_room(seed))
    path = Path(tempfile.mkdtemp(prefix="table_assembly_")) / "cell.sdf"
    path.write_text(sdf)
    return str(path)


def _chain_spawners(names: list[str], attempts: int = SPAWNER_ATTEMPTS) -> list:
    """A spawner for the first controller, and what to do when it finishes.

    Each controller is only started once the one before it is running, and a
    spawner that fails is started again rather than skipped.

    The spawner's own timeouts assume a controller manager that is up in a few
    seconds. Inside Gazebo it only exists once the robot has been spawned and
    its hardware brought up, and on a cold start — the first run after an
    install, with nothing cached yet — that took over four minutes. Even told
    to wait that long, a spawner started that early has been seen to find the
    controller manager and then never hear back from it, while a fresh one
    started afterwards was answered at once. Skipping a controller that did
    not load leaves the arm with no joint states and the run hangs, so the
    next one is only started once this one has really succeeded.
    """
    name, rest = names[0], names[1:]
    spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            name,
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "300",
            "--service-call-timeout",
            "60",
        ],
    )

    def next_step(event, context):
        if event.returncode == 0:
            return _chain_spawners(rest) if rest else []
        if attempts > 1:
            return [LogInfo(msg=f"starting the {name} spawner again"), *_chain_spawners(names, attempts - 1)]
        return [LogInfo(msg=f"gave up on {name} after {SPAWNER_ATTEMPTS} attempts; the arm will not move")]

    return [spawner, RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=next_step))]


def setup(context, *args, **kwargs):
    share = Path(get_package_share_directory(PACKAGE))
    seed = int(LaunchConfiguration("seed").perform(context))
    gui = LaunchConfiguration("gui").perform(context).lower() in ("true", "1")

    world = write_world(share, seed)

    # The simulator always runs as a server on its own, and the window, when
    # there is one, is a second process that connects to it. That is not a
    # preference: on macOS `gz sim` refuses to be both in one process, because
    # the window has to own the main thread. Splitting them is fine everywhere,
    # so there is no per-platform branch here.
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": f"-s -r -v 2 --headless-rendering {world}"}.items(),
        ),
        # Given a few seconds so the server is up and has a scene to send it.
        *(
            [TimerAction(period=5.0, actions=[ExecuteProcess(cmd=["gz", "sim", "-g"], output="screen")])]
            if gui
            else []
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": ParameterValue(robot_description(share), value_type=str)},
                {"use_sim_time": True},
            ],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=["-topic", "robot_description", "-name", "assembly_arm"],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            parameters=[
                {"config_file": str(share / "world" / "gz_bridge.yaml")},
                {"use_sim_time": True},
            ],
        ),
        # One controller at a time. Three spawners racing each other into a
        # controller manager that is still starting up is enough to make one of
        # them try to configure a controller another has already activated.
        *_chain_spawners(["joint_state_broadcaster", "arm_controller", "gripper_controller"]),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", str(share / "world" / "view.rviz")],
            parameters=[{"use_sim_time": True}],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    # Gazebo looks for things on its own search paths, not on the ROS ones, so
    # both have to be pointed at every install prefix in the workspace:
    # package:// mesh paths for the UR5e's visuals, and the shared library that
    # lets ros2_control drive joints inside the simulator.
    prefixes = [p for p in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if p]
    resource_path = os.pathsep.join(os.path.join(p, "share") for p in prefixes)
    plugin_path = os.pathsep.join(os.path.join(p, "lib") for p in prefixes)

    return LaunchDescription(
        [
            DeclareLaunchArgument("seed", default_value="1", description="which random room to build"),
            DeclareLaunchArgument("gui", default_value="true", description="show the Gazebo window"),
            DeclareLaunchArgument("rviz", default_value="false", description="also open RViz"),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_path),
            # Applies to every node started below, because both ends of a topic
            # have to agree on how big a message may be.
            SetEnvironmentVariable(
                "FASTRTPS_DEFAULT_PROFILES_FILE",
                str(Path(get_package_share_directory(PACKAGE)) / "world" / "fastdds.xml"),
            ),
            OpaqueFunction(function=setup),
        ]
    )
