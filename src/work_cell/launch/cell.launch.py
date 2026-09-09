"""Start the simulated cell: Gazebo, the robot, its controllers and the bridge.

The world file is written fresh on every start, because the cuboids in it are
random. ``seed`` makes any one arrangement repeatable.
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
from work_cell.cuboids.spawn import random_cuboids
from work_cell.world.build import build_world, read_parts

PACKAGE = "work_cell"


def robot_description(share: Path) -> str:
    """Run the xacro and hand back the finished URDF."""
    import xacro

    return xacro.process_file(
        str(share / "arm" / "arm.urdf.xacro"),
        mappings={"controllers_file": str(share / "arm" / "controllers.yaml")},
    ).toxml()


def write_world(share: Path, count: int, seed: int) -> str:
    """Assemble the room, the table and this run's cuboids into one world file."""
    world_template, table = read_parts(share)
    sdf = build_world(world_template, table, random_cuboids(count, seed))
    path = Path(tempfile.mkdtemp(prefix="work_cell_")) / "cell.sdf"
    path.write_text(sdf)
    return str(path)


def _chain_spawners(*names: str) -> list:
    """Spawner nodes, each one starting only once the one before it has finished."""
    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[name, "--controller-manager", "/controller_manager"],
        )
        for name in names
    ]
    actions = [spawners[0]]
    for previous, following in zip(spawners, spawners[1:], strict=False):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=previous, on_exit=[following])))
    return actions


def setup(context, *args, **kwargs):
    share = Path(get_package_share_directory(PACKAGE))
    cuboids = int(LaunchConfiguration("cuboids").perform(context))
    seed = int(LaunchConfiguration("seed").perform(context))
    gui = LaunchConfiguration("gui").perform(context).lower() in ("true", "1")

    world = write_world(share, cuboids, seed)

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
            arguments=["-topic", "robot_description", "-name", "work_cell"],
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
        *_chain_spawners("joint_state_broadcaster", "arm_controller", "gripper_controller"),
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
            DeclareLaunchArgument(
                "cuboids", default_value="3", description="how many cuboids to put on the table"
            ),
            DeclareLaunchArgument("seed", default_value="1", description="which random arrangement to use"),
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
