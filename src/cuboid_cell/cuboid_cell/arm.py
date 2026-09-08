"""The arm, its gripper, and the fingertip contact sensor.

Planning goes through MoveIt: free moves are planned by OMPL and can bend
around the table and the other cuboids, while approach and retreat use
MoveIt's Cartesian path service so the fingertip travels in a straight line
along the face normal instead of arriving from wherever the planner fancied.
"""

from __future__ import annotations

import time

import numpy as np
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit.planning import MoveItPy
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .cell import WORLD_FRAME
from .transforms import make_pose

FINGER_JOINTS = ("left_finger_joint", "right_finger_joint")


def _describe(pose: Pose) -> str:
    p = pose.position
    return f"({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"


# A contact report older than this is treated as stale: the sensor only
# publishes while surfaces are actually touching.
CONTACT_FRESHNESS = 0.4


class MotionFailed(RuntimeError):
    """Raised when the arm could not carry out a requested move."""


class Arm:
    def __init__(
        self,
        node: Node,
        *,
        group: str = "ur_manipulator",
        tip_link: str = "tool0",
    ) -> None:
        self._node = node
        self._group = group
        self._tip_link = tip_link
        # MoveIt's own node picks up the robot description, the SRDF and the
        # planning pipelines from the parameters the launch file supplies.
        self._moveit = MoveItPy(node_name="cuboid_task_moveit")
        self._planner = self._moveit.get_planning_component(group)
        self._model = self._moveit.get_robot_model()

        self._cartesian = node.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._arm_controller = ActionClient(
            node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self._gripper = ActionClient(
            node, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory"
        )
        sensors = MutuallyExclusiveCallbackGroup()
        self._last_contact = 0.0
        node.create_subscription(
            Contacts, "/fingertip_contacts", self._on_contacts, 10, callback_group=sensors
        )
        self._finger_positions = (0.0, 0.0)
        node.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10, callback_group=sensors
        )

    # ---------------------------------------------------------------- moving

    def move_to_named(self, name: str) -> None:
        """Go to one of the poses named in the SRDF."""
        self._planner.set_start_state_to_current_state()
        self._planner.set_goal_state(configuration_name=name)
        self._run_plan(f"named pose '{name}'")

    def move_to_pose(self, position: np.ndarray, rotation: np.ndarray) -> None:
        """Plan a free move that puts the tip link at the given pose."""
        goal = PoseStamped()
        goal.header.frame_id = WORLD_FRAME
        goal.pose = make_pose(position, rotation)

        self._planner.set_start_state_to_current_state()
        self._planner.set_goal_state(pose_stamped_msg=goal, pose_link=self._tip_link)
        self._run_plan(f"pose {np.round(position, 3).tolist()}")

    def move_to_first_reachable(self, position: np.ndarray, rotations: list[np.ndarray]) -> np.ndarray:
        """Try each orientation in turn and keep the one that plans.

        Returns the orientation that worked, so the caller can carry on using
        it. Raises if none of them do.
        """
        for index, rotation in enumerate(rotations):
            try:
                self.move_to_pose(position, rotation)
                return rotation
            except MotionFailed:
                if index == len(rotations) - 1:
                    raise
        raise MotionFailed("no orientations were offered")

    def move_linear(
        self,
        waypoints: list[Pose],
        *,
        step: float = 0.005,
        min_fraction: float = 0.9,
        avoid_collisions: bool = True,
    ) -> float:
        """Move the tip link along straight lines through ``waypoints``.

        Returns the fraction of the path that was executed. With collision
        checking on, a path that would drive the fingers into the table comes
        back short instead of being run.
        """
        if not self._cartesian.wait_for_service(timeout_sec=10.0):
            raise MotionFailed("/compute_cartesian_path is not available")

        request = GetCartesianPath.Request()
        request.header.frame_id = WORLD_FRAME
        request.group_name = self._group
        request.link_name = self._tip_link
        request.waypoints = waypoints
        request.max_step = step
        request.avoid_collisions = avoid_collisions
        request.max_velocity_scaling_factor = 0.2
        request.max_acceleration_scaling_factor = 0.2

        response = self._cartesian.call(request)
        if response.fraction < min_fraction:
            raise MotionFailed(
                f"straight-line move to {_describe(waypoints[-1])} only solved "
                f"{response.fraction:.0%} of the way (MoveIt error {response.error_code.val})"
            )

        # Straight-line paths go to the controller directly. MoveIt has already
        # timed the trajectory, and its own executor takes a plan object rather
        # than the message this service hands back.
        self._send(self._arm_controller, response.solution.joint_trajectory, "the straight-line move")
        return float(response.fraction)

    def _run_plan(self, what: str) -> None:
        result = self._planner.plan()
        if not result:
            raise MotionFailed(f"no plan found for {what}")
        self._moveit.execute(result.trajectory, controllers=[])

    def _send(self, client: ActionClient, trajectory: JointTrajectory, what: str) -> None:
        """Run a joint trajectory on a controller and wait for it to finish."""
        if not client.wait_for_server(timeout_sec=10.0):
            raise MotionFailed(f"the controller for {what} is not available")
        result = client.send_goal(FollowJointTrajectory.Goal(trajectory=trajectory))
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise MotionFailed(f"the controller did not finish {what}")

    # --------------------------------------------------------------- gripper

    def set_gripper(self, opening: float, *, seconds: float = 1.0) -> None:
        """Command the total gap between the fingers, in metres."""
        half = max(0.0, opening) / 2.0

        point = JointTrajectoryPoint()
        point.positions = [half, half]
        point.time_from_start = DurationMsg(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))

        trajectory = JointTrajectory(joint_names=list(FINGER_JOINTS), points=[point])

        if not self._gripper.wait_for_server(timeout_sec=10.0):
            raise MotionFailed("the gripper controller is not available")
        # This blocks until the fingers stop. Closing on a box leaves them short
        # of where they were asked to go, so unlike an arm move, the
        # controller's own verdict on the result is not worth acting on.
        self._gripper.send_goal(FollowJointTrajectory.Goal(trajectory=trajectory))

    def _on_joint_states(self, msg: JointState) -> None:
        by_name = dict(zip(msg.name, msg.position, strict=False))
        if all(joint in by_name for joint in FINGER_JOINTS):
            self._finger_positions = tuple(by_name[joint] for joint in FINGER_JOINTS)

    @property
    def gripper_gap(self) -> float:
        """The gap the fingers are actually at, which is not always the one asked for.

        Closing on a box stops the fingers early, so this is how the arm can
        tell it has hold of something from how it can tell it closed on air.
        """
        return float(sum(self._finger_positions))

    # --------------------------------------------------------------- contact

    def _on_contacts(self, msg: Contacts) -> None:
        if msg.contacts:
            self._last_contact = time.monotonic()

    @property
    def in_contact(self) -> bool:
        """True while a fingertip is touching something."""
        return (time.monotonic() - self._last_contact) < CONTACT_FRESHNESS

    def wait_for_contact(self, timeout: float = 2.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.in_contact:
                return True
            time.sleep(0.02)
        return False
