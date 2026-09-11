"""The arm, its gripper, and the fingertip contact sensors.

Planning goes through MoveIt: free moves are planned by OMPL and can bend
around the stands and the parts, while approaches and retreats use MoveIt's
Cartesian path service so the gripper travels in a straight line onto a part
instead of arriving from wherever the planner fancied.

Poses are 4x4 matrices for tool0 in the world frame.
"""

from __future__ import annotations

import copy
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ..messages import make_pose, transform_to_matrix
from .dimensions import READY_JOINTS, WORLD_FRAME

FINGER_JOINTS = ("left_finger_joint", "right_finger_joint")

# The arm joints, by position in the planning group, that can turn a full
# circle either way. The elbow is the one that cannot: it stops at half a turn.
FULL_TURN_JOINTS = (0, 1, 3, 4, 5)
ELBOW = 2

# How far inside a full turn either way those joints are kept. Always taking
# the angle nearest where a joint already is lets it creep, move by move, to
# the end of its travel, and then the next straight line that needs it to go
# on turning the same way stops part way. After four legs carried round the
# base, the line in to the table top's edge stopped a third of the way for
# that alone, with the last wrist joint at exactly a full turn.
END_STOP_MARGIN = 0.5  # radians
WRIST_2 = 4

# A contact report older than this is treated as stale: the sensor only
# publishes while surfaces are actually touching.
CONTACT_FRESHNESS = 0.4

# How close the tool has to end up to where it was sent. A move is only
# finished when the arm is really there, not when the controller says so.
ARRIVAL_TOLERANCE = 0.005  # metres
ARRIVAL_ANGLE = 0.03  # radians


def _describe(pose: np.ndarray) -> str:
    return str(np.round(pose[:3, 3], 3).tolist())


def _wrist_side(joints) -> float:
    """Which way the wrist is flipped: the side of the arm the middle wrist joint bends to."""
    return 1.0 if np.sin(joints[WRIST_2]) >= 0.0 else -1.0


class MotionFailed(RuntimeError):
    """Raised when the arm could not carry out a requested move."""


class Arm:
    def __init__(self, node: Node, *, group: str = "ur_manipulator", tip_link: str = "tool0") -> None:
        self._node = node
        self._group = group
        self._tip_link = tip_link
        # MoveIt's own node picks up the robot description, the SRDF and the
        # planning pipelines from the parameters the launch file supplies.
        self._moveit = MoveItPy(node_name="assembly_task_moveit")
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
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, node)
        self._missed = ""

    # --------------------------------------------------------------- startup

    def wait_until_ready(self, timeout: float = 120.0) -> None:
        """Block until the controllers and the Cartesian service are up.

        How long the simulator needs to get there depends on the machine it is
        running on, so this waits for the things themselves rather than for a
        fixed delay that is generous on one machine and short on the next.
        """
        deadline = time.monotonic() + timeout
        for what, wait in (
            ("the arm controller", self._arm_controller.wait_for_server),
            ("the gripper controller", self._gripper.wait_for_server),
            ("/compute_cartesian_path", self._cartesian.wait_for_service),
        ):
            if not wait(timeout_sec=max(0.0, deadline - time.monotonic())):
                raise MotionFailed(f"{what} did not come up within {timeout:.0f}s")

    # ---------------------------------------------------------------- moving

    def move_to(
        self,
        pose: np.ndarray,
        *,
        near: np.ndarray | None = None,
        speed: float | None = None,
        any_shape: bool = True,
        wrist: float | None = None,
    ) -> None:
        """Plan a free move that puts the tip link at ``pose``.

        A 6-axis arm can reach most poses in up to eight different ways —
        elbow up or down, shoulder forward or back, wrist flipped or not — and
        left to itself the planner picks one of them at random. So the joint
        angles are worked out here first, starting the search from ``near``,
        which finds the way of reaching the pose that is closest to it.

        By default ``near`` is the ready posture — elbow up, upper arm
        upright — turned on the base to face the pose. Starting every search
        from the same posture keeps the arm in the same shape from move to
        move. Starting it from wherever the arm happens to be lets one odd
        posture breed the next: the arm once ended up leaning its upper arm
        back past horizontal and ground its elbow into the floor.

        If that search fails and ``any_shape`` is set, the planner is given the
        pose and left to find any way there. With a part in hand that is not
        allowed: a random way of reaching the pose can mean the whole arm
        flipping over on the way, and a part held by friction does not survive
        that. Then the move fails instead, and the caller tries another pose.

        ``wrist`` pins which way the wrist is flipped (see `wrist_side()`).
        Without a part in hand it is free. With one (``any_shape`` off) it
        stays the way it is now: flipping the wrist over means turning the
        part through half a circle on the way.

        ``speed`` scales the arm's joint speeds and accelerations, for moves
        carrying something held only by friction.
        """
        self._planner.set_start_state_to_current_state()
        if wrist is None and not any_shape:
            wrist = self.current_wrist_side()
        target = self._solve(pose, near, wrist, explain=True)
        result = None
        if target is not None:
            self._planner.set_goal_state(robot_state=target)
            result = self._plan(speed)
        if not result and any_shape:
            goal = PoseStamped()
            goal.header.frame_id = WORLD_FRAME
            goal.pose = make_pose(pose[:3, 3], pose[:3, :3])
            self._planner.set_goal_state(pose_stamped_msg=goal, pose_link=self._tip_link)
            result = self._plan(speed)
        if not result:
            why = "the planner found no way there" if target is not None else self._missed
            raise MotionFailed(f"no plan found to {_describe(pose)}: {why}")
        self._moveit.execute(result.trajectory, controllers=[])
        self._check_arrival(pose)

    def _plan(self, speed: float | None):
        if speed is None:
            return self._planner.plan()
        # The same request as config/moveit_cpp.yaml sets up, only slower.
        # Spelled out because the parameter lookup behind this class reads
        # from a different prefix than that file uses, and comes back empty.
        parameters = PlanRequestParameters(self._moveit, "")
        parameters.planning_pipeline = "ompl"
        parameters.planner_id = "RRTConnectkConfigDefault"
        parameters.planning_attempts = 6
        parameters.planning_time = 10.0
        parameters.max_velocity_scaling_factor = speed
        parameters.max_acceleration_scaling_factor = speed
        return self._planner.plan(parameters)

    def _solve(
        self, pose: np.ndarray, near: np.ndarray | None, wrist: float | None = None, *, explain: bool = False
    ) -> RobotState | None:
        """Joint angles that put the tip link at ``pose``, found starting from ``near``.

        ``near`` decides which of the arm's shapes is used; where the arm is
        now decides only which of each joint's equivalent angles. If the
        search from ``near`` fails, it is tried once more from where the arm
        is now. An answer with the elbow bent the other way from the posture
        it was searched from is a different shape of arm, and is not taken;
        nor is one with the wrist flipped the other way from ``wrist``, when
        that is given. With ``explain``, what an answer would hit is logged.

        Every joint but the elbow can turn a full circle each way, so each has
        two angles that put the arm in exactly the same place, a full turn
        apart. The solver does not care which it returns, and the planner then
        dutifully takes the arm the long way round, swinging whatever it holds
        through a full circle. So each joint is brought to whichever of its
        two angles is nearer where it is now, as long as that leaves it room
        to keep turning (`END_STOP_MARGIN`).
        """
        # Why the last search came back empty, for the error message.
        self._missed = "no joint angles reach it"
        with self._moveit.get_planning_scene_monitor().read_only() as scene:
            current = np.array(scene.current_state.get_joint_group_positions(self._group), dtype=float)
            if near is None:
                near = self.looking_towards(float(np.arctan2(pose[1, 3], pose[0, 3])))

            for seed in (np.asarray(near, dtype=float), current):
                # A copy of the arm as it is, so the collision check below has
                # the fingers open as wide as they really are, and whatever
                # the gripper is holding, rather than a closed empty gripper.
                state = copy.copy(scene.current_state)
                state.set_joint_group_positions(self._group, seed)
                state.update()
                if not state.set_from_ik(
                    self._group, make_pose(pose[:3, 3], pose[:3, :3]), self._tip_link, 0.5
                ):
                    continue
                joints = np.array(state.get_joint_group_positions(self._group), dtype=float)
                if np.sign(joints[ELBOW]) != np.sign(seed[ELBOW]):
                    self._missed = "only with the elbow bent the other way"
                    continue
                if wrist is not None and _wrist_side(joints) != wrist:
                    self._missed = "only with the wrist flipped the other way"
                    continue
                for index in FULL_TURN_JOINTS:
                    turns = [joints[index] + k * 2 * np.pi for k in (-2, -1, 0, 1, 2)]
                    turns = [a for a in turns if abs(a) <= 2 * np.pi - END_STOP_MARGIN]
                    if turns:
                        joints[index] = min(turns, key=lambda a: abs(a - current[index]))
                state.set_joint_group_positions(self._group, joints)
                state.update()

                # The answer is checked rather than trusted: the solver has been
                # seen to report success with a configuration that is somewhere else.
                reached = state.get_pose(self._tip_link).position
                if np.linalg.norm(np.array([reached.x, reached.y, reached.z]) - pose[:3, 3]) >= 0.001:
                    continue
                # An answer that puts the arm through the floor or a part is no
                # answer. Handed to the planner, it fails anyway, after ten
                # seconds and a screenful of errors.
                if scene.is_state_colliding(
                    robot_state=state, joint_model_group_name=self._group, verbose=explain
                ):
                    self._missed = "every way of reaching it hits something (see the contacts logged above)"
                    continue
                return state
        return None

    def can_reach(self, pose: np.ndarray, *, wrist: float | None = None) -> bool:
        """Whether the arm can put the tip link at ``pose`` without hitting anything it knows of.

        Asked before picking a part up, about where it will be put down: a part
        picked up and then found to have nowhere to go gets dropped wherever
        the arm happens to be. ``wrist`` is the way the wrist will be flipped
        when it gets there, because the arm cannot flip it with a part in hand.
        """
        return self._solve(pose, None, wrist) is not None

    def wrist_side(self, pose: np.ndarray) -> float | None:
        """Which way the wrist is flipped when `move_to()` puts the tool at ``pose``.

        A 6-axis arm can reach most poses with its wrist flipped either way,
        and no straight-line move can turn one into the other: it would have
        to pass through the point where two wrist axes line up. So whichever
        way the wrist is when a part is picked up is the way it stays until
        the part is put down. +1 or -1, or ``None`` if the pose is out of reach.
        """
        state = self._solve(pose, None)
        return None if state is None else _wrist_side(state.get_joint_group_positions(self._group))

    def current_wrist_side(self) -> float:
        with self._moveit.get_planning_scene_monitor().read_only() as scene:
            return _wrist_side(scene.current_state.get_joint_group_positions(self._group))

    @property
    def planning_scene_monitor(self):
        """The planner's own picture of the room, for the scene client to keep up to date."""
        return self._moveit.get_planning_scene_monitor()

    @staticmethod
    def looking_towards(azimuth: float) -> np.ndarray:
        """The arm's ready posture, turned on its base to face ``azimuth``.

        Elbow up, forearm high and level, wrist over the floor. Starting the
        search for a camera pose from here finds the way of reaching it that
        keeps the forearm above and behind the camera, out of the picture.
        """
        joints = np.array(READY_JOINTS, dtype=float)
        joints[0] = azimuth
        return joints

    def move_to_first(
        self,
        poses: list[np.ndarray],
        *,
        near: np.ndarray | None = None,
        speed: float | None = None,
        any_shape: bool = True,
        wrist: float | None = None,
    ) -> int:
        """Try each pose in turn and stop at the first one that plans.

        Returns the index of the pose that worked, so the caller can carry on
        from it. Raises if none of them do.
        """
        for index, pose in enumerate(poses):
            try:
                self.move_to(pose, near=near, speed=speed, any_shape=any_shape, wrist=wrist)
                return index
            except MotionFailed as failure:
                if index == len(poses) - 1:
                    raise
                # Expected, not a fault: the options are in order of preference,
                # and the first few are often out of reach.
                self._node.get_logger().debug(f"option {index + 1} of {len(poses)} failed: {failure}")
        raise MotionFailed("no poses were offered")

    def move_linear(
        self,
        path: np.ndarray | list[np.ndarray],
        *,
        step: float = 0.005,
        min_fraction: float = 0.99,
        avoid_collisions: bool = True,
        speed: float = 0.2,
    ) -> float:
        """Move the tip link in straight lines through one pose or a list of them.

        With collision checking on, a line that would drive the gripper into
        something comes back short instead of being run, and a line that comes
        back shorter than ``min_fraction`` is refused. A line allowed to stop
        short is not held to arriving at its end. Returns how much of the line
        was run.
        """
        waypoints = [path] if isinstance(path, np.ndarray) and path.ndim == 2 else list(path)
        pose = waypoints[-1]
        if not self._cartesian.wait_for_service(timeout_sec=10.0):
            raise MotionFailed("/compute_cartesian_path is not available")

        request = GetCartesianPath.Request()
        request.header.frame_id = WORLD_FRAME
        # Start from wherever the arm is now. Without this, move_group logs an
        # error about the empty start state on every straight line.
        request.start_state.is_diff = True
        request.group_name = self._group
        request.link_name = self._tip_link
        request.waypoints = [make_pose(p[:3, 3], p[:3, :3]) for p in waypoints]
        request.max_step = step
        request.avoid_collisions = avoid_collisions
        request.max_velocity_scaling_factor = speed
        request.max_acceleration_scaling_factor = speed

        response = self._cartesian.call(request)
        if response.fraction < min_fraction:
            raise MotionFailed(
                f"straight-line move to {_describe(pose)} only solved "
                f"{response.fraction:.0%} of the way (MoveIt error {response.error_code.val})"
            )

        # Straight-line paths go to the controller directly. MoveIt has already
        # timed the trajectory, and its own executor takes a plan object rather
        # than the message this service hands back.
        self._send(self._arm_controller, response.solution.joint_trajectory, "the straight-line move")
        if response.fraction > 0.999:
            self._check_arrival(pose)
        return float(response.fraction)

    def tool_pose(self) -> np.ndarray:
        """Where the tool really is now, as the joint encoders put it."""
        transform = self._tf.lookup_transform(
            WORLD_FRAME, self._tip_link, rclpy.time.Time(), timeout=Duration(seconds=1.0)
        )
        return transform_to_matrix(transform.transform)

    def _check_arrival(self, pose: np.ndarray, settle: float = 1.5) -> None:
        """Make sure the tool ended up at ``pose``.

        The planner and the controller both report success when they have done
        their part, which is not the same as the arm being where it was asked
        to be, and a grasp that closes a centimetre off the part closes on
        air. The joint states lag the motion slightly, so the check waits a
        moment for them to catch up before deciding.
        """
        deadline = time.monotonic() + settle
        while True:
            actual = self.tool_pose()
            offset = float(np.linalg.norm(actual[:3, 3] - pose[:3, 3]))
            cosine = (float(np.trace(actual[:3, :3].T @ pose[:3, :3])) - 1.0) / 2.0
            angle = float(np.arccos(np.clip(cosine, -1.0, 1.0)))
            if offset < ARRIVAL_TOLERANCE and angle < ARRIVAL_ANGLE:
                return
            if time.monotonic() > deadline:
                raise MotionFailed(
                    f"the arm stopped {offset * 1000:.0f} mm and {np.degrees(angle):.1f} degrees "
                    f"away from {_describe(pose)}"
                )
            time.sleep(0.05)

    def _send(self, client: ActionClient, trajectory: JointTrajectory, what: str) -> None:
        """Run a joint trajectory on a controller and wait for it to finish."""
        if not client.wait_for_server(timeout_sec=10.0):
            raise MotionFailed(f"the controller for {what} is not available")
        result = client.send_goal(FollowJointTrajectory.Goal(trajectory=trajectory))
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise MotionFailed(f"the controller did not finish {what}")

    # --------------------------------------------------------------- gripper

    def set_gripper(self, opening: float, *, seconds: float = 1.0, settle: float = 5.0) -> float:
        """Command the gap between the fingers, in metres, and wait for them to finish.

        Finished means both fingers where they were sent, or both stopped
        short against something, as read from the finger joints themselves.
        The controller's own report is no help here: it declares the move
        done at once whatever the fingers did. Each finger is watched on its
        own, because one sometimes sets off a second or more after the other,
        and while it has not moved yet the gap is not changing either; judged
        by the gap alone, that looked like being finished, and the late finger
        then closed in on a leg as the gripper came down over it and knocked
        it over. Returns the gap they ended at.
        """
        half = max(0.0, opening) / 2.0

        point = JointTrajectoryPoint()
        point.positions = [half, half]
        point.time_from_start = DurationMsg(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))

        trajectory = JointTrajectory(joint_names=list(FINGER_JOINTS), points=[point])

        if not self._gripper.wait_for_server(timeout_sec=10.0):
            raise MotionFailed("the gripper controller is not available")
        self._gripper.send_goal(FollowJointTrajectory.Goal(trajectory=trajectory))

        deadline = time.monotonic() + settle
        last, still_since = self._finger_positions, time.monotonic()
        while time.monotonic() < deadline:
            fingers = self._finger_positions
            arrived = [abs(finger - half) < 0.00075 for finger in fingers]
            if all(arrived):
                break
            if max(abs(finger - before) for finger, before in zip(fingers, last, strict=True)) > 0.0003:
                last, still_since = fingers, time.monotonic()
            elif not any(arrived) and time.monotonic() - still_since > 0.4:
                break
            time.sleep(0.02)
        return self.gripper_gap

    def open_gripper(self, opening: float) -> None:
        """Open the fingers to ``opening``, and make sure both of them really did.

        A finger that has stuck — pinned against something by the arm, say —
        leaves the gripper lopsided, and a lopsided gripper lowered over a part
        lands a finger on top of it instead of beside it. A finger still wider
        open than asked is no better: it is on its way in, and it will close
        on the part as the gripper comes down round it.
        """
        self.set_gripper(opening)
        half = opening / 2.0
        if max(abs(finger - half) for finger in self._finger_positions) > 0.002:
            left, right = (p * 1000 for p in self._finger_positions)
            raise MotionFailed(
                f"the fingers did not open: {left:.0f} mm and {right:.0f} mm out of {half * 1000:.0f}"
            )

    def _on_joint_states(self, msg: JointState) -> None:
        by_name = dict(zip(msg.name, msg.position, strict=False))
        if all(joint in by_name for joint in FINGER_JOINTS):
            self._finger_positions = tuple(by_name[joint] for joint in FINGER_JOINTS)

    @property
    def gripper_gap(self) -> float:
        """The gap the fingers are actually at, which is not always the one asked for.

        Closing on a part stops the fingers at the part, so this is a second,
        independent measurement of whatever they closed across.
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
