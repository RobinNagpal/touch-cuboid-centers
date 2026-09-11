"""The workflow.

1. Look round the room and work out what is in it: the floor, the table top
   lying on its stands, the stands, and the legs standing on the floor.
2. Measure the table top properly, close up: length, width and thickness.
3. From the top's size, work out where each of the four legs has to stand
   for a table built at the arm's usual spot.
4. For each leg: pick it up by its top end, carry it round hanging from the
   fingers, stand it on its spot, and look to check it is really standing
   there.
5. Pick the top up by its near edge, carry it round level, and lower it onto
   the legs where they actually ended up.
6. Look at the finished table and check it: its height, whether it is level,
   and whether it is where it was meant to be.

This is the only module that knows what order things happen in. Everything it
calls offers a capability and has no opinion about when it is used.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .arm.camera import WristCamera
from .arm.dimensions import (
    BASE_POSITION,
    CAMERA_OFFSET,
    GRIPPER_MAX_OPENING,
    MAX_GRASP_WIDTH,
    SELF_RADIUS,
    SURVEY_AZIMUTHS_DEG,
    SURVEY_CAMERA_HEIGHT,
    SURVEY_CAMERA_RADIUS,
    SURVEY_LOOK_RADIUS,
)
from .arm.motion import Arm, MotionFailed
from .assembly.grasps import (
    LEG_GRIP_DEPTH,
    backed_off,
    carry_round,
    hold,
    leg_pick_poses,
    leg_place_pose,
    level_top_pose,
    shifted,
    top_pick_poses,
)
from .assembly.plan import SITE, TablePlan, in_the_way, plan_table, table_footprint
from .geometry import Box, describe
from .perception.fitting import cluster, fit_resting_box, surface_tilt
from .perception.pixels import back_project, colour_masks
from .perception.room import Room, is_standing, leg_length, leg_thickness, read_room
from .scene import PlanningSceneClient
from .transforms import WORLD_Z, frame, look_along, view_options

LEGS_NEEDED = 4

# How much narrower than a part the fingers are told to close, so that they
# stop on the part rather than at their target.
#
# It is bounded on both sides. Below about 3 mm it is smaller than the error in
# the measured width, so on a part measured a couple of millimetres too wide
# the fingers close on nothing. Above about 5 mm the fingers are being asked to
# be somewhere the part already is, and since they are position controlled,
# the simulator resolves that by firing the part out sideways.
GRIP_SQUEEZE = 0.004

# How high a part is lifted clear of where it lies before it is carried
# anywhere, and how high above its final place it is brought before being
# lowered on.
LIFT = 0.08
LOWER_FROM = 0.06

# How far a carried part passes above the tops of the legs standing.
CARRY_CLEARANCE = 0.04

# Joint speed, as a fraction of the arm's limits, for moves with a part in
# the gripper. The part is held by friction alone, and a fast swing with it is
# how it ends up across the room.
CARRY_SPEED = 0.1

# How far above the floor, or the legs, a part is let go of. Setting it down
# exactly on the measured surface would, on a surface measured a millimetre
# low, push the part into it; letting it drop a few millimetres is harmless.
DROP = 0.003

# How far back from the top's edge the gripper lines up before reaching in
# over it. Further than the fingers reach in, so they start clear of the board.
TOP_APPROACH = 0.08

# How far the gripper pulls back out of the top once it has let go, along the
# way it reached in. Further than the fingers reach under the board, but no
# further: it is pulling back towards the arm's own base, and the arm is
# already folded up to reach the top's near edge.
TOP_RETREAT = 0.06

# The top is gripped as if it were lying level. One tipped further than this
# is not where the arm thinks it is, and is left alone.
TOP_MAX_TILT = math.radians(5.0)

# A standing leg must be found within this distance of where it was put.
LEG_PLACED_TOLERANCE = 0.03

# How far a leg's measured length or thickness may be from the median of all
# four before it is not taken for a leg. Two legs that end up touching are seen
# as one lump twice as long or twice as thick, and must not be picked up as one.
LEG_SIZE_TOLERANCE = 0.012

# What counts as a good table: within a centimetre of the height it should be,
# and within three degrees of level.
HEIGHT_TOLERANCE = 0.010
TILT_TOLERANCE = math.radians(3.0)

# Attempts at each leg before giving up on it. After a failed attempt the room
# is looked round again, and whichever standing leg is found is tried next.
LEG_ATTEMPTS = 3


@dataclass
class TableCheck:
    """What the camera saw of the finished table."""

    size: np.ndarray  # length along the near edge, depth, height
    expected_height: float
    tilt: float
    offset: float  # how far its centre is from where the legs were planned

    @property
    def good(self) -> bool:
        return (
            abs(float(self.size[2]) - self.expected_height) < HEIGHT_TOLERANCE and self.tilt < TILT_TOLERANCE
        )


@dataclass
class Result:
    top: Box
    leg_length: float
    legs_standing: int
    check: TableCheck | None


class TaskFailed(RuntimeError):
    """Raised when the table cannot be built at all."""


class AssembleTableTask:
    def __init__(self, node, arm: Arm, camera: WristCamera, scene: PlanningSceneClient) -> None:
        self._log = node.get_logger()
        self._arm = arm
        self._camera = camera
        self._scene = scene
        self._floor_z: float | None = None
        self._leg_size: tuple[float, float] | None = None  # length and thickness, once measured
        self._spots: tuple[np.ndarray, ...] = ()  # where the legs go, once planned
        # What the arm currently believes is in the room, by name, which is
        # also exactly what MoveIt is told about.
        self._known: dict[str, Box] = {}

    # ------------------------------------------------------------------ run

    def run(self) -> Result:
        # The launch file gives the cell a head start, but how long it really
        # needs depends on the machine, so wait for the pieces themselves
        # rather than trust that the head start was long enough.
        self._log.info("waiting for the cell to come up")
        self._scene.wait_until_ready()
        self._arm.wait_until_ready()
        self._camera.wait_until_ready()
        self._arm.set_gripper(GRIPPER_MAX_OPENING)

        self._log.info("looking round the room")
        room = self._survey()
        self._report_room(room)
        if room.top is None:
            raise TaskFailed("no table top in sight")
        if len(room.legs) < LEGS_NEEDED:
            raise TaskFailed(f"found {len(room.legs)} legs, and a table needs {LEGS_NEEDED}")

        top = self._measure_top(room.top)
        length = float(np.median([leg_length(leg) for leg in room.legs]))
        thickness = float(np.median([leg_thickness(leg) for leg in room.legs]))
        self._leg_size = (length, thickness)
        if thickness > MAX_GRASP_WIDTH or top.size[2] > MAX_GRASP_WIDTH:
            raise TaskFailed("a part is too thick for the gripper to close round")

        plan = self._plan(top, length, thickness, room)
        self._spots = plan.leg_spots

        standing: list[Box] = []
        for number, spot in enumerate(plan.leg_spots, start=1):
            self._log.info(f"--- leg {number} of {LEGS_NEEDED} ---")
            leg = self._install_leg(spot, number)
            if leg is None:
                raise TaskFailed(f"could not put leg {number} on its spot; the top cannot go on three legs")
            standing.append(leg)

        self._log.info("--- table top ---")
        self._install_top(top, plan, standing)
        check = self._check_table(plan, top, standing)
        self._park()
        return Result(top=top, leg_length=length, legs_standing=len(standing), check=check)

    # -------------------------------------------------------------- looking

    def _survey(self) -> Room:
        """Look all the way round the arm and read the room from everything seen."""
        views = []
        for azimuth_deg in SURVEY_AZIMUTHS_DEG:
            heading = np.array(
                [math.cos(math.radians(azimuth_deg)), math.sin(math.radians(azimuth_deg)), 0.0]
            )
            camera = BASE_POSITION + heading * SURVEY_CAMERA_RADIUS + WORLD_Z * SURVEY_CAMERA_HEIGHT
            target = BASE_POSITION + heading * SURVEY_LOOK_RADIUS
            views.append((camera, target))
        room = self._look(views, stride=2)
        if self._floor_z is None:
            self._floor_z = room.floor_z
            self._scene.set_floor(room.floor_z)
        self._known = {f"obstacle_{i}": box for i, box in enumerate(room.obstacles)}
        if room.top is not None:
            self._known["top"] = room.top
        for i, leg in enumerate(room.legs):
            self._known[f"leg_{i}"] = leg
        self._publish()
        return room

    def _look(self, views, *, stride: int = 1) -> Room:
        """Take a picture from every view and read the room from all of them together.

        A view the arm cannot reach is skipped rather than treated as a
        failure: the extra views are there to fill in what one view misses,
        so losing one costs coverage, not the measurement.
        """
        parts, others = [], []
        for camera, target in views:
            if not self._point_camera(camera, target - camera):
                self._log.warning(
                    f"could not get the camera to {np.round(camera, 2).tolist()}; skipping that view"
                )
                continue
            view = self._camera.capture()
            part_mask, other_mask = colour_masks(view.rgb, view.depth)
            parts.append(back_project(view.depth, part_mask, view.intrinsics, view.camera_to_world))
            others.append(
                back_project(view.depth, other_mask, view.intrinsics, view.camera_to_world, stride=stride)
            )
        if not parts:
            raise MotionFailed("could not reach any of the views")
        return read_room(
            np.concatenate(parts),
            np.concatenate(others),
            self_centre=BASE_POSITION,
            self_radius=SELF_RADIUS,
            floor_z=self._floor_z,
        )

    def _point_camera(self, position: np.ndarray, direction: np.ndarray) -> bool:
        """Put the camera at ``position`` looking along ``direction``.

        tool0 does not go to ``position``: the camera is bolted to one side of
        it, and the offset turns with the tool.

        The arm is steered into its ready posture turned towards the view, so
        that its own forearm stays above and behind the camera. Left to pick
        any way of reaching the view, the planner sometimes brings the elbow
        round underneath, and the picture is then half full of arm.
        """
        near = self._arm.looking_towards(math.atan2(position[1], position[0]))
        for rotation in view_options(look_along(direction)):
            try:
                self._arm.move_to(frame(position - rotation @ CAMERA_OFFSET, rotation), near=near)
                return True
            except MotionFailed:
                continue
        return False

    def _measure_top(self, seen: Box) -> Box:
        """Measure the top close up, from above and from the arm's side.

        The survey saw it from far off. Up close, the upper face gives length
        and width to a millimetre or so, and the near side face, seen at a
        slant, gives its thickness.
        """
        toward = BASE_POSITION - seen.centre
        toward[2] = 0.0
        toward /= np.linalg.norm(toward)
        along = np.cross(WORLD_Z, toward)
        directions = (
            WORLD_Z + 0.8 * toward,
            WORLD_Z + 0.8 * toward + 0.5 * along,
            WORLD_Z + 0.8 * toward - 0.5 * along,
            WORLD_Z + 0.2 * toward,
        )
        views = [(seen.centre + 0.40 * d / np.linalg.norm(d), seen.centre) for d in directions]
        room = self._look(views)
        if room.top is None or np.linalg.norm(room.top.centre - seen.centre) > 0.08:
            self._log.warning("lost the top close up; keeping the survey's measurement")
            top = seen
        else:
            top = room.top
        tilt = math.acos(min(1.0, abs(float(top.axis(2)[2]))))
        length, width, thickness = (float(v) * 100 for v in top.size)
        self._log.info(f"table top: {length:.1f} x {width:.1f} cm, {thickness:.1f} cm thick")
        self._log.info(f"  lying {math.degrees(tilt):.1f} degrees off level")
        if tilt > TOP_MAX_TILT:
            raise TaskFailed("the top is not lying flat, and it can only be picked up lying flat")
        self._known["top"] = top
        self._publish()
        return top

    def _look_around(self, point: np.ndarray, height: float = 0.30) -> Room:
        """Look down on one spot, from straight above and from two sides."""
        offsets = (
            np.array([0.0, 0.0, height]),
            np.array([0.09, -0.06, height - 0.03]),
            np.array([-0.06, 0.09, height - 0.03]),
        )
        return self._look([(point + offset, point) for offset in offsets])

    # -------------------------------------------------------------- planning

    def _plan(self, top: Box, length: float, thickness: float, room: Room) -> TablePlan:
        plan = plan_table(top, length, thickness, self._floor_z, SITE, BASE_POSITION)
        blocking = in_the_way(plan, room.everything())
        if blocking:
            where = np.round(blocking[0].centre[:2], 3).tolist()
            raise TaskFailed(f"the floor where the table goes is not clear: something is at {where}")
        footprint = table_footprint(top)
        self._log.info(
            f"building a {footprint[0] * 100:.1f} x {footprint[1] * 100:.1f} cm table, "
            f"{(length + top.size[2]) * 100:.1f} cm high, centred at {np.round(SITE, 3).tolist()}"
        )
        for spot in plan.leg_spots:
            self._log.info(f"  a leg goes at {np.round(spot[:2], 3).tolist()}")
        return plan

    # --------------------------------------------------------------- legs

    def _install_leg(self, spot: np.ndarray, number: int) -> Box | None:
        """Move one standing leg onto ``spot``, and return it as measured there.

        A failed attempt leaves the leg wherever it ended up, so the room is
        looked round again and whichever waiting leg is found is tried next.
        """
        for attempt in range(1, LEG_ATTEMPTS + 1):
            try:
                if attempt > 1:
                    self._survey()
                waiting = self._waiting_legs()
                if not waiting:
                    self._log.error("no standing legs left to use")
                    return None
                name = min(
                    waiting, key=lambda n: float(np.linalg.norm(waiting[n].centre[:2] - BASE_POSITION[:2]))
                )
                leg = self._inspect_leg(waiting[name])
                self._move_leg(name, leg, spot)
                placed = self._check_standing(spot)
                if placed is None:
                    raise MotionFailed("the leg is not standing where it was put")
                self._known[f"leg_placed_{number}"] = placed
                self._publish()
                self._log.info(
                    f"leg standing, {placed.size[2] * 100:.1f} cm tall, "
                    f"{np.linalg.norm(placed.centre[:2] - spot[:2]) * 1000:.0f} mm from its spot"
                )
                return placed
            except MotionFailed as failure:
                self._log.error(f"attempt {attempt} at leg {number} failed: {failure}")
                self._let_go()
        return None

    def _waiting_legs(self) -> dict[str, Box]:
        """The legs still standing where they started, by name.

        A leg that has fallen over is not one of them: the arm only picks legs
        up standing. Nor is anything the size of two legs, or a leg already on
        one of the table's spots.
        """
        return {
            name: box
            for name, box in self._known.items()
            if name.startswith("leg_")
            and not name.startswith("leg_placed")
            and is_standing(box)
            and self._is_one_leg(box)
            and all(np.linalg.norm(box.centre[:2] - spot[:2]) > LEG_PLACED_TOLERANCE for spot in self._spots)
        }

    def _inspect_leg(self, seen: Box) -> Box:
        """Measure a standing leg again from close above, just before picking it up."""
        room = self._look_around(seen.centre + WORLD_Z * 0.05)
        near = [leg for leg in room.legs if np.linalg.norm(leg.centre[:2] - seen.centre[:2]) < 0.06]
        if not near:
            raise MotionFailed("could not find the leg again close up")
        leg = min(near, key=lambda box: float(np.linalg.norm(box.centre[:2] - seen.centre[:2])))
        if not is_standing(leg):
            raise MotionFailed("that leg has fallen over, and a leg is only picked up standing")
        if not self._is_one_leg(leg):
            raise MotionFailed(f"close up it measures {describe(leg.size)}, which is not one leg")
        self._log.info(f"picking up a {describe(leg.size)} leg")
        return leg

    def _is_one_leg(self, box: Box) -> bool:
        """Whether a stick is the size of one leg, rather than two touching."""
        length, thickness = self._leg_size
        return (
            abs(leg_length(box) - length) < LEG_SIZE_TOLERANCE
            and abs(leg_thickness(box) - thickness) < LEG_SIZE_TOLERANCE
        )

    def _move_leg(self, name: str, leg: Box, spot: np.ndarray) -> None:
        """Pick a standing leg up by its top end and stand it on ``spot``.

        The gripper points straight down the whole time and the leg hangs
        straight down from it: up, round the base above the other legs, and
        down onto the spot.
        """
        width = float(leg_thickness(leg))
        # 2 cm of room either side, so a leg measured a little off is still
        # between the fingers when they come down round it.
        open_width = min(width + 0.040, GRIPPER_MAX_OPENING)
        length = leg_length(leg)

        # Each grip is paired with the pose that would stand the leg on its
        # spot, and a grip with no such pose is not used. Finding that out with
        # the leg already in the air would mean dropping it wherever the arm
        # happened to be.
        choices = []
        for pick in leg_pick_poses(leg):
            wrist = self._arm.wrist_side(shifted(pick, WORLD_Z * LIFT))
            place = leg_place_pose(hold(leg, pick), pick, spot, length, DROP, BASE_POSITION)
            if wrist is not None and self._reachable([place], wrist) is not None:
                choices.append((pick, place))
        if not choices:
            raise MotionFailed("there is no way to stand this leg on its spot, however it is gripped")

        self._arm.open_gripper(open_width)
        pick, place = choices[self._arm.move_to_first([shifted(p, WORLD_Z * LIFT) for p, _ in choices])]
        self._reach(pick)
        self._forget(name)
        self._grip(width)

        held = hold(leg, pick)
        self._scene.attach("carried", leg, pick)
        try:
            # High enough that the leg's lower end passes over every leg standing.
            tops = [box.top_z for n, box in self._known.items() if n.startswith("leg_")]
            height = max(tops + [leg.top_z]) + CARRY_CLEARANCE + length / 2.0
            self._carry(held, pick, place, height)
            self._lower(place)
            if not self._arm.in_contact:
                raise MotionFailed("the fingers stopped feeling the leg on the way over")
            self._release()
        finally:
            self._scene.detach("carried")
        # Straight up, so the open fingers slide off the top of the leg.
        self._arm.move_linear(shifted(place, WORLD_Z * (LEG_GRIP_DEPTH + LIFT)), avoid_collisions=False)

    def _check_standing(self, spot: np.ndarray) -> Box | None:
        """The standing leg nearest ``spot``, if one is there."""
        room = self._look_around(spot + WORLD_Z * 0.05, height=0.35)
        near = [
            leg
            for leg in room.legs
            if is_standing(leg)
            and self._is_one_leg(leg)
            and np.linalg.norm(leg.centre[:2] - spot[:2]) < LEG_PLACED_TOLERANCE
        ]
        return min(near, key=lambda leg: float(np.linalg.norm(leg.centre[:2] - spot[:2])), default=None)

    # ------------------------------------------------------------------ top

    def _install_top(self, top: Box, plan: TablePlan, legs: list[Box]) -> None:
        """Pick the top up by its near edge and lay it on the legs.

        It goes where the legs really are, not where they were planned to be:
        its centre over the middle of the four, at the height of the tallest.
        """
        centre = np.mean([leg.centre for leg in legs], axis=0)
        centre[2] = max(leg.top_z for leg in legs) + float(top.size[2]) / 2.0 + DROP
        thickness = float(top.size[2])
        open_width = min(thickness + 0.030, GRIPPER_MAX_OPENING)

        # As with a leg: a grip is only used if the top can be laid on the legs with it.
        choices = []
        for pick in top_pick_poses(top, BASE_POSITION):
            wrist = self._arm.wrist_side(backed_off(pick, TOP_APPROACH))
            place = level_top_pose(hold(top, pick), centre, plan.outward, pick[:3, 1])
            if wrist is not None and self._reachable([place], wrist) is not None:
                choices.append((pick, place))
        if not choices:
            raise TaskFailed("there is no way to lay the top on the legs from where it lies")

        self._arm.open_gripper(open_width)
        pick, place = choices[self._arm.move_to_first([backed_off(p, TOP_APPROACH) for p, _ in choices])]
        self._forget("top")
        self._approach(pick)
        self._grip(thickness)

        held = hold(top, pick)
        self._scene.attach("carried", top, pick)
        try:
            # Straight up first, unchecked: the top starts off resting on the
            # stands, so MoveIt would refuse any move from there.
            lifted = shifted(pick, WORLD_Z * LIFT)
            self._arm.move_linear(lifted, avoid_collisions=False, speed=CARRY_SPEED)
            centre_height = (lifted @ np.linalg.inv(held))[2, 3]
            self._carry(held, lifted, place, max(centre_height, centre[2] + LOWER_FROM))
            self._lower(place)
            if not self._arm.in_contact:
                raise TaskFailed("the top slipped out of the gripper on the way over")
            self._release()
        finally:
            self._scene.detach("carried")
        self._known["top"] = Box(centre, plan.top.rotation, plan.top.size)
        self._publish()
        self._pull_out(place, TOP_RETREAT)

    # --------------------------------------------------------------- checking

    def _check_table(self, plan: TablePlan, top: Box, legs: list[Box]) -> TableCheck | None:
        """Measure what was built.

        The top and the legs under it now touch, so they come back from the
        camera as one coloured lump, and a box fitted to that lump is the
        table: its footprint is the top's, its height is legs plus top.
        """
        centre = plan.table.centre.copy()
        centre[2] = self._floor_z
        parts = []
        for camera, target in (
            (centre + np.array([0.0, 0.0, 0.50]), centre),
            (centre - plan.outward * 0.15 + np.array([0.0, 0.0, 0.45]), centre),
        ):
            if self._point_camera(camera, target - camera):
                view = self._camera.capture()
                mask, _ = colour_masks(view.rgb, view.depth)
                parts.append(back_project(view.depth, mask, view.intrinsics, view.camera_to_world))
        if not parts:
            self._log.error("could not get the camera over the table to check it")
            return None
        lumps = [
            lump
            for lump in cluster(np.concatenate(parts))
            if np.linalg.norm(lump.mean(axis=0)[:2] - centre[:2]) < 0.15
        ]
        if not lumps:
            self._log.error("no table where the table should be")
            return None
        lump = lumps[0]
        table = fit_resting_box(lump, self._floor_z)
        surface = lump[lump[:, 2] > table.top_z - 0.006]
        check = TableCheck(
            size=table.size,
            expected_height=max(leg.top_z for leg in legs) - self._floor_z + float(top.size[2]),
            tilt=surface_tilt(surface),
            offset=float(np.linalg.norm(table.centre[:2] - plan.table.centre[:2])),
        )
        self._log.info(
            f"table built: {describe(check.size)}, top {math.degrees(check.tilt):.1f} degrees off level"
        )
        return check

    # ------------------------------------------------------------ utilities

    def _reachable(self, poses: list[np.ndarray], wrist: float) -> np.ndarray | None:
        """The first pose a held part could be put down from, or ``None``.

        The arm has to be able to reach the pose itself and the one it is
        lowered from, without hitting anything it knows of, and with its wrist
        flipped the way it will be: the way it was when it picked the part up.
        """
        for pose in poses:
            if self._arm.can_reach(pose, wrist=wrist) and self._arm.can_reach(
                shifted(pose, WORLD_Z * LOWER_FROM), wrist=wrist
            ):
                return pose
        return None

    def _carry(self, held: np.ndarray, start: np.ndarray, place: np.ndarray, height: float) -> None:
        """Carry a held part up to ``height`` and round the base to above ``place``.

        ``height`` is where the part's centre travels. Round the base on an
        arc first (`carry_round()`), because that never tips the part. If that
        path is refused, the planner is asked for a way to the end of it, with
        the arm kept in its usual shape.
        """
        path = carry_round(held, start, place, height, BASE_POSITION)
        try:
            self._arm.move_linear(path, speed=CARRY_SPEED)
            return
        except MotionFailed as failure:
            self._log.info(f"{failure}; letting the planner carry it instead")
        self._arm.move_to(path[-1], speed=CARRY_SPEED, any_shape=False)

    def _grip(self, width: float) -> None:
        """Close the fingers across a part ``width`` wide, and make sure they feel it."""
        self._arm.set_gripper(max(width - GRIP_SQUEEZE, 0.0))
        self._log.info(f"fingers closed to {self._arm.gripper_gap * 1000:.0f} mm on {width * 1000:.0f} mm")
        if not self._arm.wait_for_contact():
            raise MotionFailed("the fingers felt nothing, so the grasp missed")

    def _reach(self, pose: np.ndarray) -> None:
        """Move in a straight line if the arm can, and let the planner decide if not.

        Straight lines are followed from wherever the arm happens to be
        standing, and some of those configurations cannot be walked along the
        line without folding the arm into something. Picking a part up only
        needs the tool to arrive, not the path to be straight.
        """
        try:
            self._arm.move_linear(pose)
        except MotionFailed as failure:
            self._log.info(f"{failure}; letting the planner find its own way there")
            self._arm.move_to(pose)

    def _lower(self, pose: np.ndarray) -> None:
        """Lower a held part the last few centimetres onto where it goes.

        Straight down, checked against everything MoveIt knows about — the
        floor, the parts, and the arm itself. The part stops 3 mm short of
        whatever it is being set on, so the checking does not get in the way
        of setting it down; what it catches is the arm.

        If the straight line is refused, the planner is asked for a way down
        with the arm in its usual shape.
        """
        try:
            self._arm.move_linear(pose, speed=CARRY_SPEED)
        except MotionFailed as failure:
            self._log.info(f"{failure}; letting the planner lower it instead")
            self._arm.move_to(pose, speed=CARRY_SPEED, any_shape=False)

    def _let_go(self) -> None:
        """Get the gripper free after something went wrong.

        Straight up first, then open. A failed move can leave the gripper
        pressed down on a part, and fingers told to open while they are pinned
        like that stay where they are.
        """
        up = shifted(self._arm.tool_pose(), WORLD_Z * LIFT)
        try:
            self._arm.move_linear(up)
        except MotionFailed:
            # Checked first, because an unchecked line can fold the arm into
            # itself. Only if that is refused — the gripper is touching
            # something, so the start already counts as a collision — is a
            # short unchecked lift used.
            try:
                self._arm.move_linear(shifted(self._arm.tool_pose(), WORLD_Z * 0.03), avoid_collisions=False)
            except MotionFailed as failure:
                self._log.warning(f"could not lift clear before letting go: {failure}")
        self._arm.set_gripper(GRIPPER_MAX_OPENING)

    def _release(self) -> None:
        """Let go of a part that has just been set down.

        The fingers open all the way, not just clear of the part. A leg that
        has only just been set down falls over if a finger so much as brushes
        it as the gripper moves away.
        """
        self._arm.set_gripper(GRIPPER_MAX_OPENING)

    def _pull_out(self, place: np.ndarray, distance: float) -> None:
        """Pull the open gripper back out of the top, then lift clear.

        The top has to be backed out of, because one finger is under it.
        """
        away = backed_off(place, distance)
        try:
            self._arm.move_linear(away, avoid_collisions=False)
        except MotionFailed as failure:
            self._log.info(f"{failure}; letting the planner back away instead")
            self._arm.move_to(away)
        self._reach(shifted(away, WORLD_Z * LIFT))

    def _approach(self, pose: np.ndarray) -> None:
        """The last few centimetres in over the top's edge.

        Checked if possible. The stands are in the scene as measured, and the
        finger under the board passes between them, so a stand measured a
        little fat can make the checked line refuse. The line is short and
        starts from a pose reached under checking, so it is then run
        unchecked rather than given up on.
        """
        try:
            self._arm.move_linear(pose)
        except MotionFailed as failure:
            self._log.info(f"{failure}; running the last {TOP_APPROACH * 100:.0f} cm unchecked")
            self._arm.move_linear(pose, avoid_collisions=False)

    def _forget(self, name: str) -> None:
        """Drop a part from MoveIt's scene, so the fingers are allowed to reach it."""
        self._known.pop(name, None)
        self._publish()

    def _publish(self) -> None:
        self._scene.set_objects(dict(self._known))

    def _park(self) -> None:
        self._arm.set_gripper(GRIPPER_MAX_OPENING)
        try:
            self._arm.move_to(frame(np.array([0.30, 0.0, 0.55]), look_along(-WORLD_Z)))
        except MotionFailed as failure:
            self._log.warning(f"could not park the arm: {failure}")

    def _report_room(self, room: Room) -> None:
        self._log.info(f"floor at z = {room.floor_z * 1000:.1f} mm")
        for box in room.obstacles:
            self._log.info(f"  obstacle, {describe(box.size)}, at {np.round(box.centre[:2], 3).tolist()}")
        if room.top is not None:
            self._log.info(
                f"  table top, {describe(room.top.size)}, at {np.round(room.top.centre, 3).tolist()}"
            )
        for leg in room.legs:
            self._log.info(f"  leg, {describe(leg.size)}, at {np.round(leg.centre[:2], 3).tolist()}")
        for box in room.unknown:
            self._log.warning(f"  something coloured that is neither a leg nor a top, {describe(box.size)}")
