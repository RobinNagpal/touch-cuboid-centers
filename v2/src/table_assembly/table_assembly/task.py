"""The workflow.

1. Look round the room and work out what is in it: the floor, the wall, the
   table top leaning against it, and the legs lying on the floor.
2. Measure the table top properly, close up: length, width and thickness.
3. Decide where to build the table, and from the top's size, where each of
   the four legs has to stand.
4. For each leg: pick it up, stand it upright on its spot, and look to check
   it is really standing there.
5. Pick the top up by its upper edge, turn it level, and lower it onto the
   legs where they actually ended up.
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
    BUILD_RADIUS_MAX,
    BUILD_RADIUS_MIN,
    BUILD_RADIUS_PREFERRED,
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
    backed_off,
    carry_upright,
    hold,
    leg_pick_poses,
    level_top_poses,
    shifted,
    standing_leg_poses,
    top_pick_poses,
    turned_upright,
    upright_tool_pose,
)
from .assembly.plan import TablePlan, choose_site, gripped_axis, plan_table, table_footprint
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

# How high a part is lifted clear of the floor before it is carried anywhere,
# and how high above its final place it is brought before being lowered on.
LIFT = 0.08
LOWER_FROM = 0.06

# How far a carried leg's lower end passes above the tops of the legs already
# standing.
CARRY_CLEARANCE = 0.04

# How far a leg's lower end stays above the floor as it is turned upright.
# Higher than any leg still lying on the floor nearby, which the turning leg's
# lower end would otherwise sweep into.
TURN_CLEARANCE = 0.08

# Joint speed, as a fraction of the arm's limits, for moves with a part in
# the gripper. The part is held by friction alone, and a fast swing with it is
# how it ends up across the room.
CARRY_SPEED = 0.1

# How far above the floor, or the legs, a part is let go of. Setting it down
# exactly on the measured surface would, on a surface measured a millimetre
# low, push the part into it; letting it drop a few millimetres is harmless.
DROP = 0.003

# How far back along the slope of the top the gripper lines up before moving
# onto its edge.
TOP_APPROACH = 0.08

# How far the gripper pulls back out of the top once it has let go, along the
# way it reached in. Further than the fingers reach under the board, but no
# further: the gripper is pulling back towards the arm's own base, and the arm
# is already folded up tight to reach the near edge of the table.
TOP_RETREAT = 0.07

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

# Attempts at each leg before giving up on it. A leg that falls over is found
# again by looking round the room, and tried again.
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

        standing: list[Box] = []
        for number, spot in enumerate(plan.leg_spots, start=1):
            self._log.info(f"--- leg {number} of {LEGS_NEEDED} ---")
            leg = self._install_leg(spot, plan, number)
            if leg is None:
                raise TaskFailed(f"could not stand leg {number} up; the top cannot go on three legs")
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
            self._known[f"leg_{'standing' if is_standing(leg) else 'lying'}_{i}"] = leg
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
        """Measure the top close up, from in front of it and from above its edge.

        The survey saw it from far off and at a slant. Up close, the front
        face gives length and width to a millimetre or so, and a look down on
        its upper edge shows how thick it is.
        """
        normal, up = seen.axis(2), _up_the_face(seen)
        along = np.cross(up, normal)
        directions = (
            normal + 0.9 * WORLD_Z,
            normal + 0.9 * WORLD_Z + 0.5 * along,
            normal + 0.9 * WORLD_Z - 0.5 * along,
            0.4 * normal + WORLD_Z,
        )
        views = [(seen.centre + 0.40 * d / np.linalg.norm(d), seen.centre) for d in directions]
        room = self._look(views)
        if room.top is None or np.linalg.norm(room.top.centre - seen.centre) > 0.08:
            self._log.warning("lost the top close up; keeping the survey's measurement")
            return seen
        top = room.top
        lean = math.degrees(math.acos(min(1.0, abs(float(_up_the_face(top)[2])))))
        length, width, thickness = (float(v) * 100 for v in top.size)
        self._log.info(f"table top: {length:.1f} x {width:.1f} cm, {thickness:.1f} cm thick")
        self._log.info(f"  standing on its edge, leaning {lean:.1f} degrees back from upright")
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
        footprint = table_footprint(top)
        radii = tuple(np.linspace(BUILD_RADIUS_MIN, BUILD_RADIUS_MAX, 3))
        centre = choose_site(
            footprint, room.everything(), BASE_POSITION, radii=radii, preferred_radius=BUILD_RADIUS_PREFERRED
        )
        if centre is None:
            raise TaskFailed("no clear patch of floor in reach is big enough to build the table on")
        plan = plan_table(top, length, thickness, self._floor_z, centre, BASE_POSITION)
        self._log.info(
            f"building a {footprint[0] * 100:.1f} x {footprint[1] * 100:.1f} cm table, "
            f"{(length + top.size[2]) * 100:.1f} cm high, centred at {np.round(centre, 3).tolist()}"
        )
        for spot in plan.leg_spots:
            self._log.info(f"  a leg goes at {np.round(spot[:2], 3).tolist()}")
        return plan

    # --------------------------------------------------------------- legs

    def _install_leg(self, spot: np.ndarray, plan: TablePlan, number: int) -> Box | None:
        """Stand one lying leg upright on ``spot``, and return it as measured there.

        A failed attempt leaves the leg wherever it ended up, so the room is
        looked round again and whichever lying leg is found is tried next.
        """
        for attempt in range(1, LEG_ATTEMPTS + 1):
            try:
                if attempt > 1:
                    self._survey()
                lying = {
                    name: box
                    for name, box in self._known.items()
                    if name.startswith("leg_lying") and self._is_one_leg(box)
                }
                if not lying:
                    self._log.error("no lying legs left to use")
                    return None
                name = min(
                    lying, key=lambda n: float(np.linalg.norm(lying[n].centre[:2] - BASE_POSITION[:2]))
                )
                leg = self._inspect_leg(lying[name])
                self._stand_up(name, leg, spot)
                standing = self._check_standing(spot)
                if standing is None:
                    raise MotionFailed("the leg is not standing where it was put")
                self._known[f"leg_standing_{number}"] = standing
                self._publish()
                self._log.info(
                    f"leg standing, {standing.size[2] * 100:.1f} cm tall, "
                    f"{np.linalg.norm(standing.centre[:2] - spot[:2]) * 1000:.0f} mm from its spot"
                )
                return standing
            except MotionFailed as failure:
                self._log.error(f"attempt {attempt} at leg {number} failed: {failure}")
                self._let_go()
        return None

    def _inspect_leg(self, seen: Box) -> Box:
        """Measure a lying leg again from close above, just before picking it up."""
        room = self._look_around(seen.centre)
        near = [leg for leg in room.legs if np.linalg.norm(leg.centre[:2] - seen.centre[:2]) < 0.06]
        if not near:
            raise MotionFailed("could not find the leg again close up")
        leg = min(near, key=lambda box: float(np.linalg.norm(box.centre[:2] - seen.centre[:2])))
        if is_standing(leg):
            raise MotionFailed("that leg is already standing")
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

    def _stand_up(self, name: str, leg: Box, spot: np.ndarray) -> None:
        """Pick a lying leg up round its middle and stand it upright on ``spot``.

        Four stages, each short and slow: lift it clear, turn it upright where
        it is, carry it upright to above its spot, and lower it on.
        """
        width = float(leg.size[1])
        open_width = min(width + 0.020, GRIPPER_MAX_OPENING)
        length = leg_length(leg)
        lift = WORLD_Z * (self._floor_z + length / 2.0 + TURN_CLEARANCE - leg.centre[2])

        # The leg stays in MoveIt's scene until the open fingers are either side
        # of it. If the straight line down fails and the planner has to find
        # its own way, a planner that did not know the leg was there would be
        # free to swing the fingers straight through it.
        # Of the two ways to pick the leg up, the one that ends up, once the
        # leg is turned upright, with the gripper already reaching towards the
        # spot is tried first, so the carry has least turning to do.
        outward = _outward(spot)
        options = leg_pick_poses(leg, self._floor_z)
        options.sort(
            key=lambda pose: -float(turned_upright(hold(leg, pose), shifted(pose, lift))[-1][:3, 2] @ outward)
        )

        self._arm.open_gripper(open_width)
        pick = options[self._arm.move_to_first([shifted(pose, WORLD_Z * LIFT) for pose in options])]
        self._reach(pick)
        self._forget(name)
        self._grip(width)

        held = hold(leg, pick)
        self._scene.attach("carried", leg, pick)
        try:
            self._arm.move_linear(shifted(pick, lift), speed=CARRY_SPEED)
            reach = self._turn_upright(held, shifted(pick, lift))
            place = self._carry_leg(
                held, (shifted(pick, lift) @ np.linalg.inv(held))[:3, 3], reach, spot, length
            )
            self._lower(place)
            if not self._arm.in_contact:
                raise MotionFailed("the fingers stopped feeling the leg on the way over")
            self._release()
        finally:
            self._scene.detach("carried")
        self._lift_off(place, length)

    def _carry_leg(
        self, held: np.ndarray, centre: np.ndarray, reach: np.ndarray, spot: np.ndarray, length: float
    ) -> np.ndarray:
        """Carry an upright leg to just above ``spot``; return the pose to lower it to.

        Upright all the way, round the base at a height that clears any leg
        already standing (`carry_upright()`). If that path is refused, the
        planner is asked for a way to the nearest standing pose instead, with
        the arm kept in its usual shape.
        """
        tops = [box.top_z for name, box in self._known.items() if name.startswith("leg_standing")]
        clear = max(tops + [self._floor_z + TURN_CLEARANCE]) + CARRY_CLEARANCE
        height = max(centre[2], clear + length / 2.0)
        end = spot + WORLD_Z * (length / 2.0 + DROP)
        path = carry_upright(held, centre, reach, end, height, BASE_POSITION)
        try:
            self._arm.move_linear(path, speed=CARRY_SPEED)
            return upright_tool_pose(held, end, _outward(spot))
        except MotionFailed as failure:
            self._log.info(f"{failure}; letting the planner carry it instead")
        options = standing_leg_poses(held, spot, length, DROP, _outward(spot))
        index = self._arm.move_to_first(
            [shifted(pose, WORLD_Z * LOWER_FROM) for pose in options], speed=CARRY_SPEED, any_shape=False
        )
        return options[index]

    def _turn_upright(self, held: np.ndarray, lifted: np.ndarray) -> np.ndarray:
        """Turn a held leg upright where it is.

        It ends with the gripper pointing out along the arm's reach if that
        can be done, back towards the base if not, and only as a last resort
        pointing whichever way the leg happened to lie. Each is a slow straight
        path, checked against everything in the room. There is deliberately no
        fall back to a free planned move here: with a tall part in hand near
        the floor, that is where parts get flung.

        The turn does not have to finish exactly. It only has to bring the leg
        close to upright, round its own middle; the carry that follows goes to
        the exact standing pose. So a turn that gets nine tenths of the way is
        good enough.
        """
        out = _outward(lifted[:3, 3])
        for reach in (out, -out, None):
            try:
                turn = turned_upright(held, lifted, reach)
                fraction = self._arm.move_linear(turn, speed=CARRY_SPEED, min_fraction=0.9)
                if fraction < 0.999:
                    self._log.info(f"turned the leg {fraction:.0%} of the way upright")
                return turn[-1][:3, 2]
            except MotionFailed as failure:
                self._log.debug(f"could not turn the leg that way: {failure}")
        raise MotionFailed("could not turn the leg upright")

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
        """Pick the top up by its upper edge and lay it on the legs.

        It goes where the legs really are, not where they were planned to be:
        its centre over the middle of the four, at the height of the tallest.
        """
        centre = np.mean([leg.centre for leg in legs], axis=0)
        centre[2] = max(leg.top_z for leg in legs) + float(top.size[2]) / 2.0 + DROP
        thickness = float(top.size[2])
        open_width = min(thickness + 0.030, GRIPPER_MAX_OPENING)

        self._arm.open_gripper(open_width)
        options = top_pick_poses(top)
        pick = options[self._arm.move_to_first([backed_off(pose, TOP_APPROACH) for pose in options])]
        self._approach(pick)
        self._forget("top")
        self._grip(thickness)

        held = hold(top, pick)
        self._scene.attach("carried", top, pick)
        try:
            # Straight up first, unchecked: the top starts off touching the
            # floor and the wall, so MoveIt would refuse any move from there.
            # Lifting it moves its back away from the wall as well as up.
            self._arm.move_linear(shifted(pick, WORLD_Z * LIFT), avoid_collisions=False, speed=CARRY_SPEED)
            options = level_top_poses(held, centre, plan.outward)
            place = options[
                self._arm.move_to_first(
                    [shifted(pose, WORLD_Z * LOWER_FROM) for pose in options],
                    speed=CARRY_SPEED,
                    any_shape=False,
                )
            ]
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
        of setting it down; what it catches is the arm. Folded up tight to
        reach the near legs, the arm can come down into the floor or into
        itself before the part gets there, and an unchecked line once drove it
        into something and stalled 9 cm short.

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

        The fingers open all the way, not just clear of the part. A tall thin
        leg that has only just been stood up falls over if a finger so much as
        brushes it as the gripper backs away.
        """
        self._arm.set_gripper(GRIPPER_MAX_OPENING)

    def _lift_off(self, place: np.ndarray, length: float) -> None:
        """Lift the open gripper straight up off a leg it has just stood up.

        Up, not back. The fingers are either side of the leg's middle, and
        lifting slides them up and off its top. Pulling back instead, towards
        the arm's base, folds an arm that is already folded tight for the near
        legs further still, until it cannot move at all.
        """
        self._arm.move_linear(shifted(place, WORLD_Z * (length / 2.0 + LIFT)), avoid_collisions=False)

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
        """The last few centimetres onto the top's edge.

        Checked if possible. The wall is in the scene as measured, and the
        finger on its side passes a couple of centimetres above it, so a wall
        measured a little fat can make the checked line refuse. The line is
        short and starts from a pose reached under checking, so it is then run
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


def _outward(point: np.ndarray) -> np.ndarray:
    """Level unit vector from the arm's base towards ``point``."""
    direction = np.array([point[0] - BASE_POSITION[0], point[1] - BASE_POSITION[1], 0.0])
    return direction / np.linalg.norm(direction)


def _up_the_face(top: Box) -> np.ndarray:
    """The in-plane axis of the top that points most upwards."""
    index, sign = gripped_axis(top)
    return top.axis(index) * sign
