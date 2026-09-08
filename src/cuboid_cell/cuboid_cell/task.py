"""The workflow.

For every cuboid on the pending side of the table:

1. pick it up and set it down on the done side, so the pending side always
   holds exactly the boxes that are still to do;
2. look at it again from three angles now that it is on its own;
3. work out its length, width and height, and the area of its three
   different faces;
4. touch the middle of the biggest face the arm can reach.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np

from .arm import Arm, MotionFailed
from .camera import WristCamera
from .cell import (
    APPROACH_HEIGHT,
    CAMERA_OFFSET,
    DONE_ZONE,
    FINGER_TABLE_CLEARANCE,
    FINGERTIP_OFFSET,
    GRIPPER_MAX_OPENING,
    LIFT_HEIGHT,
    PENDING_ZONE,
    ROBOT_BASE,
    SURVEY_HEIGHT,
    TABLE_TOP_Z,
    TOUCH_STANDOFF,
    zone_centre,
    zone_slots,
)
from .geometry import WIDTH, Cuboid, Face, largest_touchable_face
from .perception import back_project, find_cuboids, object_mask
from .scene import PlanningSceneClient
from .transforms import frame, grasp_options, look_along, make_pose, release_options, view_options

UP = np.array([0.0, 0.0, 1.0])

# Where the camera is put to look over a whole zone, relative to the middle of
# that zone. One view from straight above plus two from the sides, so a box
# that hides another one from one angle does not hide it from all three.
SURVEY_OFFSETS = (
    np.array([0.00, 0.00, SURVEY_HEIGHT]),
    np.array([0.10, -0.08, SURVEY_HEIGHT - 0.06]),
    np.array([-0.08, 0.10, SURVEY_HEIGHT - 0.06]),
)

# The same idea, closer in, for looking at one box on its own.
INSPECT_OFFSETS = (
    np.array([0.00, 0.00, APPROACH_HEIGHT + 0.14]),
    np.array([0.09, -0.05, APPROACH_HEIGHT + 0.10]),
    np.array([-0.05, 0.09, APPROACH_HEIGHT + 0.10]),
)

# A side face is approached from above the horizontal, otherwise the body of
# the gripper would reach the table before the fingertip reached the face.
SIDE_APPROACH_TILT = math.radians(40.0)

# How far past the surface the fingertip is driven. It has to be more than the
# error in the measurement, or a face measured a millimetre further away than
# it is would never actually be touched.
TOUCH_DEPTH = 0.004

# How much narrower than the box the fingers are told to close, so that they
# stop on the box rather than at their target.
#
# It is bounded on both sides. Below about 3 mm it is smaller than the error in
# the measured width, so on a box measured a couple of millimetres too wide the
# fingers close on nothing. Above about 5 mm the fingers are being asked to be
# somewhere the box already is, and since they are position controlled, the
# simulator resolves that by firing the box out sideways.
#
# Whether the box is really held is read from the fingertip contact sensors
# rather than from how far the fingers travelled.
GRIP_SQUEEZE = 0.004


@dataclass
class Result:
    """What the arm worked out about one cuboid."""

    cuboid: Cuboid
    face: Face
    contact: bool


class TouchCuboidsTask:
    def __init__(self, node, arm: Arm, camera: WristCamera, scene: PlanningSceneClient) -> None:
        self._log = node.get_logger()
        self._arm = arm
        self._camera = camera
        self._scene = scene

    # ------------------------------------------------------------------ run

    def run(self) -> list[Result]:
        # The launch file gives the cell a head start, but how long it really
        # needs depends on the machine, so wait for the pieces themselves
        # rather than trust that the head start was long enough.
        self._log.info("waiting for the cell to come up")
        self._scene.wait_until_ready()
        self._arm.wait_until_ready()
        self._camera.wait_until_ready()

        self._scene.add_table()
        self._arm.set_gripper(GRIPPER_MAX_OPENING)

        pending = self._survey(PENDING_ZONE)
        self._log.info(f"{len(pending)} cuboid(s) waiting on the pending side")
        for found in pending:
            self._log.info(f"  {_describe(found)} at {np.round(found.centre, 3).tolist()}")
        slots = zone_slots(DONE_ZONE, max(len(pending), 1))

        results: list[Result] = []
        # A slot is only used up by a box that actually reaches it, so a
        # cuboid the arm fumbles is tried again rather than costing a place on
        # the done side. The cap is what stops a box the arm simply cannot
        # manage from being retried forever.
        tries_left = 2 * len(slots)
        while len(results) < len(slots) and tries_left > 0:
            tries_left -= 1

            # The pending side is surveyed again every time round, so the boxes
            # still to do are whatever is still there. A box the first survey
            # missed, or one the arm dropped, gets picked up on a later pass.
            remaining = self._survey(PENDING_ZONE)
            if not remaining:
                break

            target = min(remaining, key=lambda box: np.linalg.norm(box.centre - ROBOT_BASE))
            slot = slots[len(results)]
            self._log.info(f"--- cuboid {len(results) + 1} of {len(slots)} ---")

            # Everything except the box being picked stays in the planning
            # scene as an obstacle.
            others = [box for box in remaining if box is not target]
            self._publish(others, [r.cuboid for r in results])

            try:
                self._relocate(target, slot)

                measured = self._inspect(slot)
                if measured is None:
                    self._log.warning("lost track of the cuboid after moving it; skipping")
                    continue

                self._publish(others, [r.cuboid for r in results] + [measured])
                face = largest_touchable_face(measured, ROBOT_BASE)
                self._report(measured, face)

                contact = self._touch(face)
                self._log.info("touched it" if contact else "reached the face but felt no contact")
                results.append(Result(measured, face, contact))
            except MotionFailed as failure:
                # One cuboid the arm cannot manage is not a reason to abandon
                # the rest of the table.
                self._log.error(f"giving up on this cuboid: {failure}")
                self._arm.set_gripper(GRIPPER_MAX_OPENING)

        left_over = self._survey(PENDING_ZONE)
        if left_over:
            self._log.warning(f"{len(left_over)} cuboid(s) still on the pending side, not done")
        self._park()
        return results

    # -------------------------------------------------------------- looking

    def _look(self, target: np.ndarray, offsets) -> list[Cuboid]:
        """Take a picture from every offset and fit boxes to what came back.

        A viewpoint the arm cannot reach is skipped rather than treated as a
        failure: the extra angles are there to fill in what one view misses, so
        losing one costs accuracy, not the measurement.
        """
        clouds = []
        for offset in offsets:
            if not self._point_camera(target + offset, -offset):
                self._log.warning(f"skipping the viewpoint {np.round(offset, 2).tolist()} from here")
                continue
            view = self._camera.capture()
            mask = object_mask(view.rgb, view.depth)
            clouds.append(back_project(view.depth, mask, view.intrinsics, view.camera_to_world))

        if not clouds:
            raise MotionFailed("could not reach any viewpoint over the table")
        return find_cuboids(np.concatenate(clouds), TABLE_TOP_Z)

    def _point_camera(self, position: np.ndarray, direction: np.ndarray) -> bool:
        """Put the camera at ``position`` looking along ``direction``.

        How far the camera is rolled about the direction it is looking makes no
        difference to what it sees, because every pixel is placed in the world
        using the pose the arm was actually in. So the roll is free, and trying
        a few of them is what lets the arm reach viewpoints over the far
        corners of the table.

        tool0 does not go to ``position``: the camera is bolted to one side of
        it, and the offset turns with the tool.
        """
        for rotation in view_options(look_along(direction)):
            try:
                self._arm.move_to_pose(position - rotation @ CAMERA_OFFSET, rotation)
                return True
            except MotionFailed:
                continue
        return False

    def _survey(self, zone) -> list[Cuboid]:
        x_min, x_max, y_min, y_max = zone
        found = self._look(zone_centre(zone), SURVEY_OFFSETS)
        return [
            box
            for box in found
            if x_min - 0.06 <= box.centre[0] <= x_max + 0.06 and y_min - 0.06 <= box.centre[1] <= y_max + 0.06
        ]

    def _inspect(self, slot: np.ndarray) -> Cuboid | None:
        """Measure the one box sitting at ``slot``, close up."""
        found = self._look(slot + UP * 0.03, INSPECT_OFFSETS)
        near = [box for box in found if np.linalg.norm(box.centre[:2] - slot[:2]) < 0.10]
        return min(near, key=lambda box: np.linalg.norm(box.centre[:2] - slot[:2])) if near else None

    # --------------------------------------------------------------- moving

    def _relocate(self, box: Cuboid, slot: np.ndarray, name: str = "carried") -> None:
        """Carry one cuboid from the pending side to ``slot`` on the done side."""
        width = float(box.size[WIDTH])
        rotation = _grasp_rotation(box)
        open_width = min(width + 0.020, GRIPPER_MAX_OPENING)

        # Both the pick and the place put the fingertips just clear of the
        # table, whatever the box's height, so the fingers close around the
        # bottom of it rather than into the table top.
        tool_z = TABLE_TOP_Z + FINGER_TABLE_CLEARANCE + FINGERTIP_OFFSET
        pick = np.array([box.centre[0], box.centre[1], tool_z])
        above_pick = pick + UP * LIFT_HEIGHT
        place = np.array([slot[0], slot[1], tool_z])
        above_place = place + UP * LIFT_HEIGHT

        self._log.info(f"picking up a {_describe(box)} box and moving it to the done side")
        self._arm.set_gripper(open_width)
        rotation = self._arm.move_to_first_reachable(above_pick, grasp_options(rotation))
        self._reach(pick, rotation)
        closed_to = max(width - GRIP_SQUEEZE, 0.0)
        self._arm.set_gripper(closed_to)
        time.sleep(0.6)
        gap = self._arm.gripper_gap
        self._log.info(f"fingers closed to {gap * 1000:.0f} mm on a {width * 1000:.0f} mm side")
        if not self._arm.wait_for_contact():
            raise MotionFailed("the fingers felt nothing, so the grasp missed the box")

        # MoveIt has to know the gripper is full, or it will plan the carry as
        # if the arm were empty and sweep the box through whatever is already
        # standing on the done side. The detach has to happen even if a move
        # fails, or the planner keeps refusing every later move because of a
        # box the arm is no longer holding.
        self._scene.attach(name, box, frame(pick, rotation))
        try:
            self._reach(above_pick, rotation)
            rotation = self._arm.move_to_first_reachable(above_place, release_options(rotation))
            # Setting the box down is unchecked, because putting it down means
            # standing it on the table, and a box resting on the table is
            # exactly what collision checking is there to prevent. Straight down
            # into an empty slot from a height that was reached under checking
            # has nothing else it can hit.
            self._arm.move_linear([make_pose(place, rotation)], avoid_collisions=False)
            if not self._arm.in_contact:
                raise MotionFailed("the fingers stopped feeling the box on the way over")
            self._arm.set_gripper(open_width)
            time.sleep(0.4)
        finally:
            self._scene.detach(name)
        self._reach(above_place, rotation)

    def _reach(self, position: np.ndarray, rotation: np.ndarray) -> None:
        """Move in a straight line if the arm can, and let the planner decide if not.

        Straight lines are followed from wherever the arm happens to be
        standing, and some of those configurations cannot be walked along the
        line without folding the arm into the table. Picking a box up and
        putting it down only needs the tool to arrive; it does not need the
        path to be straight, so a planned move is a fine second choice.
        """
        try:
            self._arm.move_linear([make_pose(position, rotation)])
        except MotionFailed as failure:
            self._log.info(f"{failure}; letting the planner find its own way there")
            self._arm.move_to_pose(position, rotation)

    def _touch(self, face: Face) -> bool:
        """Press the closed fingertips into the middle of a face."""
        back = _approach_direction(face.normal)
        rotation = look_along(-back)
        standoff = face.centre + back * (FINGERTIP_OFFSET + TOUCH_STANDOFF)
        contact = face.centre + back * (FINGERTIP_OFFSET - TOUCH_DEPTH)

        self._arm.set_gripper(0.0)
        self._arm.move_to_pose(standoff, rotation)
        # The last few centimetres are deliberately unchecked: the fingertip is
        # being driven into the box on purpose, and the straight line from a
        # standoff that was itself reached under collision checking cannot run
        # into anything else.
        # The whole line has to be followed. Allowing it to stop even a few
        # per cent short means stopping millimetres off the face, which is the
        # difference between touching the box and hovering next to it.
        self._arm.move_linear([make_pose(contact, rotation)], avoid_collisions=False, min_fraction=0.99)
        touched = self._arm.wait_for_contact()
        self._arm.move_linear([make_pose(standoff, rotation)], avoid_collisions=False)
        return touched

    def _park(self) -> None:
        self._arm.set_gripper(GRIPPER_MAX_OPENING)
        self._arm.move_to_pose(
            zone_centre(PENDING_ZONE) + np.array([0.0, 0.0, SURVEY_HEIGHT]),
            look_along(np.array([0.0, 0.0, -1.0])),
        )

    # ------------------------------------------------------------ reporting

    def _publish(self, *groups) -> None:
        boxes = [box for group in groups for box in group]
        self._scene.set_cuboids({f"cuboid_{i}": box for i, box in enumerate(boxes)})

    def _report(self, box: Cuboid, face: Face) -> None:
        length, width, height = (float(v) * 100 for v in box.size)
        self._log.info(f"measured {length:.1f} x {width:.1f} x {height:.1f} cm")
        for name, area in box.distinct_face_areas().items():
            self._log.info(f"  {name:<16} {area * 1e4:6.1f} cm^2")
        centre = np.round(face.centre, 3).tolist()
        self._log.info(f"  biggest reachable face is {face.name}, centre {centre}")


def _grasp_rotation(box: Cuboid) -> np.ndarray:
    """Tool orientation that closes the fingers across the box's short side.

    The tool points straight down and the fingers straddle the width axis,
    which is the shorter of the two horizontal sides and therefore the one
    that fits between them.
    """
    z = np.array([0.0, 0.0, -1.0])
    y = box.rotation[:, WIDTH]
    # The axis has no direction of its own; picking the +y half keeps the wrist
    # from having to spin most of a turn to reach an equivalent grip.
    if y[1] < 0:
        y = -y
    return np.column_stack((np.cross(y, z), y, z))


def _approach_direction(normal: np.ndarray) -> np.ndarray:
    """Unit vector from a face centre back to where the wrist should sit.

    Straight up for a top face. For a side face, out along the normal but
    tilted upwards, because coming in dead level would put the body of the
    gripper below the table top.
    """
    if normal[2] > 0.9:
        return UP.copy()
    horizontal = np.array([normal[0], normal[1], 0.0])
    horizontal /= np.linalg.norm(horizontal)
    return horizontal * math.cos(SIDE_APPROACH_TILT) + UP * math.sin(SIDE_APPROACH_TILT)


def _describe(box: Cuboid) -> str:
    return " x ".join(f"{float(v) * 100:.0f}" for v in box.size) + " cm"
