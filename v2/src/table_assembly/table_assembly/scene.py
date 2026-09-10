"""What MoveIt is told about the room.

MoveIt plans against its own picture of the room, not against Gazebo, and at
the start that picture is empty: the arm has not looked yet. Everything in it
is put there from measurements — the floor once it has been found, the wall
and the parts every time they are seen or moved.

There are two pictures to keep up to date, not one. ``move_group`` holds one,
and it is what straight-line moves are checked against. The planner inside
the task node holds the other, and it is what free moves are planned against.
The in-process one does not follow move_group's — it listens for scene
changes on a topic nothing here publishes to — so every change is applied to
both, one through move_group's service and one directly.
"""

from __future__ import annotations

import numpy as np
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from .arm.dimensions import WORLD_FRAME
from .geometry import Box
from .messages import make_pose

# Links a carried part is allowed to be touching without that counting as a
# collision: the ones doing the holding.
GRIPPER_LINKS = ["gripper_body", "left_finger", "right_finger"]

# The floor goes into the scene a few millimetres below where it was measured.
# The arm's base stands on the floor, and a floor measured a millimetre high
# would otherwise put the base inside it and have every plan refused.
FLOOR_SINK = 0.005
FLOOR_SIZE = (4.0, 4.0, 0.1)


class PlanningSceneClient:
    def __init__(self, node: Node, local_monitor) -> None:
        """``local_monitor`` is the planning scene monitor of the task's own planner."""
        self._node = node
        self._client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self._local = local_monitor
        self._known: set[str] = set()

    def wait_until_ready(self, timeout: float = 120.0) -> None:
        """Block until move_group is serving edits to the planning scene."""
        if not self._client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"/apply_planning_scene did not come up within {timeout:.0f}s")

    def _apply(self, objects: list[CollisionObject], attached=()) -> None:
        if not self._client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/apply_planning_scene is not available")
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene(is_diff=True)
        request.scene.world.collision_objects = objects
        request.scene.robot_state.attached_collision_objects = list(attached)
        request.scene.robot_state.is_diff = True
        self._client.call(request)

        with self._local.read_write() as scene:
            for obj in objects:
                scene.apply_collision_object(obj)
            for held in attached:
                scene.process_attached_collision_object(held)

    def set_floor(self, floor_z: float) -> None:
        centre = np.array([0.0, 0.0, floor_z - FLOOR_SINK - FLOOR_SIZE[2] / 2.0])
        self._apply([_box("floor", Box(centre, np.eye(3), FLOOR_SIZE))])

    def set_objects(self, boxes: dict[str, Box]) -> None:
        """Make the scene hold exactly these boxes, besides the floor.

        A part the arm is about to grasp has to be left out first, or the
        planner refuses to let the fingers reach it.
        """
        objects = [_box(name, box) for name, box in boxes.items()]
        objects += [_removal(name) for name in self._known - set(boxes)]
        self._apply(objects)
        self._known = set(boxes)

    def attach(self, name: str, part: Box, tool_pose: np.ndarray, link: str = "gripper_body") -> None:
        """Say that the arm is now holding this part.

        Until the part is attached, MoveIt plans as if the gripper were empty
        and will happily sweep whatever is in it through the wall.
        """
        in_tool = np.linalg.inv(tool_pose) @ part.pose
        held = _box(name, Box(in_tool[:3, 3], in_tool[:3, :3], part.size))
        held.header.frame_id = link
        self._known.discard(name)
        self._apply([], [AttachedCollisionObject(link_name=link, object=held, touch_links=GRIPPER_LINKS)])

    def detach(self, name: str, link: str = "gripper_body") -> None:
        """Say that the arm has let go.

        Detaching an object in MoveIt drops it back into the world where it was
        last held, which is between the fingers, so every move afterwards is
        refused for driving the gripper through it. The second call clears that
        copy away. Where the part really ended up is put back by set_objects,
        once it has been looked at again.
        """
        self._apply([], [AttachedCollisionObject(link_name=link, object=_removal(name))])
        self._apply([_removal(name)])


def _removal(name: str) -> CollisionObject:
    obj = CollisionObject()
    obj.id = name
    obj.header.frame_id = WORLD_FRAME
    obj.operation = CollisionObject.REMOVE
    return obj


def _box(name: str, box: Box) -> CollisionObject:
    obj = CollisionObject()
    obj.id = name
    obj.header.frame_id = WORLD_FRAME
    obj.operation = CollisionObject.ADD
    obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in box.size])]
    obj.primitive_poses = [make_pose(box.centre, box.rotation)]
    return obj
