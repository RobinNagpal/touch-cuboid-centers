"""What MoveIt is told about the world.

MoveIt plans against its own picture of the cell, not against Gazebo. Without
this the planner would happily sweep the arm through the table, so the table
goes in once at startup and the cuboids go in every time they are measured or
moved.
"""

from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from .cell import TABLE_CENTRE_XY, TABLE_SIZE, TABLE_TOP_Z, WORLD_FRAME
from .geometry import Cuboid
from .transforms import frame, make_pose

# Links the carried box is allowed to be touching without that counting as a
# collision: the ones doing the holding.
GRIPPER_LINKS = ["gripper_body", "left_finger", "right_finger"]


class PlanningSceneClient:
    def __init__(self, node: Node) -> None:
        self._node = node
        self._client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
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

    def add_table(self) -> None:
        pose = Pose()
        pose.position.x, pose.position.y = TABLE_CENTRE_XY
        pose.position.z = TABLE_TOP_Z - TABLE_SIZE[2] / 2.0
        pose.orientation.w = 1.0
        self._apply([_box("table", pose, TABLE_SIZE)])

    def set_cuboids(self, cuboids: dict[str, Cuboid]) -> None:
        """Make the scene hold exactly these cuboids and no others.

        A cuboid the arm is about to grasp has to be dropped from the scene
        first, or the planner refuses to let the fingers reach it.
        """
        objects = [_box(name, make_pose(box.centre, box.rotation), box.size) for name, box in cuboids.items()]
        objects += [_removal(name) for name in self._known - set(cuboids)]
        self._apply(objects)
        self._known = set(cuboids)

    def attach(self, name: str, cuboid: Cuboid, tool_pose: np.ndarray, link: str = "gripper_body") -> None:
        """Say that the arm is now holding this box.

        Until the box is attached, MoveIt plans as if the gripper were empty
        and will happily sweep whatever is in it through the boxes already set
        down on the done side.
        """
        in_tool = np.linalg.inv(tool_pose) @ frame(cuboid.centre, cuboid.rotation)
        held = _box(name, make_pose(in_tool[:3, 3], in_tool[:3, :3]), cuboid.size)
        held.header.frame_id = link
        self._known.discard(name)
        self._apply([], [AttachedCollisionObject(link_name=link, object=held, touch_links=GRIPPER_LINKS)])

    def detach(self, name: str, link: str = "gripper_body") -> None:
        """Say that the arm has let go.

        Detaching an object in MoveIt drops it back into the world where it was
        last held, which is between the fingers, so every move afterwards is
        refused for driving the gripper through it. The second call clears that
        copy away. Where the box really ended up is put back by set_cuboids,
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


def _box(name: str, pose: Pose, size) -> CollisionObject:
    obj = CollisionObject()
    obj.id = name
    obj.header.frame_id = WORLD_FRAME
    obj.operation = CollisionObject.ADD
    obj.primitives = [
        SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in np.asarray(size)])
    ]
    obj.primitive_poses = [pose]
    return obj
