"""Entry point. Wires the pieces together and runs the workflow once."""

from __future__ import annotations

import threading
import traceback

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .arm.camera import WristCamera
from .arm.motion import Arm
from .scene import PlanningSceneClient
from .task import TouchCuboidsTask


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("cuboid_task")

    # Everything that subscribes or calls a service is built before the node
    # starts being spun, so the executor sees the full set from its first pass.
    arm = Arm(node)
    task = TouchCuboidsTask(node, arm, WristCamera(node), PlanningSceneClient(node))

    # The workflow blocks on services, actions and camera frames, so those have
    # to keep being served from somewhere else. The executor runs on its own
    # thread and the workflow runs on this one.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except Exception:
            node.get_logger().error("executor thread died:\n" + traceback.format_exc())

    threading.Thread(target=spin, daemon=True).start()

    try:
        results = task.run()
        _report(node, results)
    finally:
        executor.shutdown()
        rclpy.shutdown()


def _report(node: Node, results) -> None:
    log = node.get_logger()
    log.info(f"finished: {len(results)} cuboid(s) measured and touched")
    for index, result in enumerate(results, start=1):
        size = " x ".join(f"{float(v) * 100:.1f}" for v in result.cuboid.size)
        log.info(
            f"  {index}. {size} cm, biggest reachable face {result.face.name} "
            f"({result.face.area * 1e4:.1f} cm^2), contact {'yes' if result.contact else 'no'}"
        )


if __name__ == "__main__":
    main()
