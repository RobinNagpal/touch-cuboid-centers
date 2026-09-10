"""Entry point. Wires the pieces together and runs the workflow once."""

from __future__ import annotations

import math
import threading
import traceback

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .arm.camera import WristCamera
from .arm.motion import Arm, MotionFailed
from .geometry import describe
from .scene import PlanningSceneClient
from .task import AssembleTableTask, Result, TaskFailed


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("assembly_task")

    # Everything that subscribes or calls a service is built before the node
    # starts being spun, so the executor sees the full set from its first pass.
    arm = Arm(node)
    scene = PlanningSceneClient(node, arm.planning_scene_monitor)
    task = AssembleTableTask(node, arm, WristCamera(node), scene)

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
        _report(node, task.run())
    except (TaskFailed, MotionFailed) as failure:
        node.get_logger().error(f"the table was not built: {failure}")
    finally:
        executor.shutdown()
        rclpy.shutdown()


def _report(node: Node, result: Result) -> None:
    log = node.get_logger()
    log.info("finished")
    log.info(f"  table top measured at {describe(result.top.size)}")
    log.info(f"  {result.legs_standing} legs standing, {result.leg_length * 100:.1f} cm long")
    check = result.check
    if check is None:
        log.warning("  the finished table could not be checked")
        return
    log.info(f"  table as built: {describe(check.size)}")
    log.info(f"  height {check.size[2] * 100:.1f} cm, expected {check.expected_height * 100:.1f} cm")
    log.info(f"  top {math.degrees(check.tilt):.1f} degrees off level")
    log.info(f"  centre {check.offset * 1000:.0f} mm from where it was planned")
    log.info(f"  verdict: {'a table' if check.good else 'NOT a proper table'}")


if __name__ == "__main__":
    main()
