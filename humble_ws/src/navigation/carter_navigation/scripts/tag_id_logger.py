#!/usr/bin/env python3
"""
ROS 2 node that logs every new AprilTag ID it sees.
"""

from datetime import datetime
from pathlib import Path
from typing import Set

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray


class TagIdLogger(Node):
    def __init__(self) -> None:
        super().__init__(
            'tag_id_logger',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )

        # Find the workspace root (…/humble_ws)
        script_path = Path(__file__).resolve()
        try:
            workspace_root = next(p for p in script_path.parents if p.name.endswith('_ws'))
        except StopIteration:
            # Fallback: move four levels up from …/install/<pkg>/lib/<pkg>/tag_id_logger.py
            workspace_root = script_path.parents[4]

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        default_log = workspace_root / 'data_log' / f'{timestamp}.csv'
        default_log.parent.mkdir(parents=True, exist_ok=True)

        # Allow an override via ROS 2 parameter
        self.declare_parameter('log_path', str(default_log))
        self.filepath = Path(self.get_parameter('log_path').get_parameter_value().string_value)
        self.filepath.parent.mkdir(parents=True, exist_ok=True)

        if not self.filepath.exists():
            with self.filepath.open('w') as f:
                f.write('sim_time,id\n')

        self.seen_ids: Set[int] = set()

        self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.on_detections,
            10,
        )

        self.get_logger().info(f"Logging new tag IDs to {self.filepath}")

    def _sim_now_str(self) -> str:
        t: Time = self.get_clock().now()
        secs, nsecs = divmod(t.nanoseconds, 1_000_000_000)
        return f"{secs}.{nsecs:09d}"

    def on_detections(self, msg: AprilTagDetectionArray) -> None:
        for det in msg.detections:
            tag_id = det.id[0] if hasattr(det.id, '__iter__') else det.id
            if tag_id in self.seen_ids:
                continue

            self.seen_ids.add(tag_id)
            with self.filepath.open('a') as f:
                f.write(f'{self._sim_now_str()},{tag_id}\n')

            self.get_logger().info(f"New tag ID {tag_id} saved")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TagIdLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

