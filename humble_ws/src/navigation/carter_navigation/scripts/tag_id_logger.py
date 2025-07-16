#!/usr/bin/env python3
"""
ROS 2 node that logs every new AprilTag ID it sees.
"""

import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray


class TagIdLogger(Node):
    def __init__(self):
        super().__init__('tag_id_logger')

        # Where to save the log
        self.filepath = os.path.expanduser(
            '~/Documents/IsaacSim-ros_workspaces/humble_ws/src/navigation/'
            'carter_navigation/scripts/detected_tags.txt'
        )

        # Write a run delimiter so the file shows distinct sessions
        self._write_run_header()

        # Keep track of IDs we've already recorded this session
        self.seen_ids = set()

        # Subscribe to the AprilTag detections topic
        self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.on_detections,
            10
        )

        self.get_logger().info(f"Logging new tag IDs to {self.filepath}")

    # ------------------------------------------------------------------

    def _now_str(self) -> str:
        """Return current system time as YYYY-MM-DD HH:MM:SS."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    def _write_run_header(self):
        """Append a run-separator line to the log file."""
        header = f"\n=== Run started {self._now_str()} ===\n"
        os.makedirs(os.path.dirname(self.filepath), exist_ok=True)
        with open(self.filepath, 'a') as f:
            f.write(header)

    # ------------------------------------------------------------------

    def on_detections(self, msg: AprilTagDetectionArray):
        """Callback every time the AprilTag node publishes detections."""
        for det in msg.detections:
            tag_id = det.id[0] if hasattr(det.id, '__iter__') else det.id
            if tag_id in self.seen_ids:
                continue

            self.seen_ids.add(tag_id)
            line = f"{self._now_str()}  |  id: {tag_id}\n"
            with open(self.filepath, 'a') as f:
                f.write(line)

            self.get_logger().info(f"New tag ID {tag_id} saved")

# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = TagIdLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

