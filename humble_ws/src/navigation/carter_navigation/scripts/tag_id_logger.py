#!/usr/bin/env python3
import rclpy
import os
from rclpy.node import Node
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

class TagIdLogger(Node):
    def __init__(self):
        super().__init__('tag_id_logger')
        # Subscribe to the AprilTag detections
        self.sub = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.on_detections,
            10
        )
        self.seen_ids = set()
        self.filepath = os.path.expanduser('~/Documents/detected_tags.txt')
        self.get_logger().info(f"Logging new tag IDs to: {self.filepath}")

    def on_detections(self, msg: AprilTagDetectionArray):
        # Each detection has a field `id` (list of ints, usually length 1)
        for det in msg.detections:
            # grab the first element of det.id
            tag_id = det.id[0] if hasattr(det.id, '__iter__') else det.id
            if tag_id not in self.seen_ids:
                self.seen_ids.add(tag_id)
                with open(self.filepath, 'a') as f:
                    f.write(f"{tag_id}\n")
                self.get_logger().info(f"New tag ID {tag_id} saved")

def main(args=None):
    rclpy.init(args=args)
    node = TagIdLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

