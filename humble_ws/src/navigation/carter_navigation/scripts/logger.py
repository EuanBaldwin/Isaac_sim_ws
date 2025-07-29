#!/usr/bin/env python3
"""
ROS 2 node that logs every new AprilTag ID it sees and waypoint it reaches,
but ‘forgets’ them whenever the /reset_tag_id_log service is called.
"""

from datetime import datetime
from pathlib import Path
from typing import Set

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from std_msgs.msg import UInt32
from std_srvs.srv import Empty

class Logger(Node):
    def __init__(self) -> None:
        super().__init__(
            'logger',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )

        script_path = Path(__file__).resolve()
        try:
            workspace_root = next(p for p in script_path.parents if p.name.endswith('_ws'))
        except StopIteration:
            workspace_root = script_path.parents[4]
            
            
        launch_ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_dir = workspace_root / 'data_log' / launch_ts
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.run_index = 0
        self.filepath = self.run_dir / f'run_{self.run_index}.csv'
        self.filepath.write_text('sim_time,event,value\n')
        self.declare_parameter('log_path', str(self.filepath))

        self.seen_ids: Set[int] = set()

        # subscriber for AprilTag detections
        self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.on_detections,
            10,
        )
        
        # subscriber for Waypoints
        self.create_subscription(
            UInt32,
            '/waypoint_reached',
            self._on_waypoint,
            10,
        )

        # service that clears the seen‑ID cache
        self.create_service(Empty, 'reset_tag_id_log', self.on_reset)

        self.get_logger().info(f"Logging new tag IDs to {self.filepath}")
        self.get_logger().info("Call `ros2 service call /reset_tag_id_log std_srvs/srv/Empty` "
                               "to reset the ID cache")

    # helpers
    def _sim_now_str(self) -> str:
        t: Time = self.get_clock().now()
        secs, nsecs = divmod(t.nanoseconds, 1_000_000_000)
        return f"{secs}.{nsecs:09d}"

    def _append_row(self, event: str, value: int | str) -> None:
        with self.filepath.open('a') as f:
            f.write(f'{self._sim_now_str()},{event},{value}\n')

    # callbacks
    def on_detections(self, msg: AprilTagDetectionArray) -> None:
        for det in msg.detections:
            tag_id = det.id[0] if hasattr(det.id, '__iter__') else det.id
            if tag_id in self.seen_ids:
                continue

            self.seen_ids.add(tag_id)
            self._append_row('tag', tag_id)
            self.get_logger().info(f"New tag ID {tag_id} saved")
            
    def _on_waypoint(self, msg: UInt32) -> None:
        self._append_row('waypoint', msg.data)
        self.get_logger().info(f"Waypoint {msg.data} saved")

    def on_reset(self, req: Empty.Request, res: Empty.Response) -> Empty.Response:
        self.get_logger().info("Reset request received – clearing seen‑ID cache")
        self.seen_ids.clear()
        self.run_index += 1
        self.filepath = self.run_dir / f'run_{self.run_index}.csv'
        self.filepath.write_text('sim_time,event,value\n')
        self.get_logger().info(f"Now logging to {self.filepath}")
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Logger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

