#!/usr/bin/env python3
"""
ROS 2 node that logs data for formal model V&V.
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
from nav2_msgs.msg import BehaviorTreeLog, BehaviorTreeStatusChange
from sensor_msgs.msg import BatteryState


GREEN  = "\033[1;32m"
RESET  = "\033[0m"

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
        
        self.seen_ids: Set[int] = set()
        self._in_recovery = False
        self._last_status: dict[str, str] = {}
        self._last_soc: float | None = None
        
        self.filepath.write_text('sim_time,event,value,soc\n')
        self._append_row('run_started', self.run_index)
        self.declare_parameter('log_path', str(self.filepath))

        # Subscribers
        self.create_subscription(
            AprilTagDetectionArray, '/tag_detections',
            self.on_detections, 10,
        )
        
        self.create_subscription(
            UInt32, '/waypoint_reached',
            self._on_waypoint, 10,
        )
        
        self.create_subscription(
            BehaviorTreeLog, '/behavior_tree_log',
            self._on_bt_log, 10,
        )
        
        self.create_subscription(
            BatteryState, '/battery_status',
            self._on_battery, 10,
        )

        # service that clears the seen‑ID cache
        self.create_service(Empty, 'reset_tag_id_log', self.on_reset)
        self.get_logger().info(f"{GREEN}Logging new tag IDs to {self.filepath}{RESET}")

    # helpers
    def _sim_now_str(self) -> str:
        t: Time = self.get_clock().now()
        secs, nsecs = divmod(t.nanoseconds, 1_000_000_000)
        return f"{secs}.{nsecs:09d}"

    def _append_row(self, event: str, value: int | str) -> None:
        soc_str = f'{self._last_soc:.6f}' if self._last_soc is not None else 'nan'
        with self.filepath.open('a') as f:
            f.write(f'{self._sim_now_str()},{event},{value},{soc_str}\n')
    
    @staticmethod
    def _is_recovery_leaf(name: str) -> bool:
        if name in {'Spin', 'BackUp', 'Wait'}:
            return True
        return name.startswith('Clear') and name != 'ClearingActions'

    # callbacks
    def _on_battery(self, msg: BatteryState) -> None:
        self._last_soc = msg.percentage

    def on_detections(self, msg: AprilTagDetectionArray) -> None:
        for det in msg.detections:
            tag_id = det.id[0] if hasattr(det.id, '__iter__') else det.id
            if tag_id in self.seen_ids:
                continue

            self.seen_ids.add(tag_id)
            self._append_row('tag', tag_id)
            self.get_logger().info(f"{GREEN}New tag ID {tag_id} saved{RESET}")
            
    def _on_waypoint(self, msg: UInt32) -> None:
        self._append_row('waypoint', msg.data)
        self.get_logger().info(f"{GREEN}Waypoint {msg.data} saved{RESET}")
        
    def _on_bt_log(self, msg: BehaviorTreeLog) -> None:
        for ev in msg.event_log:
            name, prev, curr = ev.node_name, ev.previous_status, ev.current_status

            if not self._is_recovery_leaf(name):
                 continue

            last = self._last_status.get(name)
            if last == curr:
                continue
            self._last_status[name] = curr

            if curr == 'RUNNING':
                self._append_row('recovery_enter', name)
                self.get_logger().info(f'{GREEN}Entered recovery: {name}{RESET}')
            elif last == 'RUNNING' and curr in {'SUCCESS', 'FAILURE', 'IDLE'}:
                self._append_row('recovery_exit',  name)
                self.get_logger().info(f'{GREEN}Exited  recovery: {name}{RESET}')
            elif last in {None, 'IDLE'} and curr in {'SUCCESS', 'FAILURE'}:
                self._append_row('recovery_enter', name)
                self._append_row('recovery_exit',  name)
                self.get_logger().info(f'{GREEN}Instant recovery: {name}{RESET}')

    def on_reset(self, req: Empty.Request, res: Empty.Response) -> Empty.Response:
        self.get_logger().info(f"{GREEN}Reset request received – clearing seen‑ID cache{RESET}")
        self._append_row('run_ended', self.run_index)
        self.seen_ids.clear()
        self.run_index += 1
        self.filepath = self.run_dir / f'run_{self.run_index}.csv'
        self.filepath.write_text('sim_time,event,value,soc\n')
        self._append_row('run_started', self.run_index)
        self.get_logger().info(f"{GREEN}Now logging to {self.filepath}{RESET}")
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

