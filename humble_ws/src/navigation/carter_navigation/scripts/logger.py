#!/usr/bin/env python3
"""
ROS 2 node that logs data for formal model V&V.
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
from std_msgs.msg import String


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
        self.run_index = 1
        self.filepath = self.run_dir / f'run_{self.run_index}.csv'

        self.seen_ids: Set[int] = set()
        self._last_status: dict[str, str] = {}
        self._last_soc: float | None = None
        self._awaiting_run_started = True

        # recovery block state
        self.declare_parameter('recovery_block_grace_sec', 1.0)
        self._block_grace: float = float(self.get_parameter('recovery_block_grace_sec').value)
        self._active_recovery_nodes: Set[str] = set()
        self._in_recovery_block: bool = False
        self._block_close_timer = None
        self._last_recovery_leaf_exit_time: str | None = None
        self._last_recovery_leaf_exit_soc: float | None = None

        self.filepath.write_text('sim_time,event,value,soc\n')
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
        self.create_subscription(
            String, '/dock_reason',
            self._on_dock_reason, 10,
        )

        self.create_service(Empty, 'reset_tag_id_log', self.on_reset)
        self.get_logger().info(f"{GREEN}Logging new tag IDs to {self.filepath}{RESET}")

    # helpers
    def _sim_now_str(self) -> str:
        t: Time = self.get_clock().now()
        secs, nsecs = divmod(t.nanoseconds, 1_000_000_000)
        return f"{secs}.{nsecs:09d}"

    def _append_row(self, event: str, value: int | str, sim_time: str | None = None, soc: float | None = None) -> None:
        t_str = sim_time if sim_time is not None else self._sim_now_str()
        soc_val = self._last_soc if soc is None else soc
        soc_str = f'{soc_val:.6f}' if soc_val is not None else 'nan'
        with self.filepath.open('a') as f:
            f.write(f'{t_str},{event},{value},{soc_str}\n')

    @staticmethod
    def _is_recovery_leaf(name: str) -> bool:
        if name in {'Spin', 'BackUp', 'Wait'}:
            return True
        return name.startswith('Clear') and name != 'ClearingActions'

    # recovery block helpers
    def _start_recovery_block_if_needed(self) -> None:
        if not self._in_recovery_block:
            self._in_recovery_block = True
            self._append_row('recovery', 'enter')
            self.get_logger().info(f'{GREEN}Recovery block start{RESET}')

    def _cancel_block_close_timer(self) -> None:
        if self._block_close_timer is not None:
            try:
                self._block_close_timer.cancel()
                self.destroy_timer(self._block_close_timer)
            except Exception:
                pass
            self._block_close_timer = None

    def _schedule_block_close(self) -> None:
        self._cancel_block_close_timer()
        if self._block_grace <= 0.0:
            self._close_recovery_block()
            return
        self._block_close_timer = self.create_timer(self._block_grace, self._on_block_close_timer)

    def _on_block_close_timer(self) -> None:
        self._cancel_block_close_timer()
        self._close_recovery_block()

    def _close_recovery_block(self) -> None:
        if not self._in_recovery_block:
            return
        exit_time = self._last_recovery_leaf_exit_time
        exit_soc  = self._last_recovery_leaf_exit_soc
        self._append_row('recovery', 'exit', sim_time=exit_time, soc=exit_soc)
        self.get_logger().info(f'{GREEN}Recovery block end{RESET}')
        self._in_recovery_block = False
        self._last_recovery_leaf_exit_time = None
        self._last_recovery_leaf_exit_soc = None

    # callbacks
    def _on_battery(self, msg: BatteryState) -> None:
        self._last_soc = msg.percentage
        if self._awaiting_run_started:
            self._append_row('run_started', self.run_index)
            self._awaiting_run_started = False
        if msg.percentage <= 0.0 and prev_soc is not None:
            self._append_row('run_ended', self.run_index)

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

    def _on_dock_reason(self, msg: String) -> None:
        self._append_row('dock', msg.data)
        self.get_logger().info(f"{GREEN}Dock event ({msg.data}) saved{RESET}")

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
                self._start_recovery_block_if_needed()
                self._cancel_block_close_timer()
                if name not in self._active_recovery_nodes:
                    self._active_recovery_nodes.add(name)
                self._append_row('recovery_enter', name)
                self.get_logger().info(f'{GREEN}Entered recovery: {name}{RESET}')

            elif last == 'RUNNING' and curr in {'SUCCESS', 'FAILURE', 'IDLE'}:
                if name in self._active_recovery_nodes:
                    self._active_recovery_nodes.discard(name)
                self._append_row('recovery_exit', name)
                self.get_logger().info(f'{GREEN}Exited  recovery: {name}{RESET}')
                self._last_recovery_leaf_exit_time = self._sim_now_str()
                self._last_recovery_leaf_exit_soc = self._last_soc
                if not self._active_recovery_nodes:
                    self._schedule_block_close()

            elif last in {None, 'IDLE'} and curr in {'SUCCESS', 'FAILURE'}:
                self._start_recovery_block_if_needed()
                self._cancel_block_close_timer()
                self._append_row('recovery_enter', name)
                self._append_row('recovery_exit',  name)
                self.get_logger().info(f'{GREEN}Instant recovery: {name}{RESET}')
                self._last_recovery_leaf_exit_time = self._sim_now_str()
                self._last_recovery_leaf_exit_soc = self._last_soc
                if not self._active_recovery_nodes:
                    self._schedule_block_close()

    def on_reset(self, req: Empty.Request, res: Empty.Response) -> Empty.Response:
        self.get_logger().info(f"{GREEN}Reset request received – clearing seen-ID cache{RESET}")
        self._append_row('run_ended', self.run_index)
        self.seen_ids.clear()
        self._last_soc = None
        self._awaiting_run_started = True
        self.run_index += 1
        self.filepath = self.run_dir / f'run_{self.run_index}.csv'
        self.filepath.write_text('sim_time,event,value,soc\n')
        self._active_recovery_nodes.clear()
        self._cancel_block_close_timer()
        self._in_recovery_block = False
        self._last_recovery_leaf_exit_time = None
        self._last_recovery_leaf_exit_soc = None
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

