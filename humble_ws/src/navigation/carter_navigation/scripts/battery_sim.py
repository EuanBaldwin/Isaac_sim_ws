#!/usr/bin/env python3
"""
Very simple battery model:
full at start (100 %) and drains in proportion to linear+angular speed
"""
import rclpy, math, time
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty as EmptyMsg

class BatterySim(Node):
    def __init__(self):
        super().__init__('battery_sim')
        
        self.declare_parameter('capacity_wh', 1033.0) # nominal capacity of the pack
        self.declare_parameter('idle_w',      95.0)   # W when robot just idles
        self.declare_parameter('move_w',      30.0)  # additional W when moving at 1 m/s
        self.declare_parameter('dock_charged_topic', '/dock_charged')
        self.capacity_wh = self.get_parameter('capacity_wh').value
        self.idle_w      = self.get_parameter('idle_w').value
        self.move_w      = self.get_parameter('move_w').value
        
        self.pub = self.create_publisher(BatteryState, '/battery_status', 10)
        self.sub = self.create_subscription(Odometry, '/chassis/odom', self.update_cb, 10)
        
        dock_charged_topic = self.get_parameter('dock_charged_topic').value
        self.create_subscription(EmptyMsg, dock_charged_topic, self.dock_cb, 1)

        self.last_t = self.get_clock().now()
        self.soc   = 1.0
        self.timer = self.create_timer(1.0, self.publish)

    def update_cb(self, msg):
        now      = self.get_clock().now()
        dt       = (now - self.last_t).nanoseconds / 1e9
        v        = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        power    = self.idle_w + self.move_w * v
        self.soc = max(0.0, self.soc - power*dt/3600.0 / self.capacity_wh)
        self.last_t = now

    def publish(self):
        m = BatteryState()
        m.percentage = float(self.soc)
        m.voltage    = 50.4 * self.soc
        m.present    = True
        self.pub.publish(m)
        
    def dock_cb(self, _msg):
        self.soc = 1.0
        self.last_t = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BatterySim())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

