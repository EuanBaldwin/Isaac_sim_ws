#!/usr/bin/env python3
"""
ROS 2 node that simulates a simple battery.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty as EmptyMsg


class BatterySim(Node):
    def __init__(self):
        super().__init__('battery_sim')
        self.declare_parameter('capacity_wh', 1033.0) # Nova Carter's default
        self.declare_parameter('idle_w', 95.0)
        self.declare_parameter('move_w', 30.0)
        self.declare_parameter('turn_w', 15.0)
        self.declare_parameter('dock_charged_topic', '/dock_charged')

        self.capacity_wh = self.get_parameter('capacity_wh').value
        self.idle_w = self.get_parameter('idle_w').value
        self.move_w = self.get_parameter('move_w').value
        self.turn_w = self.get_parameter('turn_w').value

        self.nominal_voltage = 50.4
        self.cap_ah = self.capacity_wh / self.nominal_voltage

        self.energy_wh = self.capacity_wh
        self.v = 0.0
        self.w = 0.0
        self.last_power = 0.0

        self.pub = self.create_publisher(BatteryState, '/battery_status', 10)
        self.sub = self.create_subscription(Odometry, '/chassis/odom', self.odom_cb, 10)
        dock_topic = self.get_parameter('dock_charged_topic').value
        self.create_subscription(EmptyMsg, dock_topic, self.dock_cb, 1)

        self.last_t = self.get_clock().now()
        self.timer = self.create_timer(1.0, self.step)

    def odom_cb(self, msg):
        self.v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.w = msg.twist.twist.angular.z

    def dock_cb(self, _msg):
        self.energy_wh = self.capacity_wh
        self.last_power = 0.0

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        power = self.idle_w + self.move_w * self.v + self.turn_w * abs(self.w)
        self.energy_wh = max(0.0, self.energy_wh - power * dt / 3600.0)
        self.last_power = power
        self.publish()
        self.last_t = now

    def soc(self):
        return self.energy_wh / self.capacity_wh

    def publish(self):
        msg = BatteryState()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        perc = self.soc()
        voltage = self.nominal_voltage * perc
        msg.voltage = voltage
        msg.current = (-self.last_power / voltage if voltage > 1e-3 else 0.0)
        msg.charge = self.cap_ah * perc
        msg.capacity = self.cap_ah
        msg.design_capacity = self.cap_ah
        msg.percentage = perc
        msg.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_FULL
            if perc >= 0.995
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BatterySim())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

