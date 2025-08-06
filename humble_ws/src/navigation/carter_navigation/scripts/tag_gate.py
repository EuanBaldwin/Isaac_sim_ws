#!/usr/bin/env python3

'''
carter_navigation/tag_gate.py
Republish images only while “at a waypoint”.
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32
from sensor_msgs.msg import Image, CameraInfo

class TagGate(Node):
    def __init__(self):
        super().__init__('tag_gate')

        self.declare_parameter('active_duration', 2.0) # seconds
        self.active_duration = self.get_parameter(
            'active_duration').get_parameter_value().double_value

        qos_sensor = rclpy.qos.QoSProfile(
            depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(
            Image, '/front_stereo_camera/left/image_raw',
            self._img_cb, qos_sensor
        )
        
        self.create_subscription(
            CameraInfo, '/front_stereo_camera/left/camera_info',
            self._info_cb, 10
        )
        
        self.create_subscription(
            UInt32, '/waypoint_reached',
            self._wp_cb, 10
        )

        self.pub_img  = self.create_publisher(
            Image, '/apriltag_gate/image_raw', 10
        )
        
        self.pub_info = self.create_publisher(
            CameraInfo, '/apriltag_gate/camera_info', 10
        )

        self._enabled = False
        self._disable_timer = None


    def _wp_cb(self, msg: UInt32):
        wp = msg.data
        if wp == 0:
            return

        self.get_logger().info(f'Waypoint {wp} reached - enabling AprilTag detector '
                               f'for {self.active_duration:.1f}s')
        self._enabled = True

        # restart timer
        if self._disable_timer:
            self._disable_timer.cancel()
        self._disable_timer = self.create_timer(self.active_duration, self._disable)

    def _disable(self):
        self.get_logger().info('Disabling AprilTag detector until next waypoint')
        self._enabled = False
        if self._disable_timer:
            self._disable_timer.cancel()
            self._disable_timer = None


    def _img_cb(self, msg: Image):
        if self._enabled:
            self.pub_img.publish(msg)

    def _info_cb(self, msg: CameraInfo):
        if self._enabled:
            self.pub_info.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(TagGate())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

