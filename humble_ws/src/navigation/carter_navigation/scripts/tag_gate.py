#!/usr/bin/env python3

'''
carter_navigation/tag_gate.py
Republish exactly N image frames (and their CameraInfo) after each waypoint.
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32
from sensor_msgs.msg import Image, CameraInfo

class TagGate(Node):
    def __init__(self):
        super().__init__('tag_gate')

        # How many frames to let through per waypoint
        self.declare_parameter('frames_to_allow', 10)
        self.frames_to_allow = self.get_parameter(
            'frames_to_allow').get_parameter_value().integer_value or 10

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

        self.pub_img  = self.create_publisher(Image, '/apriltag_gate/image_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/apriltag_gate/camera_info', 10)

        self._enabled = False
        self._frames_remaining = 0
        self._last_info: CameraInfo | None = None

    def _wp_cb(self, msg: UInt32):
        wp = msg.data
        if wp == 0:
            return  # ignore "dock" or sentinel waypoint if you use 0 that way

        self._enabled = True
        self._frames_remaining = self.frames_to_allow
        self.get_logger().info(
            f'Waypoint {wp} reached - enabling AprilTag detector for '
            f'{self._frames_remaining} frames'
        )

    def _disable(self):
        if self._enabled:
            self.get_logger().info('Disabling AprilTag detector until next waypoint')
        self._enabled = False
        self._frames_remaining = 0

    def _img_cb(self, msg: Image):
        if not self._enabled or self._frames_remaining <= 0:
            return

        # Forward the image
        self.pub_img.publish(msg)

        # Forward the most recent CameraInfo once per image (if we have one)
        if self._last_info is not None:
            self.pub_info.publish(self._last_info)

        # Count down and disable when we've forwarded the requested number
        self._frames_remaining -= 1
        if self._frames_remaining == 0:
            self._disable()

    def _info_cb(self, msg: CameraInfo):
        # Keep only the latest CameraInfo; we forward it in _img_cb to keep 1:1 with images
        self._last_info = msg


def main():
    rclpy.init()
    rclpy.spin(TagGate())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

