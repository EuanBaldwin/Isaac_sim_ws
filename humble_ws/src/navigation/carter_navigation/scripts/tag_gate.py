#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32
from sensor_msgs.msg import Image, CameraInfo


class TagGate(Node):
    def __init__(self):
        super().__init__('tag_gate')

        self.declare_parameter('active_duration', 2.0)
        self._active_duration = float(self.get_parameter('active_duration').value)

        qos_sensor = rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)

        self.sub_img  = self.create_subscription(
            Image, '/front_stereo_camera/left/image_raw',
            self._img_cb, qos_sensor
        )

        self.sub_info = self.create_subscription(
            CameraInfo, '/front_stereo_camera/left/camera_info',
            self._info_cb, 10
        )

        self.sub_wp = self.create_subscription(
            UInt32, '/waypoint_reached',
            self._wp_cb, 10
        )

        self.pub_img = self.create_publisher(Image, '/apriltag_gate/image_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/apriltag_gate/camera_info', 10)

        self._enabled = False
        self._disable_timer = None

    def _wp_cb(self, msg: UInt32):
        if msg.data == 0:
            return

        self.get_logger().info(
            f'Waypoint {msg.data} reached - enabling detector for {self._active_duration:.1f}s')
        self._enabled = True

        if self._disable_timer:
            self._disable_timer.cancel()

        self._disable_timer = self.create_timer(self._active_duration, self._disable)

    def _disable(self):
        self.get_logger().info('Disabling AprilTag detector until next waypoint')
        self._enabled = False
        if self._disable_timer:
            self._disable_timer.cancel()
            self._disable_timer = None

    def _img_cb(self, msg): # Image
        if self._enabled:
            self.pub_img.publish(msg)

    def _info_cb(self, msg): # CameraInfo
        if self._enabled:
            self.pub_info.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(TagGate())
    rclpy.shutdown()

