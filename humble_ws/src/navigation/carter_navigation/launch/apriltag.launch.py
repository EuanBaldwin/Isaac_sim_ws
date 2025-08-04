# File: launch/apriltag.launch.py

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag_detector',
        remappings=[
#            ('image', '/front_stereo_camera/left/image_raw'),
#            ('camera_info', '/front_stereo_camera/left/camera_info'),
            ('image',       '/apriltag_gate/image_raw'),
            ('camera_info', '/apriltag_gate/camera_info'),
        ],
        parameters=[{
            'size': 0.25,
            'max_tags': 588,
            'tile_size': 2,
            'tag_family': 'tag36h11',
            'max_hamming': 0,
            'decision_margin_thresh': 50.0,
        }]
    )

    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container_mt',
        name='apriltag_container',
        namespace='',
        composable_node_descriptions=[apriltag_node],
        output='screen',
    )

    return launch.LaunchDescription([apriltag_container])

