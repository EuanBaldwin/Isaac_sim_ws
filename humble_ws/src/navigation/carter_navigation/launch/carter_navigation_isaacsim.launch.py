## Copyright (c) 2025, NVIDIA CORPORATION. All rights reserved.
## NVIDIA CORPORATION and its licensors retain all intellectual property
## and proprietary rights in and to this software, related documentation
## and any modifications thereto.  Any use, reproduction, disclosure or
## distribution of this software and related documentation without an express
## license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessIO
from launch.substitutions import FindExecutable

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    
    # enables visualisation of carter's meshes in rviz
#    carter_description_launch_file = os.path.join(
#    	get_package_share_directory("carter_navigation"), "launch", "nova_carter_description_isaac_sim.launch.py",
#     )

    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("carter_navigation"), "maps", "carter_warehouse_navigation.yaml"
        ),
    )

    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("carter_navigation"), "params", "carter_navigation_params.yaml"
        ),
    )


    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    rviz_config_dir = os.path.join(get_package_share_directory("carter_navigation"), "rviz2", "carter_navigation.rviz")
    
    apriltag_launch_file = os.path.join(
        get_package_share_directory("carter_navigation"), "launch", "apriltag.launch.py"
    )

    ld_apriltag = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([apriltag_launch_file]),
    )

    tag_id_logger_node = Node(
        package="carter_navigation",
        executable="tag_id_logger.py",
        name="tag_id_logger",
        output="screen",
    )

    ld_automatic_goal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("isaac_ros_navigation_goal"), "launch", "isaac_ros_navigation_goal.launch.py"
                ),
            ]
        ),
    )

    

    def execute_second_node_if_condition_met(event, second_node_action):
        output = event.text.decode().strip()
        # Look for fully loaded message from Isaac Sim. Only applicable in Gui mode.
        if "Stage loaded and simulation is playing." in output:
            # Log a message indicating the condition has been met
            print("Condition met, launching the second node.")
            
            # If Nav2 takes additional time to initialize, uncomment the lines below to add a delay of 10 seconds (or any desired duration) before launching the second_node_action
            import time
            time.sleep(10)
            return second_node_action


    return LaunchDescription(
        [
            # Declaring the Isaac Sim scene path. 'gui' launch argument is already used withing run_isaac_sim.launch.py
            DeclareLaunchArgument("gui", default_value='~/Documents/IsaacSim-ros_workspaces/humble_ws/src/navigation/carter_navigation/maps/inspection_robot.usd', description="Path to isaac sim scene"),

            # Include Isaac Sim launch file from isaacsim package with given launch parameters.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                            get_package_share_directory("isaacsim"), "launch", "run_isaacsim.launch.py"
                        ),
                    ]
                ),
                launch_arguments={
                    'version': '4.5.0',
                    'play_sim_on_start': 'true',
#                    'headless': '',
#                    "custom_args": (
#            		"--/app/omni.graph.scriptnode/opt_in=True "
#            		"--/log/level=error "
#            		"--/log/fileLogLevel=error "
#            		"--/log/outputStreamLevel=error"
#        		),
                }.items(),
            ),
            
            DeclareLaunchArgument("map", default_value=map_dir, description="Full path to map file to load"),
            DeclareLaunchArgument(
                "params_file", default_value=param_dir, description="Full path to param file to load"
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use simulation (Omniverse Isaac Sim) clock if true"
            ),
#            IncludeLaunchDescription(
#                PythonLaunchDescriptionSource(carter_description_launch_file),
#                launch_arguments={"use_sim_time": use_sim_time}.items(),
#            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
                launch_arguments={"namespace": "", "use_namespace": "False", "rviz_config": rviz_config_dir}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_launch_dir, "/bringup_launch.py"]),
                launch_arguments={"map": map_dir, "use_sim_time": use_sim_time, "params_file": param_dir}.items(),
            ),
            Node(
                package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                remappings=[('cloud_in', ['/front_3d_lidar/lidar_points']),
                            ('scan', ['/scan'])],
                parameters=[{
                    'target_frame': 'front_3d_lidar',
                    'transform_tolerance': 0.01,
                    'min_height': -0.4,
                    'max_height': 1.5,
                    'angle_min': -1.5708,  # -M_PI/2
                    'angle_max': 1.5708,  # M_PI/2
                    'angle_increment': 0.0087,  # M_PI/360.0
                    'scan_time': 0.3333,
                    'range_min': 0.05,
                    'range_max': 100.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0,
                    # 'concurrency_level': 1,
                }],
                name='pointcloud_to_laserscan'
            ),

            # Launch automatic goal generator node when Isaac Sim has finished loading.
            RegisterEventHandler(
                OnProcessIO(
                    on_stdout=lambda event: execute_second_node_if_condition_met(event, ld_automatic_goal)
                )
            ),
            ld_apriltag,
            tag_id_logger_node,
        ]
    )
