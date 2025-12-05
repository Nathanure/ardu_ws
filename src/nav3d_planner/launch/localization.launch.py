#!/usr/bin/env python3
"""
Combined localization launch file.
Launches both KISS-ICP and NDT-GICP localization in one command.

Usage (Terminal 2):
    ros2 launch nav3d_planner localization.launch.py map_file:=/path/to/map.pcd

This replaces the need to run two separate launch files.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # ========================================================================
    # ARGUMENTS
    # ========================================================================
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Path to your saved 3D map (.pcd file)'
    )
    
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/cloud_in',
        description='LiDAR point cloud topic from Gazebo/drone'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Get configurations
    map_file = LaunchConfiguration('map_file')
    lidar_topic = LaunchConfiguration('lidar_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # ========================================================================
    # LAUNCH KISS-ICP (LiDAR Odometry)
    # ========================================================================
    
    kiss_icp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kiss_icp'),
                'launch',
                'odometry.launch.py'
            ])
        ]),
        launch_arguments={
            'topic': lidar_topic,
            'visualize': 'true',
            'base_frame': 'base_link',
            'lidar_odom_frame': 'odom',
            'publish_odom_tf': 'true',
            'invert_odom_tf': 'false',
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # ========================================================================
    # LAUNCH NDT-GICP LOCALIZER (Global Localization)
    # ========================================================================
    
    ndt_gicp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lidar_map_localizer'),
                'launch',
                'ndt_gicp_localization_launch.py'
            ])
        ]),
        launch_arguments={
            'map_file': map_file,
            'odom_topic': '/kiss/odometry',
            'local_map_topic': '/kiss/local_map',
            'corrected_odom_topic': '/corrected_odometry',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'publish_tf': 'false',
            'use_odom_as_initial_guess': 'true',
            # NDT parameters (coarse)
            'ndt_resolution': '2.0',
            'ndt_step_size': '0.1',
            'ndt_epsilon': '0.01',
            'ndt_max_iter': '30',
            'ndt_voxel_leaf_size': '0.5',
            'ndt_fitness_threshold': '1.0',
            # GICP parameters (fine)
            'voxel_leaf_size_local': '0.2',
            'voxel_leaf_size_target': '0.2',
            'map_downsample_voxel': '0.1',
            'gicp_num_threads': '4',
            'gicp_max_corr_dist': '3.0',
            'gicp_corr_randomness': '20',
            'gicp_fitness_threshold': '0.3',
            # Crop box
            'crop_size_x': '30.0',
            'crop_size_y': '30.0',
            'crop_size_z': '10.0',

            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # ========================================================================
    # TF BROADCASTER: map → odom (using corrected_odometry)
    # This bridges the gap that NDT-GICP doesn't publish
    # ========================================================================
    
    tf_broadcaster = Node(
        package='nav3d_planner',
        executable='odom_to_tf_node.py',
        name='map_to_odom_tf',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('input_odom', '/corrected_odometry')
        ],
        output='screen'
    )


    return LaunchDescription([
        # Arguments
        map_file_arg,
        lidar_topic_arg,
        use_sim_time_arg,
        
        # Launch files
        kiss_icp_launch,
        ndt_gicp_launch,
        tf_broadcaster
    ])