#!/usr/bin/env python3
"""
Launch file for 3D navigation with NDT-GICP localization.

COMPLETE SYSTEM ARCHITECTURE:
=============================

Terminal 1: Gazebo Simulation
    ros2 launch ardupilot_gz_bringup iris_maze.launch.py rviz:=false use_gz_tf:=true lidar_dim:=3
    Outputs: /cloud_in (3D LiDAR scans)

Terminal 2: Localization Stack  
    A) KISS-ICP (odometry from LiDAR)
       ros2 launch kiss_icp odometry.launch.py topic:=/cloud_in
       Outputs: /kiss/odometry, /kiss/local_map
    
    B) NDT-GICP Localizer (global localization)
       ros2 launch lidar_map_localizer ndt_gicp_localization_launch.py map_file:=/path/to/map.pcd
       Input: /kiss/odometry, /kiss/local_map
       Output: /corrected_odometry (localized pose on saved map)

Terminal 3: This Launch File (Autonomous Navigation)
    Inputs: /corrected_odometry (from Terminal 2), /map_pointcloud (loaded PCD)
    Outputs: /planned_path (to ArduPilot), /planning_markers (RViz visualization)

DATA FLOW:
    Drone → /cloud_in → KISS-ICP → /kiss/odometry + /kiss/local_map 
    → NDT-GICP → /corrected_odometry → Planner → /planned_path → ArduPilot
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ========================================================================
    # DECLARE ARGUMENTS
    # ========================================================================
    
    map_pcd_file_arg = DeclareLaunchArgument(
        'map_pcd_file',
        default_value='',
        description='REQUIRED: Path to your pre-built 3D map (.pcd file)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Gazebo'
    )
    
    enable_ardupilot_arg = DeclareLaunchArgument(
        'enable_ardupilot',
        default_value='true',
        description='Enable ArduPilot MAVLink interface for autonomous flight'
    )
    
    ardupilot_connection_arg = DeclareLaunchArgument(
        'ardupilot_connection',
        default_value='udp:127.0.0.1:14550',
        description='ArduPilot SITL connection string'
    )
    
    localization_topic_arg = DeclareLaunchArgument(
        'localization_topic',
        default_value='/corrected_odometry',
        description='Topic for localized pose (from NDT-GICP localizer)'
    )
    
    # Get configurations
    map_pcd_file = LaunchConfiguration('map_pcd_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_ardupilot = LaunchConfiguration('enable_ardupilot')
    ardupilot_connection = LaunchConfiguration('ardupilot_connection')
    localization_topic = LaunchConfiguration('localization_topic')
    
    # ========================================================================
    # NODE 1: MAP PUBLISHER
    # ========================================================================
    # Loads your pre-built 3D map and publishes it as a static point cloud
    # This is used by the planner for collision checking
    pcd_publisher = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='pcd_map_publisher',
        parameters=[{
            'file_name': map_pcd_file,
            'frame_id': 'map',
            'latch': True,  # Keep publishing for late subscribers
            'publishing_rate': 0.1,
            'use_sim_time': use_sim_time
        }],
        # remappings=[
        #     ('cloud_pcd', '/map_pointcloud')
        # ],
        output='screen'
    )
    
    # ========================================================================
    # NODE 1B: POINT CLOUD FRAME FIXER
    # ========================================================================
    # Fixes the frame_id from base_link to map
    pointcloud_fixer = Node(
        package='nav3d_planner',
        executable='pointcloud_fixer.py',
        name='pointcloud_fixer',
        parameters=[{
            'target_frame': 'map',
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # ========================================================================
    # NODE 2: ODOMETRY TO POSE CONVERTER
    # ========================================================================
    # Converts /corrected_odometry (nav_msgs/Odometry) to /current_pose (PoseStamped)
    # The planner expects PoseStamped format
    odom_to_pose_converter = Node(
        package='nav3d_planner',
        executable='odom_to_pose_node.py',
        name='odom_to_pose_converter',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('input_odom', localization_topic),
            ('output_pose', '/current_pose')
        ],
        output='screen'
    )
    
    # ========================================================================
    # NODE 3: 3D PLANNER
    # ========================================================================
    # Plans collision-free 3D paths using RRT algorithm
    # 
    # Subscribes to:
    #   - /map_pointcloud (static map for collision checking)
    #   - /current_pose (drone's localized position)
    #   - /goal_pose (from RViz or goal_publisher tool)
    # 
    # Publishes:
    #   - /planned_path (3D waypoints)
    #   - /planning_markers (visualization for RViz)
    planner_node = Node(
        package='nav3d_planner',
        executable='planner_3d_node.py',
        name='planner_3d',
        output='screen',
        parameters=[{
            'resolution': 0.2,          # Collision checking voxel size (m)
            'robot_radius': 0.5,        # Safety radius around drone (m)
            'max_planning_time': 5.0,   # Planning timeout (seconds)
            'step_size': 0.5,           # RRT step size (m)
            'goal_tolerance': 0.5,      # Goal reached threshold (m)
            'use_sim_time': use_sim_time
        }]
    )
    
    # ========================================================================
    # NODE 4: ARDUPILOT INTERFACE
    # ========================================================================
    # Executes planned paths by sending waypoints to ArduPilot via MAVLink
    # 
    # Subscribes to:
    #   - /planned_path (from planner)
    # 
    # Publishes to:
    #   - ArduPilot via MAVLink (SET_POSITION_TARGET_LOCAL_NED messages)
    # 
    # Note: publish_pose is disabled because we use NDT-GICP localization
    ardupilot_node = Node(
        package='nav3d_planner',
        executable='ardupilot_interface.py',
        name='ardupilot_interface',
        output='screen',
        parameters=[{
            'connection_string': ardupilot_connection,
            'waypoint_threshold': 1.0,  # Distance to waypoint = reached (m)
            'publish_pose': False,      # Don't publish pose (we use localization)
            'control_frame': 'odom',    # Frame for local NED commands
            'use_sim_time': use_sim_time,
            'takeoff_alt': 1.0
        }],
        condition=IfCondition(enable_ardupilot)
    )
    
    # ========================================================================
    # NODE 5: RVIZ2 VISUALIZATION
    # ========================================================================
    # Displays:
    #   - 3D map (point cloud)
    #   - Current drone pose (red arrow)
    #   - Goal pose (cyan arrow) - can be set by clicking in RViz
    #   - Planned path (green line)
    #   - Planning markers (waypoint spheres)
    rviz_config = PathJoinSubstitution([
        FindPackageShare('nav3d_planner'),
        'rviz',
        'nav3d_config.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        # Arguments
        map_pcd_file_arg,
        use_sim_time_arg,
        enable_ardupilot_arg,
        ardupilot_connection_arg,
        localization_topic_arg,
        
        # Nodes
        pcd_publisher,
        pointcloud_fixer,
        odom_to_pose_converter,
        planner_node,
        ardupilot_node,
        rviz_node
    ])