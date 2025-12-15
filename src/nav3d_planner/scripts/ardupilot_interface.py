#!/usr/bin/env python3
"""
ROS-based path following interface with yaw control
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
from rclpy.duration import Duration
import copy
import numpy as np
import time
from scipy.spatial.transform import Rotation
import math

class ArduPilotInterface(Node):
    def __init__(self):
        super().__init__('ardupilot_interface')
        
        # Parameters
        self.declare_parameter('waypoint_threshold', 1.0)
        self.declare_parameter('control_frame', 'odom')
        self.declare_parameter('use_tf_for_position', True)
        
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        self.control_frame = self.get_parameter('control_frame').value
        self.use_tf_for_position = self.get_parameter('use_tf_for_position').value
        
        # TF Buffer
        try:
            self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0), clock=self.get_clock())
        except TypeError:
            self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
            
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self.current_path = None
        self.current_waypoint_idx = 0
        self.current_position = None
        self.is_path_finished = False
        self.path_start_time = None
        self.target_yaw = None  # Store target yaw from goal
        
        # Throttling
        self.last_dist_log_time = 0.0
        
        # Subscribers
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Publisher for goal pose (to be picked up by goal_publisher or external controller)
        self.goal_pub = self.create_publisher(PoseStamped, '/current_target', 10)
        
        # Control Loop (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f'✓ ArduPilot Interface Ready (Path Monitoring Mode)')
        self.get_logger().info('   Waiting for paths on /planned_path...')
        self.get_logger().info('   Current targets will be published to /current_target')

    def get_position_from_tf(self):
        """Get position from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.control_frame, 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
        except:
            return None
    
    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle (radians)"""
        # quat = [x, y, z, w]
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def goal_callback(self, msg):
        """Receive goal pose and extract yaw"""
        # Extract yaw from quaternion
        self.target_yaw = self.quaternion_to_yaw(msg.pose.orientation)
        yaw_deg = math.degrees(self.target_yaw)
        self.get_logger().info(f'Goal yaw received: {yaw_deg:.1f}°')

    def path_callback(self, msg):
        """Receive new path"""
        if len(msg.poses) == 0:
            return
            
        self.current_path = msg
        self.current_waypoint_idx = 0
        self.is_path_finished = False
        self.path_start_time = time.time()
        
        self.get_logger().info(f'\n{"="*70}')
        self.get_logger().info(f'✓ NEW PATH RECEIVED')
        self.get_logger().info(f'  Waypoints: {len(msg.poses)}')
        self.get_logger().info(f'  Goal: [{msg.poses[-1].pose.position.x:.2f}, '
                              f'{msg.poses[-1].pose.position.y:.2f}, '
                              f'{msg.poses[-1].pose.position.z:.2f}]')
        self.get_logger().info(f'  Starting navigation monitoring...')
        self.get_logger().info(f'{"="*70}\n')

    def manual_transform_pose(self, pose_stamped, transform):
        """Transform pose"""
        trans = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        
        quat_tf = [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ]
        rot_tf = Rotation.from_quat(quat_tf)
        
        pos = np.array([
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z
        ])
        
        new_pos = trans + rot_tf.apply(pos)
        
        transformed = PoseStamped()
        transformed.header.stamp = pose_stamped.header.stamp
        transformed.header.frame_id = transform.header.frame_id
        transformed.pose.position.x = new_pos[0]
        transformed.pose.position.y = new_pos[1]
        transformed.pose.position.z = new_pos[2]
        transformed.pose.orientation.w = 1.0
        
        return transformed

    def transform_pose(self, pose_stamped, target_frame):
        """Transform pose to target frame"""
        try:
            if not self.tf_buffer.can_transform(
                target_frame, pose_stamped.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            ):
                return None
            
            transform = self.tf_buffer.lookup_transform(
                target_frame, pose_stamped.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            return self.manual_transform_pose(pose_stamped, transform)
        except:
            return None

    def control_loop(self):
        """Main monitoring loop - 10Hz"""
        # Check if we have a path
        if self.current_path is None:
            return

        # Get position
        if self.use_tf_for_position:
            self.current_position = self.get_position_from_tf()
        
        if self.current_position is None:
            return

        # Get current waypoint
        idx = self.current_waypoint_idx
        if self.is_path_finished:
            return

        # Transform to odom
        target_map = copy.deepcopy(self.current_path.poses[idx])
        target_map.header.stamp = rclpy.time.Time(seconds=0).to_msg()
        target_odom = self.transform_pose(target_map, self.control_frame)
        
        if target_odom is None:
            return

        tgt_x = target_odom.pose.position.x
        tgt_y = target_odom.pose.position.y
        tgt_z = target_odom.pose.position.z

        # Calculate distance to waypoint
        dx = tgt_x - self.current_position[0]
        dy = tgt_y - self.current_position[1]
        dz = tgt_z - self.current_position[2]
        dist = np.sqrt(dx*dx + dy*dy + dz*dz)

        # Publish current target
        self.goal_pub.publish(target_odom)

        # Check if reached waypoint
        if dist < self.waypoint_threshold:
            self.get_logger().info(f'✓ Reached waypoint {idx + 1}')
            if self.current_waypoint_idx < len(self.current_path.poses) - 1:
                self.current_waypoint_idx += 1
            else:
                elapsed = time.time() - self.path_start_time
                self.get_logger().info(f'\n{"="*70}')
                self.get_logger().info(f'🎯 GOAL REACHED in {elapsed:.1f}s!')
                
                # Log final yaw if we have it
                if self.target_yaw is not None:
                    yaw_deg = math.degrees(self.target_yaw)
                    self.get_logger().info(f'   Target yaw: {yaw_deg:.1f}°')
                    self.get_logger().info(f'   Note: Yaw should be set by mission_planner/goal_publisher')
                
                self.get_logger().info(f'{"="*70}\n')
                self.is_path_finished = True
            return

        # Logging
        now = time.time()
        if now - self.last_dist_log_time > 2.0:
            elapsed = now - self.path_start_time if self.path_start_time else 0
            self.get_logger().info(
                f'[{elapsed:.0f}s] WP {idx+1}/{len(self.current_path.poses)}: {dist:.2f}m remaining'
            )
            self.last_dist_log_time = now

def main(args=None):
    rclpy.init(args=args)
    node = ArduPilotInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()