#!/usr/bin/env python3
"""
Velocity-based navigation - matches teleop control style
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import threading
import tf2_ros
from rclpy.duration import Duration
import copy
import numpy as np
import time
from scipy.spatial.transform import Rotation

class ArduPilotInterface(Node):
    def __init__(self):
        super().__init__('ardupilot_interface')
        
        # Parameters
        self.declare_parameter('connection_string', 'udp:127.0.0.1:14550')
        self.declare_parameter('waypoint_threshold', 1.0)
        self.declare_parameter('control_frame', 'odom')
        self.declare_parameter('use_tf_for_position', True)
        self.declare_parameter('max_velocity', 2.0)  # Max speed in m/s
        
        connection_string = self.get_parameter('connection_string').value
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        self.control_frame = self.get_parameter('control_frame').value
        self.use_tf_for_position = self.get_parameter('use_tf_for_position').value
        self.max_velocity = self.get_parameter('max_velocity').value
        
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
        self.current_yaw = 0.0
        self.is_path_finished = False
        self.path_start_time = None
        
        # Status monitoring
        self.current_mode = "UNKNOWN"
        self.armed = False
        self.ready_to_navigate = False
        self.last_heartbeat = None
        
        # Throttling
        self.last_dist_log_time = 0.0
        self.last_command_time = 0.0
        self.last_status_log_time = 0.0
        
        # Connect to ArduPilot
        self.get_logger().info(f'Connecting to ArduPilot at {connection_string}...')
        try:
            self.vehicle = mavutil.mavlink_connection(connection_string)
            self.vehicle.wait_heartbeat(timeout=10)
            self.get_logger().info('✓ Connected to ArduPilot!')
            
            # Request data streams
            self.vehicle.mav.request_data_stream_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                10, 1  # 10Hz for better attitude updates
            )
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            raise
        
        # Subscribers
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        
        # Threads
        self.running = True
        
        # Status monitoring thread
        self.status_thread = threading.Thread(target=self.status_monitor_loop, daemon=True)
        self.status_thread.start()
        
        # Heartbeat thread
        self.heartbeat_thread = threading.Thread(target=self.heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()
        
        # Control Loop (20Hz for smoother velocity control)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(f'✓ ArduPilot Interface Ready (Velocity Control Mode)')
        self.get_logger().warn('⚠ IMPORTANT: Drone must be ARMED and in GUIDED mode before navigation!')
        self.get_logger().info('   Use teleop to: 1) ARM  2) Switch to GUIDED  3) Takeoff')

    def heartbeat_loop(self):
        """Send heartbeat"""
        while self.running:
            try:
                self.vehicle.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                time.sleep(1.0)
            except:
                pass

    def status_monitor_loop(self):
        """Monitor ArduPilot status"""
        while self.running:
            try:
                msg = self.vehicle.recv_match(blocking=True, timeout=0.5)
                if msg is None:
                    continue
                    
                msg_type = msg.get_type()
                
                if msg_type == 'HEARTBEAT':
                    self.last_heartbeat = time.time()
                    self.armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    
                    # Decode mode
                    mode_map = {v: k for k, v in self.vehicle.mode_mapping().items()}
                    old_mode = self.current_mode
                    self.current_mode = mode_map.get(msg.custom_mode, f"MODE_{msg.custom_mode}")
                    
                    # Check if ready to navigate
                    was_ready = self.ready_to_navigate
                    self.ready_to_navigate = (self.current_mode == "GUIDED" and self.armed)
                    
                    # Log mode changes
                    if old_mode != self.current_mode:
                        self.get_logger().info(f'Mode changed: {old_mode} → {self.current_mode}')
                    
                    # Log when becoming ready
                    if not was_ready and self.ready_to_navigate:
                        self.get_logger().info('\n' + '='*70)
                        self.get_logger().info('✓ READY TO NAVIGATE!')
                        self.get_logger().info('  Mode: GUIDED | Armed: True')
                        self.get_logger().info('  You can now set navigation goals')
                        self.get_logger().info('='*70 + '\n')
                    
                    # Log when becoming not ready
                    if was_ready and not self.ready_to_navigate:
                        self.get_logger().warn('\n' + '='*70)
                        self.get_logger().warn('⚠ NOT READY TO NAVIGATE!')
                        self.get_logger().warn(f'  Mode: {self.current_mode} | Armed: {self.armed}')
                        self.get_logger().warn('='*70 + '\n')
                        
                elif msg_type == 'ATTITUDE':
                    # Get yaw from attitude
                    self.current_yaw = msg.yaw
                    
                elif msg_type == 'LOCAL_POSITION_NED':
                    if not self.use_tf_for_position:
                        self.current_position = [msg.y, msg.x, -msg.z]
                        
            except Exception:
                pass

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
        
        if not self.ready_to_navigate:
            self.get_logger().warn(f'  Status: NOT READY (Mode: {self.current_mode}, Armed: {self.armed})')
            self.get_logger().warn(f'  Waiting for GUIDED mode + Armed...')
        else:
            self.get_logger().info(f'  Status: READY - Starting navigation!')
            
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

    def send_velocity_local_ned(self, vx, vy, vz, yaw_rate=0):
        """
        Send velocity command in LOCAL NED frame
        This matches the teleop control style
        vx: North velocity (m/s)
        vy: East velocity (m/s)  
        vz: Down velocity (m/s)
        yaw_rate: Yaw rate (rad/s)
        """
        now = time.time()
        if now - self.last_command_time < 0.04:  # 25Hz max
            return
        
        self.last_command_time = now
        
        # Send velocity command (same style as teleop but in LOCAL_NED instead of BODY_OFFSET)
        self.vehicle.mav.set_position_target_local_ned_send(
            0,
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # Velocity + yaw rate control (ignore position, accel, yaw)
            0, 0, 0,      # Ignore position
            vx, vy, vz,   # Velocity in m/s
            0, 0, 0,      # Ignore acceleration
            0,            # Ignore yaw
            yaw_rate      # Yaw rate
        )

    def control_loop(self):
        """Main control loop - 20Hz"""
        # Check if we have a path
        if self.current_path is None:
            return

        # CRITICAL: Only navigate if in GUIDED mode and armed
        if not self.ready_to_navigate:
            now = time.time()
            if now - self.last_status_log_time > 5.0:
                self.get_logger().warn(
                    f'⚠ Waiting for GUIDED + Armed (Current: {self.current_mode}, Armed={self.armed})'
                )
                self.last_status_log_time = now
            return

        # Get position
        if self.use_tf_for_position:
            self.current_position = self.get_position_from_tf()
        
        if self.current_position is None:
            return

        # Get current waypoint
        idx = self.current_waypoint_idx
        if self.is_path_finished:
            # Hold position at final waypoint
            self.send_velocity_local_ned(0, 0, 0, 0)
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

        # Calculate error vector (in odom frame, which is ENU)
        dx = tgt_x - self.current_position[0]
        dy = tgt_y - self.current_position[1]
        dz = tgt_z - self.current_position[2]
        dist = np.sqrt(dx*dx + dy*dy + dz*dz)

        # Check if reached waypoint
        if dist < self.waypoint_threshold:
            self.get_logger().info(f'✓ Reached waypoint {idx + 1}')
            if self.current_waypoint_idx < len(self.current_path.poses) - 1:
                self.current_waypoint_idx += 1
            else:
                elapsed = time.time() - self.path_start_time
                self.get_logger().info(f'\n{"="*70}')
                self.get_logger().info(f'🎯 GOAL REACHED in {elapsed:.1f}s!')
                self.get_logger().info(f'{"="*70}\n')
                self.is_path_finished = True
            return

        # Compute desired velocity (proportional control)
        # Use a gain to make it responsive but not too aggressive
        gain = 1.0  # Adjust this if needed (0.5-2.0 range)
        
        desired_vx = gain * dx
        desired_vy = gain * dy
        desired_vz = gain * dz
        
        # Clamp to max velocity
        vel_magnitude = np.sqrt(desired_vx**2 + desired_vy**2 + desired_vz**2)
        if vel_magnitude > self.max_velocity:
            scale = self.max_velocity / vel_magnitude
            desired_vx *= scale
            desired_vy *= scale
            desired_vz *= scale

        # Convert ENU (odom) to NED for ArduPilot
        # ENU: x=East, y=North, z=Up
        # NED: x=North, y=East, z=Down
        vx_ned = desired_vy   # North = ENU y
        vy_ned = desired_vx   # East = ENU x
        vz_ned = -desired_vz  # Down = -ENU z

        # Send velocity command
        self.send_velocity_local_ned(vx_ned, vy_ned, vz_ned, 0)

        # Logging
        now = time.time()
        if now - self.last_dist_log_time > 2.0:
            elapsed = now - self.path_start_time if self.path_start_time else 0
            self.get_logger().info(
                f'[{elapsed:.0f}s] WP {idx+1}/{len(self.current_path.poses)}: {dist:.2f}m '
                f'| vel=[{vx_ned:.2f}, {vy_ned:.2f}, {vz_ned:.2f}] m/s'
            )
            self.last_dist_log_time = now

    def destroy_node(self):
        """Cleanup"""
        self.running = False
        # Send stop command
        try:
            self.send_velocity_local_ned(0, 0, 0, 0)
        except:
            pass
        super().destroy_node()

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