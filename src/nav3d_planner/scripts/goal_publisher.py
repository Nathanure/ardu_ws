#!/usr/bin/env python3
"""
Autonomous goal publisher - publishes goal after takeoff and hover.
Usage: ros2 run nav3d_planner goal_publisher.py --x 10 --y 5 --z 3 --yaw 90
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import argparse
import time
from pymavlink import mavutil
import math

class GoalPublisher(Node):
    def is_on_ground(self, altitude_threshold=0.5):
        """Check if drone is on the ground by reading altitude"""
        try:
            # Request LOCAL_POSITION_NED message
            self.connection.mav.request_data_stream_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                10, 1
            )
            
            # Wait for position update
            start_time = time.time()
            while time.time() - start_time < 3.0:  # 3 second timeout
                msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.5)
                if msg:
                    altitude = -msg.z  # Convert NED to altitude (up is positive)
                    self.get_logger().info(f"  Current altitude: {altitude:.2f}m")
                    return altitude < altitude_threshold
            
            # If no message received, assume on ground for safety
            self.get_logger().warn("  Could not read altitude, assuming on ground")
            return True
        except Exception as e:
            self.get_logger().warn(f"  Error checking altitude: {e}, assuming on ground")
            return True
    
    def __init__(self, x, y, z, yaw_deg):
        super().__init__('goal_publisher')
        
        # Store goal coordinates
        self.goal_x = x
        self.goal_y = y
        self.goal_z = z
        self.goal_yaw_deg = yaw_deg
        self.goal_yaw_rad = math.radians(yaw_deg)
        
        # ROS publisher
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Connect to MAVLink (ONLY for initialization: arm, takeoff)
        self.get_logger().info('Connecting to ArduPilot...')
        self.connection = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
        self.connection.wait_heartbeat()
        self.get_logger().info(f"✓ Heartbeat from sys:{self.connection.target_system} comp:{self.connection.target_component}")
        
        # Set GUIDED mode
        self.connection.set_mode_apm('GUIDED')
        self.get_logger().info("✓ Mode -> GUIDED")
        time.sleep(1)
        
        # Check if on the ground
        self.get_logger().info("Checking altitude...")
        on_ground = self.is_on_ground()
        
        if on_ground:
            self.get_logger().info("✓ Drone is on the ground - proceeding with takeoff")
            
            # Arm motors
            self.connection.arducopter_arm()
            self.get_logger().info("✓ Arming motors...")
            time.sleep(2)
            
            # Takeoff to initial altitude
            takeoff_alt = max(2.0, z)  # At least 2m or target altitude
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,        # confirmation
                0, 0, 0, 0,  # params 1-4 unused
                0, 0,     # lat, lon (ignored in guided mode)
                takeoff_alt
            )
            self.get_logger().info(f"✓ Taking off to {takeoff_alt} m...")
            time.sleep(8)  # Wait for takeoff to complete
        else:
            self.get_logger().info("✓ Drone is already airborne - skipping takeoff")
        
        # Now publish the goal for ardupilot_interface to handle
        self.get_logger().info(f'\n{"="*70}')
        self.get_logger().info(f'📍 PUBLISHING NAVIGATION GOAL')
        self.get_logger().info(f'   Target: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, Yaw={yaw_deg:.1f}°')
        self.get_logger().info(f'{"="*70}\n')
        
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        
        # Convert yaw to quaternion (rotation around Z-axis)
        # Quaternion formula for rotation around Z: [0, 0, sin(yaw/2), cos(yaw/2)]
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(self.goal_yaw_rad / 2.0)
        goal.pose.orientation.w = math.cos(self.goal_yaw_rad / 2.0)
        
        # Publish goal multiple times to ensure it's received
        for i in range(5):
            self.pub.publish(goal)
            time.sleep(0.1)
        
        self.get_logger().info('✓ Goal published! ardupilot_interface will now navigate to target.')
        
        # Send yaw command to ArduPilot
        self.send_yaw_command(self.goal_yaw_deg)
        
        self.get_logger().info('   (This node will now exit - navigation continues via ardupilot_interface)')
    
    def send_yaw_command(self, yaw_deg):
        """Send yaw command to ArduPilot"""
        self.get_logger().info(f'✓ Setting yaw to {yaw_deg:.1f}°...')
        
        # Send MAV_CMD_CONDITION_YAW command
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,              # confirmation
            yaw_deg,        # param1: target angle in degrees
            0,              # param2: angular speed (0 = default)
            1,              # param3: direction (1=clockwise, -1=counter-clockwise)
            0,              # param4: 0=absolute angle, 1=relative
            0, 0, 0         # params 5-7: unused
        )
        time.sleep(0.5)  # Give time for yaw command to process

def main():
    parser = argparse.ArgumentParser(description='Autonomous goal publisher with takeoff')
    parser.add_argument('--x', type=float, required=True, help='Target X coordinate (meters)')
    parser.add_argument('--y', type=float, required=True, help='Target Y coordinate (meters)')
    parser.add_argument('--z', type=float, required=True, help='Target Z altitude (meters)')
    parser.add_argument('--yaw', type=float, default=0.0, help='Target yaw angle in degrees (default: 0)')
    args = parser.parse_args()
    
    rclpy.init()
    node = GoalPublisher(args.x, args.y, args.z, args.yaw)
    
    # Keep node alive briefly to ensure messages are sent
    time.sleep(1)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()