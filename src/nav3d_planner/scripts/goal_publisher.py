#!/usr/bin/env python3
"""
Autonomous goal publisher - handles complete navigation mission.
Takes off, navigates to target, and exits when destination is reached.

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
            self.connection.mav.request_data_stream_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                10, 1
            )
            
            start_time = time.time()
            while time.time() - start_time < 3.0:
                msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.5)
                if msg:
                    altitude = -msg.z
                    self.get_logger().info(f"  Current altitude: {altitude:.2f}m")
                    return altitude < altitude_threshold
            
            self.get_logger().warn("  Could not read altitude, assuming on ground")
            return True
        except Exception as e:
            self.get_logger().warn(f"  Error checking altitude: {e}, assuming on ground")
            return True
    
    def verify_guided_mode(self):
        """Verify that GUIDED mode is active"""
        self.get_logger().info("Verifying GUIDED mode...")
        try:
            msg = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=3.0)
            if msg:
                mode_mapping = self.connection.mode_mapping()
                if mode_mapping:
                    current_mode = None
                    for mode_name, mode_num in mode_mapping.items():
                        if mode_num == msg.custom_mode:
                            current_mode = mode_name
                            break
                    
                    if current_mode == 'GUIDED':
                        self.get_logger().info("✓ GUIDED mode confirmed!")
                        return True
                    else:
                        self.get_logger().error(f"✗ Mode is {current_mode}, not GUIDED!")
                        self.get_logger().error("  Trying to set GUIDED mode again...")
                        self.connection.set_mode_apm('GUIDED')
                        time.sleep(1)
                        return False
            else:
                self.get_logger().warn("Could not verify mode (no heartbeat)")
                return False
        except Exception as e:
            self.get_logger().warn(f"Mode verification failed: {e}")
            return False
    
    def __init__(self, x, y, z, yaw_deg):
        super().__init__('goal_publisher')
        
        # Store goal coordinates (ROS ENU frame)
        self.goal_x = x
        self.goal_y = y
        self.goal_z = z
        self.goal_yaw_deg = yaw_deg
        self.goal_yaw_rad = math.radians(yaw_deg)
        
        # ROS publisher (for visualization)
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Connect to MAVLink
        self.get_logger().info('Connecting to ArduPilot...')
        self.connection = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
        self.connection.wait_heartbeat()
        self.get_logger().info(f"✓ Heartbeat from sys:{self.connection.target_system} comp:{self.connection.target_component}")
        
        # Set GUIDED mode
        self.connection.set_mode_apm('GUIDED')
        self.get_logger().info("✓ Mode -> GUIDED")
        time.sleep(1)
        
        # Verify GUIDED mode is active
        self.verify_guided_mode()
        
        # Check if on the ground
        self.get_logger().info("Checking altitude...")
        on_ground = self.is_on_ground()
        
        if on_ground:
            self.get_logger().info("✓ Drone is on the ground - proceeding with takeoff")
            
            # Arm motors
            self.connection.arducopter_arm()
            self.get_logger().info("✓ Arming motors...")
            time.sleep(2)
            
            # Takeoff
            takeoff_alt = max(2.0, z)
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0,
                takeoff_alt
            )
            self.get_logger().info(f"✓ Taking off to {takeoff_alt} m...")
            time.sleep(8)
        else:
            self.get_logger().info("✓ Drone is already airborne - skipping takeoff")
        
        # Publish goal to ROS (for visualization in RViz)
        self.get_logger().info(f'\n{"="*70}')
        self.get_logger().info(f'📍 MISSION START')
        self.get_logger().info(f'   Target: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, Yaw={yaw_deg:.1f}°')
        self.get_logger().info(f'{"="*70}\n')
        
        # Publish to ROS topic for visualization
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(self.goal_yaw_rad / 2.0)
        goal.pose.orientation.w = math.cos(self.goal_yaw_rad / 2.0)
        
        for i in range(5):
            self.pub.publish(goal)
            time.sleep(0.1)
        
        # Navigate to target and wait until reached
        self.send_position_target_until_reached(x, y, z, yaw_deg)
        
        # Mission complete - node will exit
        self.get_logger().info('✓ Mission complete! Exiting...\n')
    
    def send_position_target_until_reached(self, x, y, z, yaw_deg, tolerance=0.5):
        """
        Send position setpoint continuously until drone reaches target.
        ArduPilot GUIDED mode requires continuous commands (like RC input).
        """
        # Convert ROS ENU to ArduPilot NED
        ned_x = y      # North = ROS y
        ned_y = x      # East = ROS x
        ned_z = -z     # Down = -ROS z
        yaw_rad = math.radians(yaw_deg)
        
        self.get_logger().info(f'🎯 NAVIGATING TO TARGET')
        self.get_logger().info(f'   ROS ENU: X={x:.2f}, Y={y:.2f}, Z={z:.2f}')
        self.get_logger().info(f'   NED: X={ned_x:.2f}, Y={ned_y:.2f}, Z={ned_z:.2f}, Yaw={yaw_deg:.1f}°')
        self.get_logger().info(f'   Tolerance: {tolerance}m\n')
        
        frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED
        type_mask = 0x0FF8  # Position only
        
        start_time = time.time()
        last_log_time = start_time
        reached_count = 0  # Require multiple checks for stability
        
        while rclpy.ok():
            # Send position command at 20Hz
            self.connection.mav.set_position_target_local_ned_send(
                0,
                self.connection.target_system,
                self.connection.target_component,
                frame,
                type_mask,
                ned_x, ned_y, ned_z,
                0, 0, 0,
                0, 0, 0,
                yaw_rad, 0
            )
            
            # Get current position
            msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            if msg:
                # Calculate distance to target
                dx = ned_x - msg.x
                dy = ned_y - msg.y
                dz = ned_z - msg.z
                dist = (dx**2 + dy**2 + dz**2)**0.5
                
                # Log progress every 2 seconds
                current_time = time.time()
                elapsed = current_time - start_time
                if current_time - last_log_time > 2.0:
                    self.get_logger().info(f'   [{elapsed:.0f}s] Distance: {dist:.2f}m')
                    last_log_time = current_time
                
                # Check if reached (with stability check)
                if dist < tolerance:
                    reached_count += 1
                    if reached_count >= 20:  # Stay near for 1 second (20 x 0.05s)
                        self.get_logger().info(f'\n{"="*70}')
                        self.get_logger().info(f'✓ TARGET REACHED in {elapsed:.1f}s!')
                        self.get_logger().info(f'   Final distance: {dist:.2f}m')
                        self.get_logger().info(f'{"="*70}\n')
                        
                        # Send yaw command
                        self.send_yaw_command(yaw_deg)
                        return  # Exit the loop - mission complete!
                else:
                    reached_count = 0
            
            # Sleep to maintain 20Hz
            time.sleep(0.05)
    
    def send_yaw_command(self, yaw_deg):
        """Send yaw command to ArduPilot"""
        self.get_logger().info(f'   Setting final yaw to {yaw_deg:.1f}°...')
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            yaw_deg,
            0,
            1,
            0,
            0, 0, 0
        )
        time.sleep(2)

def main():
    parser = argparse.ArgumentParser(description='Navigate to target position and exit when reached')
    parser.add_argument('--x', type=float, required=True, help='Target X coordinate (meters, East)')
    parser.add_argument('--y', type=float, required=True, help='Target Y coordinate (meters, North)')
    parser.add_argument('--z', type=float, required=True, help='Target Z altitude (meters, Up)')
    parser.add_argument('--yaw', type=float, default=0.0, help='Target yaw angle in degrees (default: 0)')
    args = parser.parse_args()
    
    rclpy.init()
    node = GoalPublisher(args.x, args.y, args.z, args.yaw)
    
    # Node exits automatically after mission complete
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()