#!/usr/bin/env python3
"""
Mission planner for autonomous multi-waypoint navigation.
Usage: 
  ros2 run nav3d_planner mission_planner.py --waypoints "10,5,3,90" "20,10,5,180" "15,15,7,0"
  ros2 run nav3d_planner mission_planner.py --file mission.txt
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import argparse
import time
from pymavlink import mavutil
import sys
import math

class MissionPlanner(Node):
    def __init__(self, waypoints):
        super().__init__('mission_planner')
        
        # Store waypoints
        self.waypoints = waypoints
        self.current_waypoint_idx = 0
        
        # ROS publisher
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
        
        # Check if on the ground
        self.get_logger().info("Checking altitude...")
        on_ground = self.is_on_ground()
        
        if on_ground:
            self.get_logger().info("✓ Drone is on the ground - proceeding with takeoff")
            
            # Arm motors
            self.connection.arducopter_arm()
            self.get_logger().info("✓ Arming motors...")
            time.sleep(2)
            
            # Takeoff to safe altitude (max of all waypoint altitudes + 1m)
            max_alt = max([wp[2] for wp in waypoints])
            takeoff_alt = max(2.0, max_alt)
            
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0,
                takeoff_alt
            )
            self.get_logger().info(f"✓ Taking off to {takeoff_alt} m...")
            time.sleep(8)  # Wait for takeoff to complete
        else:
            self.get_logger().info("✓ Drone is already airborne - skipping takeoff")
        
        # Display mission plan
        self.print_mission_plan()
        
        # Start mission execution
        self.execute_mission()
    
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
    
    def print_mission_plan(self):
        """Display the complete mission plan"""
        self.get_logger().info(f'\n{"="*70}')
        self.get_logger().info(f'📋 MISSION PLAN')
        self.get_logger().info(f'   Total Waypoints: {len(self.waypoints)}')
        self.get_logger().info(f'{"="*70}')
        for i, (x, y, z, yaw) in enumerate(self.waypoints, 1):
            self.get_logger().info(f'   WP{i}: X={x:.2f}m, Y={y:.2f}m, Z={z:.2f}m, Yaw={yaw:.1f}°')
        self.get_logger().info(f'{"="*70}\n')
    
    def publish_goal(self, x, y, z, yaw_deg):
        """Publish a single goal"""
        yaw_rad = math.radians(yaw_deg)
        
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        
        # Convert yaw to quaternion
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal.pose.orientation.w = math.cos(yaw_rad / 2.0)
        
        # Publish multiple times to ensure receipt
        for _ in range(5):
            self.pub.publish(goal)
            time.sleep(0.1)
        
        self.get_logger().info(f'✓ Goal published: WP{self.current_waypoint_idx + 1} -> X={x:.2f}, Y={y:.2f}, Z={z:.2f}, Yaw={yaw_deg:.1f}°')
    
    def send_yaw_command(self, yaw_deg):
        """Send yaw command to ArduPilot"""
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
    
    def get_current_position(self):
        """Get current drone position from MAVLink"""
        try:
            msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.5)
            if msg:
                # Convert NED to ENU (matching ROS coordinate system)
                current_x = msg.y
                current_y = msg.x
                current_z = -msg.z
                return current_x, current_y, current_z
        except:
            pass
        return None, None, None
    
    def execute_mission(self):
        """Execute the mission waypoint by waypoint"""
        for idx, (x, y, z, yaw) in enumerate(self.waypoints):
            self.current_waypoint_idx = idx
            
            self.get_logger().info(f'\n{"─"*70}')
            self.get_logger().info(f'🎯 Navigating to WP{idx + 1}/{len(self.waypoints)}: ({x:.2f}, {y:.2f}, {z:.2f}, {yaw:.1f}°)')
            self.get_logger().info(f'{"─"*70}')
            
            # Publish goal - let ROS navigation handle path planning
            self.publish_goal(x, y, z, yaw)
            
            # Wait for waypoint to be reached
            start_time = time.time()
            last_log_time = start_time
            reached_count = 0  # Require staying near waypoint for stability
            
            while True:
                # Get current position
                curr_x, curr_y, curr_z = self.get_current_position()
                
                if curr_x is not None:
                    # Calculate distance to target
                    dx = x - curr_x
                    dy = y - curr_y
                    dz = z - curr_z
                    dist = (dx**2 + dy**2 + dz**2)**0.5
                    
                    # Log progress every 2 seconds
                    current_time = time.time()
                    if current_time - last_log_time > 2.0:
                        elapsed = current_time - start_time
                        self.get_logger().info(f'  [{elapsed:.0f}s] Distance: {dist:.2f}m')
                        last_log_time = current_time
                    
                    # Check if reached (with stability check)
                    if dist < 1.0:  # Within 1m threshold
                        reached_count += 1
                        if reached_count >= 10:  # Stay near for 1 second (10 x 0.1s)
                            elapsed = current_time - start_time
                            self.get_logger().info(f'✓ WP{idx + 1} REACHED in {elapsed:.1f}s!')
                            
                            # Set yaw once waypoint is reached
                            self.get_logger().info(f'  Setting yaw to {yaw:.1f}°...')
                            self.send_yaw_command(yaw)
                            time.sleep(1)  # Give time for yaw to adjust
                            
                            break
                    else:
                        reached_count = 0  # Reset if moved away
                else:
                    # No position data available
                    current_time = time.time()
                    if current_time - last_log_time > 2.0:
                        self.get_logger().warn('  Waiting for position data...')
                        last_log_time = current_time
                
                time.sleep(0.1)
            
            # Hover briefly before next waypoint
            if idx < len(self.waypoints) - 1:
                self.get_logger().info('  Hovering for 2 seconds...')
                time.sleep(2)
        
        # Mission complete
        self.get_logger().info(f'\n{"="*70}')
        self.get_logger().info(f'🎉 MISSION COMPLETE!')
        self.get_logger().info(f'   All {len(self.waypoints)} waypoints reached!')
        self.get_logger().info(f'{"="*70}\n')

def parse_waypoints_from_file(filepath):
    """Parse waypoints from a text file"""
    waypoints = []
    try:
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                # Skip empty lines and comments
                if not line or line.startswith('#'):
                    continue
                # Parse x,y,z,yaw (yaw is optional, defaults to 0)
                parts = line.replace(',', ' ').split()
                if len(parts) >= 3:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    yaw = float(parts[3]) if len(parts) >= 4 else 0.0
                    waypoints.append((x, y, z, yaw))
    except Exception as e:
        print(f"Error reading file: {e}")
        sys.exit(1)
    return waypoints

def main():
    parser = argparse.ArgumentParser(
        description='Mission planner for autonomous multi-waypoint navigation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Command-line waypoints (x,y,z,yaw in degrees)
  ros2 run nav3d_planner mission_planner.py --waypoints "10,5,3,90" "20,10,5,180" "15,15,7,0"
  
  # Yaw is optional (defaults to 0)
  ros2 run nav3d_planner mission_planner.py --waypoints "10,5,3" "20,10,5,180"
  
  # From file
  ros2 run nav3d_planner mission_planner.py --file mission.txt
  
Mission file format (mission.txt):
  # Lines starting with # are comments
  # Format: x, y, z, yaw(degrees) - yaw is optional
  10, 5, 3, 90
  20, 10, 5, 180
  15, 15, 7, 0
  0, 0, 2
        """
    )
    
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--waypoints', nargs='+', help='Waypoints as "x,y,z,yaw" strings (yaw optional)')
    group.add_argument('--file', help='Path to mission file')
    
    args = parser.parse_args()
    
    # Parse waypoints
    waypoints = []
    if args.waypoints:
        for wp_str in args.waypoints:
            parts = wp_str.replace(',', ' ').split()
            if len(parts) < 3:
                print(f"Error: Invalid waypoint format '{wp_str}'. Expected 'x,y,z' or 'x,y,z,yaw'")
                sys.exit(1)
            try:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                yaw = float(parts[3]) if len(parts) >= 4 else 0.0
                waypoints.append((x, y, z, yaw))
            except ValueError:
                print(f"Error: Invalid numbers in waypoint '{wp_str}'")
                sys.exit(1)
    elif args.file:
        waypoints = parse_waypoints_from_file(args.file)
        if not waypoints:
            print(f"Error: No valid waypoints found in {args.file}")
            sys.exit(1)
    
    if len(waypoints) == 0:
        print("Error: No waypoints provided")
        sys.exit(1)
    
    print(f"\n📋 Mission loaded: {len(waypoints)} waypoints")
    
    # Initialize ROS and execute mission
    rclpy.init()
    node = MissionPlanner(waypoints)
    
    # Keep node alive during mission
    time.sleep(1)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()