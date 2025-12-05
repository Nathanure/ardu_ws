#!/usr/bin/env python3
"""
Interactive goal publisher for testing.
Usage: ros2 run nav3d_planner goal_publisher.py --x 10 --y 5 --z 3
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import argparse

class GoalPublisher(Node):
    def __init__(self, x, y, z):
        super().__init__('goal_publisher')
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Publish goal once
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        
        self.get_logger().info(f'Publishing goal: ({x}, {y}, {z})')
        self.pub.publish(goal)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--z', type=float, required=True)
    args = parser.parse_args()
    
    rclpy.init()
    node = GoalPublisher(args.x, args.y, args.z)
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()