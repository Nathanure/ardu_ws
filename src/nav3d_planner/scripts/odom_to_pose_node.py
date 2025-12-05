#!/usr/bin/env python3
"""
Simple converter node: nav_msgs/Odometry → geometry_msgs/PoseStamped

The NDT-GICP localizer publishes nav_msgs/Odometry, but the planner
expects geometry_msgs/PoseStamped. This node bridges them.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class OdomToPoseNode(Node):
    def __init__(self):
        super().__init__('odom_to_pose_converter')
        
        # Subscriber to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            'input_odom',
            self.odom_callback,
            10
        )
        
        # Publisher for pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            'output_pose',
            10
        )
        
        self.get_logger().info('Odometry to Pose converter initialized')
        self.get_logger().info('  Input: input_odom (nav_msgs/Odometry)')
        self.get_logger().info('  Output: output_pose (geometry_msgs/PoseStamped)')
    
    def odom_callback(self, odom_msg):
        """Convert Odometry to PoseStamped"""
        pose_msg = PoseStamped()
        
        # Copy header
        pose_msg.header = odom_msg.header
        
        # Copy pose
        pose_msg.pose = odom_msg.pose.pose
        
        # Publish
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToPoseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()