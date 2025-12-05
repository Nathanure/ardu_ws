#!/usr/bin/env python3
"""
Point Cloud Frame Fixer
Republishes point cloud with corrected frame_id

This fixes the bug in pcl_ros pcd_to_pointcloud that publishes in base_link frame
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class PointCloudFrameFixer(Node):
    def __init__(self):
        super().__init__('pointcloud_fixer')
        
        # Declare parameters
        self.declare_parameter('target_frame', 'map')
        self.target_frame = self.get_parameter('target_frame').value
        
        # Subscriber to raw point cloud
        self.sub = self.create_subscription(
            PointCloud2,
            'cloud_pcd',
            self.pointcloud_callback,
            10
        )
        
        # Publisher for fixed point cloud
        self.pub = self.create_publisher(
            PointCloud2,
            '/map_pointcloud',
            10
        )
        
        self.get_logger().info(f'Point cloud frame fixer initialized')
        self.get_logger().info(f'  Input: cloud_pcd')
        self.get_logger().info(f'  Output: /map_pointcloud (frame: {self.target_frame})')
    
    def pointcloud_callback(self, msg):
        """Republish point cloud with corrected frame_id"""
        # Create new message with same data but corrected frame
        fixed_msg = PointCloud2()
        fixed_msg.header.stamp = msg.header.stamp
        fixed_msg.header.frame_id = self.target_frame  # ← Fix the frame!
        fixed_msg.height = msg.height
        fixed_msg.width = msg.width
        fixed_msg.fields = msg.fields
        fixed_msg.is_bigendian = msg.is_bigendian
        fixed_msg.point_step = msg.point_step
        fixed_msg.row_step = msg.row_step
        fixed_msg.data = msg.data
        fixed_msg.is_dense = msg.is_dense
        
        # Publish fixed message
        self.pub.publish(fixed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFrameFixer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()