#!/usr/bin/env python3
"""
Robust TF broadcaster for map -> odom transform
Fixes timestamp issues and ensures continuous transform availability
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.duration import Duration
import threading

class OdomToTFNode(Node):
    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Store last transform for republishing
        self.last_transform = None
        self.transform_lock = threading.Lock()
        
        # Subscribe to corrected odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            'input_odom',
            self.odom_callback,
            10
        )
        
        # Timer to republish transform at high rate (50Hz)
        # This ensures transform is always available
        self.timer = self.create_timer(0.02, self.republish_transform)
        
        self.msg_count = 0
        self.get_logger().info('TF Broadcaster initialized (Robust Mode with 50Hz republish)')

    def odom_callback(self, odom_msg):
        """Receive corrected odometry and update stored transform"""
        self.msg_count += 1
        
        with self.transform_lock:
            tf_msg = TransformStamped()
            
            # Use current time to avoid stale transform warnings
            current_time = self.get_clock().now()
            tf_msg.header.stamp = current_time.to_msg()
            
            # Set frames
            tf_msg.header.frame_id = 'map'
            tf_msg.child_frame_id = 'odom'
            
            # Copy pose from odometry
            tf_msg.transform.translation.x = odom_msg.pose.pose.position.x
            tf_msg.transform.translation.y = odom_msg.pose.pose.position.y
            tf_msg.transform.translation.z = odom_msg.pose.pose.position.z
            tf_msg.transform.rotation = odom_msg.pose.pose.orientation
            
            # Store for republishing
            self.last_transform = tf_msg
            
            # Broadcast immediately
            self.tf_broadcaster.sendTransform(tf_msg)
        
        if self.msg_count % 20 == 0:
            self.get_logger().info(
                f'Published {self.msg_count} transforms (map->odom) | '
                f'Pos: [{tf_msg.transform.translation.x:.2f}, '
                f'{tf_msg.transform.translation.y:.2f}, '
                f'{tf_msg.transform.translation.z:.2f}]'
            )

    def republish_transform(self):
        """Republish last transform at high frequency"""
        with self.transform_lock:
            if self.last_transform is not None:
                # Update timestamp to current time
                updated_tf = TransformStamped()
                updated_tf.header.frame_id = self.last_transform.header.frame_id
                updated_tf.child_frame_id = self.last_transform.child_frame_id
                updated_tf.header.stamp = self.get_clock().now().to_msg()
                updated_tf.transform = self.last_transform.transform
                
                self.tf_broadcaster.sendTransform(updated_tf)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()