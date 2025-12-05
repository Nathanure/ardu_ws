#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy.spatial import KDTree
import struct
from collections import deque

# TF2 Imports for robust position lookup
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class Planner3D(Node):
    def __init__(self):
        super().__init__('planner_3d_node')
        
        # Parameters
        self.declare_parameter('map_pcd_file', '')
        self.declare_parameter('resolution', 0.2)  # 20cm voxel resolution
        self.declare_parameter('robot_radius', 0.3)  # 30cm safety radius
        self.declare_parameter('max_planning_time', 5.0)  # seconds
        self.declare_parameter('step_size', 0.5)  # RRT step size
        self.declare_parameter('goal_tolerance', 0.5)  # meters
        
        self.map_file = self.get_parameter('map_pcd_file').value
        self.resolution = self.get_parameter('resolution').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.max_planning_time = self.get_parameter('max_planning_time').value
        self.step_size = self.get_parameter('step_size').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        # State variables
        self.current_pose = None
        self.map_points = None
        self.kdtree = None
        self.bounds = None
        
        # TF Buffer for looking up robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/current_pose', self.pose_callback, 10)
        
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        self.map_sub = self.create_subscription(
            PointCloud2, '/map_pointcloud', self.map_callback, 10)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/planning_markers', 10)
        
        self.get_logger().info('3D Planner Node initialized')
        
        # Load map if file provided
        if self.map_file:
            self.load_pcd_map(self.map_file)
    
    def pose_callback(self, msg):
        """Update current robot pose (Backup method)"""
        self.current_pose = msg
    
    def get_robot_pose_from_tf(self):
        """Look up the current robot position from TF tree (map -> base_link)"""
        try:
            # Look up the latest available transform
            t = self.tf_buffer.lookup_transform(
                'map',          # Target frame
                'base_link',    # Source frame (Robot)
                rclpy.time.Time()
            )
            return np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z
            ])
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF Lookup failed: {e}')
            return None

    def map_callback(self, msg):
        """Process incoming point cloud map"""
        self.get_logger().info('Received map point cloud')
        points = []
        
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])
        
        if len(points) > 0:
            self.map_points = np.array(points)
            self.kdtree = KDTree(self.map_points)
            self.bounds = {
                'x_min': np.min(self.map_points[:, 0]),
                'x_max': np.max(self.map_points[:, 0]),
                'y_min': np.min(self.map_points[:, 1]),
                'y_max': np.max(self.map_points[:, 1]),
                'z_min': np.min(self.map_points[:, 2]),
                'z_max': np.max(self.map_points[:, 2])
            }
            self.get_logger().info(f'Map loaded with {len(self.map_points)} points')
            self.get_logger().info(f'Bounds: {self.bounds}')
    
    def load_pcd_map(self, filename):
        """Load PCD file manually (simple ASCII PCD parser)"""
        self.get_logger().info(f'Loading PCD map from {filename}')
        try:
            points = []
            with open(filename, 'r') as f:
                data_started = False
                for line in f:
                    if line.startswith('DATA'):
                        data_started = True
                        continue
                    if data_started:
                        try:
                            parts = line.strip().split()
                            if len(parts) >= 3:
                                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                                points.append([x, y, z])
                        except ValueError:
                            continue
            
            if len(points) > 0:
                self.map_points = np.array(points)
                self.kdtree = KDTree(self.map_points)
                self.bounds = {
                    'x_min': np.min(self.map_points[:, 0]),
                    'x_max': np.max(self.map_points[:, 0]),
                    'y_min': np.min(self.map_points[:, 1]),
                    'y_max': np.max(self.map_points[:, 1]),
                    'z_min': np.min(self.map_points[:, 2]),
                    'z_max': np.max(self.map_points[:, 2])
                }
                self.get_logger().info(f'Map loaded with {len(self.map_points)} points')
        except Exception as e:
            self.get_logger().error(f'Failed to load PCD: {e}')
    
    def is_collision_free(self, point):
        """Check if a point is collision-free"""
        if self.kdtree is None:
            return True
        
        # Query nearest neighbors within robot radius
        distances, _ = self.kdtree.query(point, k=1)
        return distances > self.robot_radius
    
    def is_path_collision_free(self, start, end, num_checks=10):
        """Check if straight line path is collision-free"""
        for i in range(num_checks + 1):
            alpha = i / num_checks
            point = start + alpha * (end - start)
            if not self.is_collision_free(point):
                return False
        return True
    
    def sample_random_point(self):
        """Sample a random point in the map bounds"""
        if self.bounds is None:
            return np.array([0.0, 0.0, 0.0])
        
        x = np.random.uniform(self.bounds['x_min'], self.bounds['x_max'])
        y = np.random.uniform(self.bounds['y_min'], self.bounds['y_max'])
        z = np.random.uniform(self.bounds['z_min'], self.bounds['z_max'])
        return np.array([x, y, z])
    
    def find_nearest_node(self, tree, point):
        """Find nearest node in tree to given point"""
        min_dist = float('inf')
        nearest = None
        for node in tree:
            dist = np.linalg.norm(node - point)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        return nearest
    
    def rrt_plan(self, start, goal, max_iterations=1000):
        """RRT (Rapidly-exploring Random Tree) path planning"""
        self.get_logger().info(f'Planning path from {start} to {goal}')
        
        if self.map_points is None:
            self.get_logger().warn('No map loaded, cannot plan')
            return None
        
        # Check if start and goal are collision-free
        if not self.is_collision_free(start):
            self.get_logger().error(f'Start position {start} is in collision! Radius: {self.robot_radius}')
            # Attempt to find nearest free point
            # return None # Strict mode
        
        if not self.is_collision_free(goal):
            self.get_logger().error('Goal position is in collision!')
            return None
        
        # Initialize tree
        tree = {tuple(start): None}  # node -> parent
        
        for iteration in range(max_iterations):
            # Sample random point (bias towards goal 10% of the time)
            if np.random.random() < 0.1:
                rand_point = goal
            else:
                rand_point = self.sample_random_point()
            
            # Find nearest node
            nearest = self.find_nearest_node(list(tree.keys()), rand_point)
            nearest_arr = np.array(nearest)
            
            # Steer towards random point
            direction = rand_point - nearest_arr
            distance = np.linalg.norm(direction)
            
            if distance > self.step_size:
                direction = direction / distance * self.step_size
            
            new_point = nearest_arr + direction
            
            # Check collision
            if self.is_path_collision_free(nearest_arr, new_point):
                tree[tuple(new_point)] = nearest
                
                # Check if goal is reached
                if np.linalg.norm(new_point - goal) < self.goal_tolerance:
                    tree[tuple(goal)] = tuple(new_point)
                    self.get_logger().info(f'Path found in {iteration} iterations!')
                    return self.extract_path(tree, start, goal)
        
        self.get_logger().warn('Max iterations reached, no path found')
        return None
    
    def extract_path(self, tree, start, goal):
        """Extract path from tree"""
        path = []
        current = tuple(goal)
        
        while current is not None:
            path.append(np.array(current))
            current = tree[current]
        
        path.reverse()
        return self.smooth_path(path)
    
    def smooth_path(self, path):
        """Simple path smoothing by removing unnecessary waypoints"""
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            # Try to connect to farthest visible point
            for j in range(len(path) - 1, i, -1):
                if self.is_path_collision_free(path[i], path[j], num_checks=20):
                    smoothed.append(path[j])
                    i = j
                    break
            else:
                i += 1
        
        return smoothed
    
    def goal_callback(self, msg):
        """Handle goal pose and trigger planning"""
        
        # 1. Try to get start position from TF (Most Accurate)
        start = self.get_robot_pose_from_tf()
        
        # 2. Fallback to topic if TF fails
        if start is None:
            if self.current_pose is not None:
                self.get_logger().warn('TF Lookup failed. Using /current_pose topic as fallback.')
                start = np.array([
                    self.current_pose.pose.position.x,
                    self.current_pose.pose.position.y,
                    self.current_pose.pose.position.z
                ])
            else:
                self.get_logger().error('Cannot plan: Robot position unknown (TF failed and no /current_pose)')
                return

        goal = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        self.get_logger().info(f'Planning from {start} to {goal}')
        
        # Plan path
        path_points = self.rrt_plan(start, goal)
        
        if path_points is not None:
            # Publish path
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.get_clock().now().to_msg()
            
            for point in path_points:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = point[2]
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            
            self.path_pub.publish(path_msg)
            self.visualize_path(path_points)
            self.get_logger().info(f'Published path with {len(path_points)} waypoints')
    
    def visualize_path(self, path_points):
        """Visualize path in RViz"""
        marker_array = MarkerArray()
        
        # Line strip for path
        line_marker = Marker()
        line_marker.header.frame_id = 'map'
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'path'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        
        for point in path_points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            line_marker.points.append(p)
        
        marker_array.markers.append(line_marker)
        
        # Spheres for waypoints
        for i, point in enumerate(path_points):
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'waypoints'
            sphere.id = i + 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = point[0]
            sphere.pose.position.y = point[1]
            sphere.pose.position.z = point[2]
            sphere.scale.x = 0.2
            sphere.scale.y = 0.2
            sphere.scale.z = 0.2
            sphere.color.r = 1.0
            sphere.color.g = 0.0
            sphere.color.b = 0.0
            sphere.color.a = 0.8
            marker_array.markers.append(sphere)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = Planner3D()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()