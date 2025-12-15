#!/usr/bin/env python3
"""
Fix Map Ground Plane
Aligns the map's ground plane to Z=0

This fixes localization Z-offset issues by ensuring the map's
coordinate system matches Gazebo's coordinate system.
"""

import numpy as np
import open3d as o3d
import argparse
import sys

def fix_ground_plane(input_pcd, output_pcd, ground_percentile=5):
    """
    Align map's ground plane to Z=0
    
    Args:
        input_pcd: Path to input PCD file
        output_pcd: Path to output PCD file
        ground_percentile: Percentile of Z values to consider as ground (default: 5%)
    """
    print(f"Loading point cloud from: {input_pcd}")
    
    # Load point cloud
    pcd = o3d.io.read_point_cloud(input_pcd)
    points = np.asarray(pcd.points)
    
    print(f"Loaded {len(points)} points")
    print(f"Original Z range: [{points[:, 2].min():.3f}, {points[:, 2].max():.3f}]")
    
    # Find ground level (lowest 5% of points)
    ground_z = np.percentile(points[:, 2], ground_percentile)
    print(f"Detected ground level at Z = {ground_z:.3f}m")
    
    # Shift all points so ground is at Z=0
    z_offset = -ground_z
    points[:, 2] += z_offset
    
    print(f"Applied Z-offset: {z_offset:.3f}m")
    print(f"New Z range: [{points[:, 2].min():.3f}, {points[:, 2].max():.3f}]")
    
    # Remove points below ground (noise)
    noise_threshold = -0.1  # Remove points more than 10cm below ground
    valid_points = points[points[:, 2] > noise_threshold]
    removed = len(points) - len(valid_points)
    
    if removed > 0:
        print(f"Removed {removed} noise points below ground")
        points = valid_points
    
    # Create new point cloud
    fixed_pcd = o3d.geometry.PointCloud()
    fixed_pcd.points = o3d.utility.Vector3dVector(points)
    
    # Copy colors/normals if they exist
    if pcd.has_colors():
        fixed_pcd.colors = pcd.colors
    if pcd.has_normals():
        fixed_pcd.normals = pcd.normals
    
    # Save fixed point cloud
    o3d.io.write_point_cloud(output_pcd, fixed_pcd)
    print(f"\nFixed point cloud saved to: {output_pcd}")
    print("✅ Ground plane aligned to Z=0")
    
    return z_offset

def main():
    parser = argparse.ArgumentParser(description='Fix map ground plane alignment')
    parser.add_argument('input', help='Input PCD file')
    parser.add_argument('--output', '-o', help='Output PCD file (default: input_fixed.pcd)')
    parser.add_argument('--percentile', '-p', type=float, default=5.0,
                       help='Percentile for ground detection (default: 5)')
    
    args = parser.parse_args()
    
    # Set output path
    if args.output:
        output_pcd = args.output
    else:
        output_pcd = args.input.replace('.pcd', '_fixed.pcd')
    
    try:
        z_offset = fix_ground_plane(args.input, output_pcd, args.percentile)
        
        print("\n" + "="*60)
        print("NEXT STEPS:")
        print("="*60)
        print(f"1. Use the fixed map in your launch file:")
        print(f"   map_pcd_file:={output_pcd}")
        print(f"\n2. The Z-offset was {z_offset:.3f}m")
        print(f"   This explains why your drone appeared at Z={-0.32:.3f}m")
        print("\n3. Restart localization with the fixed map:")
        print("   ros2 launch nav3d_planner localization.launch.py \\")
        print(f"     map_file:={output_pcd} \\")
        print("     use_sim_time:=true")
        
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    
    return 0

if __name__ == '__main__':
    sys.exit(main())