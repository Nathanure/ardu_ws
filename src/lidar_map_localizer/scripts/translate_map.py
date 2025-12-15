#!/usr/bin/env python3
"""
Translate Map Point Cloud
Applies translation offset to X, Y, or Z axes

This is useful for aligning maps with simulation environments
or correcting coordinate system offsets.
"""

import numpy as np
import open3d as o3d
import argparse
import sys

def translate_map(input_pcd, output_pcd, x_offset=0, y_offset=0, z_offset=0):
    """
    Translate point cloud by specified offsets
    
    Args:
        input_pcd: Path to input PCD file
        output_pcd: Path to output PCD file
        x_offset: Translation along X axis (meters)
        y_offset: Translation along Y axis (meters)
        z_offset: Translation along Z axis (meters)
    """
    print(f"Loading point cloud from: {input_pcd}")
    
    # Load point cloud
    pcd = o3d.io.read_point_cloud(input_pcd)
    points = np.asarray(pcd.points)
    
    print(f"Loaded {len(points)} points")
    print(f"\nOriginal ranges:")
    print(f"  X: [{points[:, 0].min():.3f}, {points[:, 0].max():.3f}]")
    print(f"  Y: [{points[:, 1].min():.3f}, {points[:, 1].max():.3f}]")
    print(f"  Z: [{points[:, 2].min():.3f}, {points[:, 2].max():.3f}]")
    
    # Apply translation
    print(f"\nApplying translation:")
    if x_offset != 0:
        print(f"  X offset: {x_offset:+.3f}m")
        points[:, 0] += x_offset
    if y_offset != 0:
        print(f"  Y offset: {y_offset:+.3f}m")
        points[:, 1] += y_offset
    if z_offset != 0:
        print(f"  Z offset: {z_offset:+.3f}m")
        points[:, 2] += z_offset
    
    print(f"\nNew ranges:")
    print(f"  X: [{points[:, 0].min():.3f}, {points[:, 0].max():.3f}]")
    print(f"  Y: [{points[:, 1].min():.3f}, {points[:, 1].max():.3f}]")
    print(f"  Z: [{points[:, 2].min():.3f}, {points[:, 2].max():.3f}]")
    
    # Create new point cloud
    translated_pcd = o3d.geometry.PointCloud()
    translated_pcd.points = o3d.utility.Vector3dVector(points)
    
    # Copy colors/normals if they exist
    if pcd.has_colors():
        translated_pcd.colors = pcd.colors
    if pcd.has_normals():
        translated_pcd.normals = pcd.normals
    
    # Save translated point cloud
    o3d.io.write_point_cloud(output_pcd, translated_pcd)
    print(f"\n✅ Translated point cloud saved to: {output_pcd}")
    
    return points

def main():
    parser = argparse.ArgumentParser(
        description='Translate map point cloud along X, Y, or Z axes',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Translate X by +3 meters
  python3 translate_map.py warehouse_map.pcd --x 3
  
  # Translate X by +3 and Y by -2
  python3 translate_map.py warehouse_map.pcd --x 3 --y -2
  
  # Translate with custom output name
  python3 translate_map.py warehouse_map.pcd --x 3 -o warehouse_map_shifted.pcd
        """
    )
    
    parser.add_argument('input', help='Input PCD file')
    parser.add_argument('--x', type=float, default=0, help='X-axis offset in meters')
    parser.add_argument('--y', type=float, default=0, help='Y-axis offset in meters')
    parser.add_argument('--z', type=float, default=0, help='Z-axis offset in meters')
    parser.add_argument('--output', '-o', help='Output PCD file (default: input_translated.pcd)')
    
    args = parser.parse_args()
    
    # Check if any offset is specified
    if args.x == 0 and args.y == 0 and args.z == 0:
        print("Error: No translation specified. Use --x, --y, or --z to set offsets.")
        parser.print_help()
        return 1
    
    # Set output path
    if args.output:
        output_pcd = args.output
    else:
        # Generate descriptive output name
        offset_str = []
        if args.x != 0:
            offset_str.append(f"x{args.x:+.0f}")
        if args.y != 0:
            offset_str.append(f"y{args.y:+.0f}")
        if args.z != 0:
            offset_str.append(f"z{args.z:+.0f}")
        
        suffix = "_".join(offset_str)
        output_pcd = args.input.replace('.pcd', f'_{suffix}.pcd')
    
    try:
        translate_map(args.input, output_pcd, args.x, args.y, args.z)
        
        print("\n" + "="*60)
        print("NEXT STEPS:")
        print("="*60)
        print(f"1. Use the translated map in your launch file:")
        print(f"   map_pcd_file:={output_pcd}")
        print(f"\n2. Restart your system with the new map:")
        print("   ros2 launch nav3d_planner localization.launch.py \\")
        print(f"     map_file:={output_pcd} \\")
        print("     use_sim_time:=true")
        
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    
    return 0

if __name__ == '__main__':
    sys.exit(main())