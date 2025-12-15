import open3d as o3d
import numpy as np
import sys
import argparse
import os

def main():
    # Set up command line argument parsing
    parser = argparse.ArgumentParser(description="Convert PLY/PCD to 32-bit Binary PCD for ROS 2 compatibility.")
    parser.add_argument("input", help="Path to the input file (.ply or .pcd)")
    parser.add_argument("-o", "--output", help="Path to the output file (default: output.pcd)", default="output.pcd")
    
    args = parser.parse_args()
    
    input_file = args.input
    output_file = args.output

    if not os.path.exists(input_file):
        print(f"Error: Input file '{input_file}' not found.")
        sys.exit(1)

    print(f"1. Loading: {input_file}")
    
    # Load the file using Legacy IO
    # We use Legacy because it handles generic imports well, 
    # and we are immediately extracting to numpy arrays anyway.
    pcd = o3d.io.read_point_cloud(input_file)

    if len(pcd.points) == 0:
        print("Error: Point cloud is empty or failed to load!")
        sys.exit(1)

    print(f"   Points found: {len(pcd.points)}")

    # 2. Convert Data to 32-bit Float (Standard Numpy)
    # This step strips away the 64-bit precision that crashes ROS
    xyz = np.asarray(pcd.points).astype(np.float32)
    
    # Handle Normals
    normals = None
    if pcd.has_normals():
        normals = np.asarray(pcd.normals).astype(np.float32)

    # 3. Stack Data and Prepare Header
    if normals is not None and normals.shape[0] > 0:
        print("   Normals found. Merging data...")
        # Stack [x, y, z, normal_x, normal_y, normal_z]
        combined_data = np.hstack((xyz, normals))
        fields = "x y z normal_x normal_y normal_z"
        size = "4 4 4 4 4 4"
        type_ = "F F F F F F"
        count = "1 1 1 1 1 1"
    else:
        print("   No normals found. Saving XYZ only.")
        combined_data = xyz
        fields = "x y z"
        size = "4 4 4"
        type_ = "F F F"
        count = "1 1 1"

    points_count = combined_data.shape[0]

    # 4. Write Manual Binary PCD
    # We write the bytes manually to guarantee the header matches the data exactly.
    print(f"4. Writing 32-bit binary to: {output_file}")
    
    try:
        with open(output_file, 'wb') as f:
            # Header must be ASCII
            header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS {fields}
SIZE {size}
TYPE {type_}
COUNT {count}
WIDTH {points_count}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {points_count}
DATA binary
"""
            f.write(header.encode('ascii'))
            
            # Data must be Binary Bytes
            f.write(combined_data.tobytes())
            
        print("Done! File saved successfully.")
        
    except IOError as e:
        print(f"Error writing file: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()