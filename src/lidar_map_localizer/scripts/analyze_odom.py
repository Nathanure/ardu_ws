#!/usr/bin/env python3
import sqlite3
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation
import argparse
import sys
import os
import csv

def extract_odometry(cursor, topic_name):
    """Extract odometry data from ROS2 bag database."""
    cursor.execute(f"""
        SELECT timestamp, data 
        FROM messages 
        WHERE topic_id = (SELECT id FROM topics WHERE name = '{topic_name}')
        ORDER BY timestamp
    """)
    
    msg_type = get_message('nav_msgs/msg/Odometry')
    times, x, y, z = [], [], [], []
    roll, pitch, yaw = [], [], []
    vx, vy, vz = [], [], []
    qx, qy, qz, qw = [], [], [], []
    
    for row in cursor.fetchall():
        timestamp = row[0] * 1e-9
        msg = deserialize_message(row[1], msg_type)
        times.append(timestamp)
        
        # Position
        x.append(msg.pose.pose.position.x)
        y.append(msg.pose.pose.position.y)
        z.append(msg.pose.pose.position.z)
        
        # Orientation (quaternion)
        qx.append(msg.pose.pose.orientation.x)
        qy.append(msg.pose.pose.orientation.y)
        qz.append(msg.pose.pose.orientation.z)
        qw.append(msg.pose.pose.orientation.w)
        
        # Convert quaternion to euler angles
        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        r = Rotation.from_quat(quat)
        euler = r.as_euler('xyz', degrees=False)
        roll.append(euler[0])
        pitch.append(euler[1])
        yaw.append(euler[2])
        
        # Linear velocities
        vx.append(msg.twist.twist.linear.x)
        vy.append(msg.twist.twist.linear.y)
        vz.append(msg.twist.twist.linear.z)
    
    return {
        'time': np.array(times), 'x': np.array(x), 'y': np.array(y), 'z': np.array(z),
        'qx': np.array(qx), 'qy': np.array(qy), 'qz': np.array(qz), 'qw': np.array(qw),
        'roll': np.array(roll), 'pitch': np.array(pitch), 'yaw': np.array(yaw),
        'vx': np.array(vx), 'vy': np.array(vy), 'vz': np.array(vz)
    }

def export_raw_data(gt, kiss, output_prefix):
    """Export raw odometry data to CSV files."""
    # Export Gazebo ground truth
    gazebo_file = f'{output_prefix}_gazebo_raw.csv'
    with open(gazebo_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_sec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 
                        'roll_rad', 'pitch_rad', 'yaw_rad', 'vx', 'vy', 'vz'])
        for i in range(len(gt['time'])):
            writer.writerow([
                gt['time'][i], gt['x'][i], gt['y'][i], gt['z'][i],
                gt['qx'][i], gt['qy'][i], gt['qz'][i], gt['qw'][i],
                gt['roll'][i], gt['pitch'][i], gt['yaw'][i],
                gt['vx'][i], gt['vy'][i], gt['vz'][i]
            ])
    print(f"Exported Gazebo raw data to: {gazebo_file}")
    
    # Export KISS-ICP
    kiss_file = f'{output_prefix}_kiss_raw.csv'
    with open(kiss_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_sec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
                        'roll_rad', 'pitch_rad', 'yaw_rad', 'vx', 'vy', 'vz'])
        for i in range(len(kiss['time'])):
            writer.writerow([
                kiss['time'][i], kiss['x'][i], kiss['y'][i], kiss['z'][i],
                kiss['qx'][i], kiss['qy'][i], kiss['qz'][i], kiss['qw'][i],
                kiss['roll'][i], kiss['pitch'][i], kiss['yaw'][i],
                kiss['vx'][i], kiss['vy'][i], kiss['vz'][i]
            ])
    print(f"Exported KISS-ICP raw data to: {kiss_file}")

def export_error_data(gt, kiss, output_prefix):
    """Calculate and export error metrics to CSV."""
    # Interpolate KISS data to match Gazebo timestamps
    kiss_x_interp = interp1d(kiss['time'], kiss['x'], fill_value='extrapolate')
    kiss_y_interp = interp1d(kiss['time'], kiss['y'], fill_value='extrapolate')
    kiss_z_interp = interp1d(kiss['time'], kiss['z'], fill_value='extrapolate')
    kiss_yaw_interp = interp1d(kiss['time'], kiss['yaw'], fill_value='extrapolate')
    
    # Calculate position errors
    error_x = gt['x'] - kiss_x_interp(gt['time'])
    error_y = gt['y'] - kiss_y_interp(gt['time'])
    error_z = gt['z'] - kiss_z_interp(gt['time'])
    error_2d = np.sqrt(error_x**2 + error_y**2)
    error_3d = np.sqrt(error_x**2 + error_y**2 + error_z**2)
    
    # Calculate yaw error and wrap to [-180, 180]
    yaw_error = np.degrees(gt['yaw'] - kiss_yaw_interp(gt['time']))
    yaw_error = (yaw_error + 180) % 360 - 180
    
    # Calculate drift rate (smoothed)
    window = min(50, len(error_2d) // 10)
    error_rate = np.gradient(error_2d, gt['time'])
    error_rate_smooth = np.convolve(error_rate, np.ones(window)/window, mode='same')
    
    # Export errors to CSV
    error_file = f'{output_prefix}_errors.csv'
    with open(error_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_sec', 'error_x_m', 'error_y_m', 'error_z_m', 
                        'error_2d_m', 'error_3d_m', 'error_yaw_deg', 'error_rate_m_per_s'])
        for i in range(len(gt['time'])):
            writer.writerow([
                gt['time'][i], error_x[i], error_y[i], error_z[i],
                error_2d[i], error_3d[i], yaw_error[i], error_rate_smooth[i]
            ])
    print(f"Exported error data to: {error_file}")
    
    return error_x, error_y, error_z, error_2d, error_3d, yaw_error, error_rate_smooth

def export_statistics(gt, kiss, error_2d, error_3d, error_z, yaw_error, error_rate_smooth, output_prefix):
    """Export summary statistics to text and CSV files."""
    total_time = gt['time'][-1]
    distance_traveled = np.sum(np.sqrt(np.diff(gt['x'])**2 + np.diff(gt['y'])**2))
    drift_per_meter = error_2d[-1] / distance_traveled if distance_traveled > 0 else 0
    
    # Export text summary
    summary_file = f'{output_prefix}_summary.txt'
    with open(summary_file, 'w') as f:
        f.write("="*60 + "\n")
        f.write("DRIFT ANALYSIS SUMMARY\n")
        f.write("="*60 + "\n")
        f.write(f"Recording Duration: {total_time:.1f} seconds ({total_time/60:.1f} minutes)\n")
        f.write(f"Gazebo Messages: {len(gt['time'])}\n")
        f.write(f"KISS-ICP Messages: {len(kiss['time'])}\n")
        f.write(f"\n2D Position Error (X-Y plane):\n")
        f.write(f"  Mean: {np.mean(error_2d):.3f} m\n")
        f.write(f"  Median: {np.median(error_2d):.3f} m\n")
        f.write(f"  Std Dev: {np.std(error_2d):.3f} m\n")
        f.write(f"  Max: {np.max(error_2d):.3f} m at t={gt['time'][np.argmax(error_2d)]:.1f}s\n")
        f.write(f"  Final: {error_2d[-1]:.3f} m\n")
        f.write(f"\n3D Position Error:\n")
        f.write(f"  Mean: {np.mean(error_3d):.3f} m\n")
        f.write(f"  Max: {np.max(error_3d):.3f} m\n")
        f.write(f"\nHeight (Z) Error:\n")
        f.write(f"  Mean: {np.mean(np.abs(error_z)):.3f} m\n")
        f.write(f"  Max: {np.max(np.abs(error_z)):.3f} m\n")
        f.write(f"\nYaw (Heading) Error:\n")
        f.write(f"  Mean: {np.mean(np.abs(yaw_error)):.2f} degrees\n")
        f.write(f"  Max: {np.max(np.abs(yaw_error)):.2f} degrees\n")
        f.write(f"\nDrift Rate:\n")
        f.write(f"  Average: {np.mean(error_rate_smooth):.4f} m/s\n")
        f.write(f"  Max: {np.max(error_rate_smooth):.4f} m/s at t={gt['time'][np.argmax(error_rate_smooth)]:.1f}s\n")
        f.write(f"\nDistance Traveled: {distance_traveled:.1f} m\n")
        f.write(f"Drift per meter: {drift_per_meter*100:.2f}% ({drift_per_meter:.4f} m/m)\n")
        f.write("="*60 + "\n")
    print(f"Exported summary to: {summary_file}")
    
    # Export CSV statistics
    stats_file = f'{output_prefix}_statistics.csv'
    with open(stats_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['metric', 'value', 'unit'])
        writer.writerow(['recording_duration', total_time, 'seconds'])
        writer.writerow(['gazebo_messages', len(gt['time']), 'count'])
        writer.writerow(['kiss_messages', len(kiss['time']), 'count'])
        writer.writerow(['error_2d_mean', np.mean(error_2d), 'meters'])
        writer.writerow(['error_2d_median', np.median(error_2d), 'meters'])
        writer.writerow(['error_2d_std', np.std(error_2d), 'meters'])
        writer.writerow(['error_2d_max', np.max(error_2d), 'meters'])
        writer.writerow(['error_2d_final', error_2d[-1], 'meters'])
        writer.writerow(['error_3d_mean', np.mean(error_3d), 'meters'])
        writer.writerow(['error_3d_max', np.max(error_3d), 'meters'])
        writer.writerow(['error_z_mean', np.mean(np.abs(error_z)), 'meters'])
        writer.writerow(['error_z_max', np.max(np.abs(error_z)), 'meters'])
        writer.writerow(['error_yaw_mean', np.mean(np.abs(yaw_error)), 'degrees'])
        writer.writerow(['error_yaw_max', np.max(np.abs(yaw_error)), 'degrees'])
        writer.writerow(['drift_rate_avg', np.mean(error_rate_smooth), 'm/s'])
        writer.writerow(['drift_rate_max', np.max(error_rate_smooth), 'm/s'])
        writer.writerow(['distance_traveled', distance_traveled, 'meters'])
        writer.writerow(['drift_per_meter', drift_per_meter, 'm/m'])
        writer.writerow(['drift_per_meter_percent', drift_per_meter*100, 'percent'])
    print(f"Exported statistics CSV to: {stats_file}")

def export_trajectory_comparison(gt, kiss, output_prefix):
    """Export synchronized trajectory data for easy plotting."""
    traj_file = f'{output_prefix}_trajectory.csv'
    with open(traj_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_sec', 'gazebo_x', 'gazebo_y', 'gazebo_z', 
                        'kiss_x', 'kiss_y', 'kiss_z'])
        
        # Interpolate KISS to match Gazebo timestamps
        kiss_x_interp = interp1d(kiss['time'], kiss['x'], fill_value='extrapolate')
        kiss_y_interp = interp1d(kiss['time'], kiss['y'], fill_value='extrapolate')
        kiss_z_interp = interp1d(kiss['time'], kiss['z'], fill_value='extrapolate')
        
        for i in range(len(gt['time'])):
            writer.writerow([
                gt['time'][i], 
                gt['x'][i], gt['y'][i], gt['z'][i],
                kiss_x_interp(gt['time'][i]), 
                kiss_y_interp(gt['time'][i]), 
                kiss_z_interp(gt['time'][i])
            ])
    print(f"Exported trajectory comparison to: {traj_file}")

def create_trajectory_plots(gt, kiss, output_prefix, bag_name):
    """Create window 1: Trajectory and Position Comparison."""
    fig = plt.figure(figsize=(18, 10))
    fig.canvas.manager.set_window_title('Trajectory Comparison')
    gs = fig.add_gridspec(2, 3, hspace=0.35, wspace=0.35, 
                          left=0.08, right=0.95, top=0.92, bottom=0.08)
    
    # 1. 2D Trajectory (Large, spans 2 rows)
    ax1 = fig.add_subplot(gs[0:2, 0])
    ax1.plot(gt['x'], gt['y'], 'g-', label='Gazebo (Ground Truth)', linewidth=2.5)
    ax1.plot(kiss['x'], kiss['y'], 'r--', label='KISS-ICP', linewidth=2.5)
    ax1.scatter(gt['x'][0], gt['y'][0], c='blue', s=100, marker='o', 
                label='Start', zorder=5, edgecolors='black', linewidth=1.5)
    ax1.scatter(gt['x'][-1], gt['y'][-1], c='purple', s=100, marker='s', 
                label='End', zorder=5, edgecolors='black', linewidth=1.5)
    ax1.set_xlabel('X (m)', fontsize=11)
    ax1.set_ylabel('Y (m)', fontsize=11)
    ax1.set_title('2D Trajectory Comparison', fontsize=13, fontweight='bold')
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # 2. 3D Trajectory
    ax2 = fig.add_subplot(gs[0, 1], projection='3d')
    ax2.plot(gt['x'], gt['y'], gt['z'], 'g-', label='Gazebo', linewidth=2)
    ax2.plot(kiss['x'], kiss['y'], kiss['z'], 'r--', label='KISS-ICP', linewidth=2)
    ax2.set_xlabel('X (m)', fontsize=9)
    ax2.set_ylabel('Y (m)', fontsize=9)
    ax2.set_zlabel('Z (m)', fontsize=9)
    ax2.set_title('3D Trajectory', fontsize=12, fontweight='bold')
    ax2.legend(fontsize=9)
    
    # 3. X Position vs Time
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(gt['time'], gt['x'], 'g-', label='Gazebo', linewidth=2)
    ax3.plot(kiss['time'], kiss['x'], 'r--', label='KISS-ICP', linewidth=2)
    ax3.set_xlabel('Time (s)', fontsize=10)
    ax3.set_ylabel('X Position (m)', fontsize=10)
    ax3.set_title('X Position vs Time', fontsize=12, fontweight='bold')
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3)
    
    # 4. Y Position vs Time
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(gt['time'], gt['y'], 'g-', label='Gazebo', linewidth=2)
    ax4.plot(kiss['time'], kiss['y'], 'r--', label='KISS-ICP', linewidth=2)
    ax4.set_xlabel('Time (s)', fontsize=10)
    ax4.set_ylabel('Y Position (m)', fontsize=10)
    ax4.set_title('Y Position vs Time', fontsize=12, fontweight='bold')
    ax4.legend(fontsize=9)
    ax4.grid(True, alpha=0.3)
    
    # 5. Z Position (Height)
    ax5 = fig.add_subplot(gs[1, 2])
    ax5.plot(gt['time'], gt['z'], 'g-', label='Gazebo', linewidth=2)
    ax5.plot(kiss['time'], kiss['z'], 'r--', label='KISS-ICP', linewidth=2)
    ax5.set_xlabel('Time (s)', fontsize=10)
    ax5.set_ylabel('Z Position (m)', fontsize=10)
    ax5.set_title('Height Over Time', fontsize=12, fontweight='bold')
    ax5.legend(fontsize=9)
    ax5.grid(True, alpha=0.3)
    
    plt.suptitle(f'Trajectory Comparison: {bag_name}', fontsize=15, fontweight='bold')
    
    output_file = f'{output_prefix}_trajectories.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Trajectory plot saved to: {output_file}")
    
    return fig

def create_error_plots(gt, kiss, error_x, error_y, error_z, error_2d, 
                       error_3d, yaw_error, error_rate_smooth, output_prefix, bag_name):
    """Create window 2: Error Analysis."""
    fig = plt.figure(figsize=(18, 10))
    fig.canvas.manager.set_window_title('Error Analysis')
    gs = fig.add_gridspec(2, 3, hspace=0.35, wspace=0.35,
                          left=0.08, right=0.95, top=0.92, bottom=0.08)
    
    # 1. Position Errors (X, Y, Z)
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(gt['time'], error_x, 'r-', label='X Error', linewidth=2)
    ax1.plot(gt['time'], error_y, 'b-', label='Y Error', linewidth=2)
    ax1.plot(gt['time'], error_z, 'g-', label='Z Error', linewidth=2)
    ax1.set_xlabel('Time (s)', fontsize=10)
    ax1.set_ylabel('Position Error (m)', fontsize=10)
    ax1.set_title('Position Errors (X, Y, Z)', fontsize=12, fontweight='bold')
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    # 2. 2D vs 3D Error
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(gt['time'], error_2d, 'b-', label='2D Error (X-Y)', linewidth=2.5)
    ax2.plot(gt['time'], error_3d, 'r--', label='3D Error (X-Y-Z)', linewidth=2.5)
    max_idx = np.argmax(error_2d)
    ax2.plot(gt['time'][max_idx], error_2d[max_idx], 'ro', markersize=10, 
             label=f'Max: {error_2d[max_idx]:.2f}m')
    ax2.set_xlabel('Time (s)', fontsize=10)
    ax2.set_ylabel('Position Error (m)', fontsize=10)
    ax2.set_title('Euclidean Distance Error', fontsize=12, fontweight='bold')
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    # 3. Yaw (Heading) Comparison
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(gt['time'], np.degrees(gt['yaw']), 'g-', label='Gazebo', linewidth=2)
    ax3.plot(kiss['time'], np.degrees(kiss['yaw']), 'r--', label='KISS-ICP', linewidth=2)
    ax3.set_xlabel('Time (s)', fontsize=10)
    ax3.set_ylabel('Yaw (degrees)', fontsize=10)
    ax3.set_title('Heading Angle Over Time', fontsize=12, fontweight='bold')
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3)
    
    # 4. Yaw Error
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.plot(gt['time'], yaw_error, 'purple', linewidth=2)
    ax4.set_xlabel('Time (s)', fontsize=10)
    ax4.set_ylabel('Yaw Error (degrees)', fontsize=10)
    ax4.set_title('Heading Angle Error', fontsize=12, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax4.fill_between(gt['time'], yaw_error, alpha=0.3, color='purple')
    
    # 5. Error Accumulation Rate
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.plot(gt['time'], error_rate_smooth, 'darkred', linewidth=2.5)
    ax5.set_xlabel('Time (s)', fontsize=10)
    ax5.set_ylabel('Error Rate (m/s)', fontsize=10)
    ax5.set_title('Drift Rate (Smoothed)', fontsize=12, fontweight='bold')
    ax5.grid(True, alpha=0.3)
    ax5.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax5.fill_between(gt['time'], error_rate_smooth, alpha=0.3, 
                     where=(error_rate_smooth > 0), color='red', label='Increasing drift')
    ax5.fill_between(gt['time'], error_rate_smooth, alpha=0.3, 
                     where=(error_rate_smooth < 0), color='green', label='Decreasing drift')
    ax5.legend(fontsize=8)
    
    # 6. Error Statistics by Phase
    ax6 = fig.add_subplot(gs[1, 2])
    total_time = gt['time'][-1]
    num_phases = min(4, max(1, int(total_time / 100)))
    phase_duration = total_time / num_phases
    
    phases = []
    phase_errors = []
    for i in range(num_phases):
        start = i * phase_duration
        end = (i + 1) * phase_duration
        phases.append(f'{int(start)}-{int(end)}s')
        mask = (gt['time'] >= start) & (gt['time'] < end)
        phase_errors.append(np.mean(error_2d[mask]))
    
    colors = plt.cm.RdYlGn_r(np.linspace(0.2, 0.8, num_phases))
    bars = ax6.bar(phases, phase_errors, color=colors, edgecolor='black', linewidth=1.5)
    ax6.set_ylabel('Mean Error (m)', fontsize=10)
    ax6.set_title('Average Drift by Time Phase', fontsize=12, fontweight='bold')
    ax6.grid(True, axis='y', alpha=0.3)
    plt.setp(ax6.xaxis.get_majorticklabels(), rotation=45, ha='right')
    
    for bar, error in zip(bars, phase_errors):
        height = bar.get_height()
        ax6.text(bar.get_x() + bar.get_width()/2., height,
                f'{error:.2f}m', ha='center', va='bottom', fontsize=9)
    
    plt.suptitle(f'Error Analysis: {bag_name}', fontsize=15, fontweight='bold')
    
    output_file = f'{output_prefix}_errors.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Error analysis plot saved to: {output_file}")
    
    return fig

def analyze_bag(bag_file, output_prefix='analysis', export_data=True):
    """Main analysis function."""
    if not os.path.exists(bag_file):
        print(f"Error: Bag file '{bag_file}' not found!")
        sys.exit(1)
    
    print(f"Analyzing bag file: {bag_file}")
    
    # Connect to database
    conn = sqlite3.connect(bag_file)
    cursor = conn.cursor()
    
    # Check available topics
    cursor.execute("SELECT name FROM topics")
    topics = [row[0] for row in cursor.fetchall()]
    print(f"Available topics: {', '.join(topics)}")
    
    if '/odometry' not in topics or '/kiss/odometry' not in topics:
        print("Error: Required topics '/odometry' and '/kiss/odometry' not found in bag!")
        conn.close()
        sys.exit(1)
    
    # Extract odometry data
    print("Extracting Gazebo odometry...")
    gt = extract_odometry(cursor, '/odometry')
    print("Extracting KISS-ICP odometry...")
    kiss = extract_odometry(cursor, '/kiss/odometry')
    
    if len(gt['time']) == 0 or len(kiss['time']) == 0:
        print("Error: No data found in odometry topics!")
        conn.close()
        sys.exit(1)
    
    print(f"Gazebo messages: {len(gt['time'])}")
    print(f"KISS-ICP messages: {len(kiss['time'])}")
    
    # Normalize timestamps to start at 0
    gt['time'] -= gt['time'][0]
    kiss['time'] -= kiss['time'][0]
    
    # Export raw data if requested
    if export_data:
        print("\nExporting raw data...")
        export_raw_data(gt, kiss, output_prefix)
        export_trajectory_comparison(gt, kiss, output_prefix)
    
    # Calculate and export errors
    print("Calculating errors...")
    error_x, error_y, error_z, error_2d, error_3d, yaw_error, error_rate_smooth = \
        export_error_data(gt, kiss, output_prefix)
    
    # Export statistics
    export_statistics(gt, kiss, error_2d, error_3d, error_z, yaw_error, 
                     error_rate_smooth, output_prefix)
    
    # Create plots in two separate windows
    print("\nGenerating plots...")
    bag_name = os.path.basename(bag_file)
    
    fig1 = create_trajectory_plots(gt, kiss, output_prefix, bag_name)
    fig2 = create_error_plots(gt, kiss, error_x, error_y, error_z, error_2d, 
                              error_3d, yaw_error, error_rate_smooth, 
                              output_prefix, bag_name)
    
    plt.show()
    
    conn.close()
    
    # Print summary to console
    total_time = gt['time'][-1]
    distance_traveled = np.sum(np.sqrt(np.diff(gt['x'])**2 + np.diff(gt['y'])**2))
    drift_per_meter = error_2d[-1] / distance_traveled if distance_traveled > 0 else 0
    
    print("\n" + "="*60)
    print("DRIFT ANALYSIS SUMMARY")
    print("="*60)
    print(f"Recording Duration: {total_time:.1f} seconds ({total_time/60:.1f} minutes)")
    print(f"\n2D Position Error (X-Y plane):")
    print(f"  Mean: {np.mean(error_2d):.3f} m")
    print(f"  Median: {np.median(error_2d):.3f} m")
    print(f"  Std Dev: {np.std(error_2d):.3f} m")
    print(f"  Max: {np.max(error_2d):.3f} m at t={gt['time'][np.argmax(error_2d)]:.1f}s")
    print(f"  Final: {error_2d[-1]:.3f} m")
    print(f"\n3D Position Error:")
    print(f"  Mean: {np.mean(error_3d):.3f} m")
    print(f"  Max: {np.max(error_3d):.3f} m")
    print(f"\nHeight (Z) Error:")
    print(f"  Mean: {np.mean(np.abs(error_z)):.3f} m")
    print(f"  Max: {np.max(np.abs(error_z)):.3f} m")
    print(f"\nYaw (Heading) Error:")
    print(f"  Mean: {np.mean(np.abs(yaw_error)):.2f} degrees")
    print(f"  Max: {np.max(np.abs(yaw_error)):.2f} degrees")
    print(f"\nDrift Rate:")
    print(f"  Average: {np.mean(error_rate_smooth):.4f} m/s")
    print(f"  Max: {np.max(error_rate_smooth):.4f} m/s at t={gt['time'][np.argmax(error_rate_smooth)]:.1f}s")
    print(f"\nDistance Traveled: {distance_traveled:.1f} m")
    print(f"Drift per meter: {drift_per_meter*100:.2f}% ({drift_per_meter:.4f} m/m)")
    print("="*60)
    print(f"\n📊 All data exported with prefix: {output_prefix}_*")

def main():
    """Command-line interface."""
    parser = argparse.ArgumentParser(
        description='Analyze drift between Gazebo ground truth and KISS-ICP odometry',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  python3 analyze_odom.py my_bag/my_bag_0.db3
  python3 analyze_odom.py ~/rosbags/test_bag_0.db3 -o test_results
  python3 analyze_odom.py my_bag.db3 --no-export
        '''
    )
    
    parser.add_argument('bag_file', 
                       help='Path to the ROS2 bag database file (.db3)')
    parser.add_argument('-o', '--output', 
                       default='analysis',
                       help='Output file prefix (default: analysis)')
    parser.add_argument('--no-export', 
                       action='store_true',
                       help='Skip exporting raw data to CSV')
    
    args = parser.parse_args()
    
    analyze_bag(args.bag_file, args.output, export_data=not args.no_export)

if __name__ == '__main__':
    main()