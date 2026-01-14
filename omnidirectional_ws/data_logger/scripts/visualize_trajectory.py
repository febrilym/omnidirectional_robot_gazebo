#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
import sys
from mpl_toolkits.mplot3d import Axes3D
import math
from datetime import datetime

def load_data(csv_file):
    try:
        print(f"Loading data from: {csv_file}")
        df = pd.read_csv(csv_file)
        
        print(f"Successfully loaded {len(df)} rows")
        print(f"Columns: {list(df.columns)}")
        
        # Show basic statistics
        if len(df) > 0:
            print(f"  Time range: {df.iloc[0,0]:.2f} to {df.iloc[-1,0]:.2f} seconds")
        
        return df
    except Exception as e:
        print(f"Error loading CSV: {e}")
        return None

def plot_trajectory(df, output_dir, robot_name):
    print("Creating trajectory plot...")
    
    # find position columns
    x_col = None
    y_col = None
    
    for col in df.columns:
        col_lower = col.lower()
        if 'x' in col_lower and ('pos' in col_lower or col_lower == 'x'):
            x_col = col
        elif 'y' in col_lower and ('pos' in col_lower or col_lower == 'y'):
            y_col = col
    
    if x_col is None or y_col not in df.columns:
        print("Position data not found")
        return None
    
    # create figure
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle(f'Robot {robot_name} - Movement Analysis', fontsize=16)
    
    # 2D trajectory
    ax = axes[0, 0]
    ax.plot(df[x_col], df[y_col], 'b-', alpha=0.6, linewidth=1.5)
    ax.scatter(df[x_col].iloc[0], df[y_col].iloc[0], 
               color='green', s=100, marker='o', label='Start', zorder=5)
    ax.scatter(df[x_col].iloc[-1], df[y_col].iloc[-1], 
               color='red', s=100, marker='s', label='End', zorder=5)
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title('2D Trajectory')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.axis('equal')
    
    # position vs time
    ax = axes[0, 1]
    # 'timestamp' to 'time', otherwise use index
    if df.columns[0] == 'timestamp' or 'time' in df.columns[0].lower():
        time = df.iloc[:, 0] - df.iloc[0, 0]
    else:
        time = np.arange(len(df)) / 20.0
    
    ax.plot(time, df[x_col], 'r-', label='X', linewidth=1.5)
    ax.plot(time, df[y_col], 'g-', label='Y', linewidth=1.5)
    
    # z position
    z_col = None
    for col in df.columns:
        if 'z' in col.lower() and ('pos' in col.lower() or col.lower() == 'z'):
            z_col = col
            break
    
    if z_col and z_col in df.columns:
        ax.plot(time, df[z_col], 'b-', label='Z', linewidth=1.5)
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (m)')
    ax.set_title('Position vs Time')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # velocity if available
    ax = axes[1, 0]
    velocity_plotted = False
    
    # look for velocity columns
    vx_col = None
    for col in df.columns:
        col_lower = col.lower()
        if ('vel' in col_lower or 'velocity' in col_lower) and ('x' in col_lower or 'lin' in col_lower):
            vx_col = col
            break
    
    if vx_col and vx_col in df.columns:
        ax.plot(time, df[vx_col], 'r-', label='Vx', linewidth=1.5)
        velocity_plotted = True
    
    # look for vy
    vy_col = None
    for col in df.columns:
        col_lower = col.lower()
        if ('vel' in col_lower or 'velocity' in col_lower) and 'y' in col_lower:
            vy_col = col
            break
    
    if vy_col and vy_col in df.columns:
        ax.plot(time, df[vy_col], 'g-', label='Vy', linewidth=1.5)
        velocity_plotted = True
    
    if not velocity_plotted:
        # try to calculate velocity from position
        if len(df) > 1:
            dt = np.diff(time)
            vx = np.diff(df[x_col]) / dt
            vy = np.diff(df[y_col]) / dt
            
            ax.plot(time[1:], vx, 'r-', label='Vx (calc)', linewidth=1.5, alpha=0.7)
            ax.plot(time[1:], vy, 'g-', label='Vy (calc)', linewidth=1.5, alpha=0.7)
            velocity_plotted = True
    
    if velocity_plotted:
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title('Linear Velocity')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.axhline(y=0, color='k', linestyle='-', alpha=0.2, linewidth=0.5)
    else:
        ax.text(0.5, 0.5, 'Velocity data not available', 
                ha='center', va='center', transform=ax.transAxes)
        ax.set_title('Velocity')
    
    # statistics
    ax = axes[1, 1]
    ax.axis('off')
    
    # calculate statistics
    stats_text = []
    
    # distance calculation
    if len(df) > 1:
        dx = np.diff(df[x_col])
        dy = np.diff(df[y_col])
        distances = np.sqrt(dx**2 + dy**2)
        total_distance = np.sum(distances)
        stats_text.append(f"Total distance: {total_distance:.2f} m")
        
        # displacement
        displacement = np.sqrt((df[x_col].iloc[-1] - df[x_col].iloc[0])**2 + 
                              (df[y_col].iloc[-1] - df[y_col].iloc[0])**2)
        stats_text.append(f"Displacement: {displacement:.2f} m")
        
        # efficiency
        if total_distance > 0:
            efficiency = (displacement / total_distance) * 100
            stats_text.append(f"Path efficiency: {efficiency:.1f}%")
    
    # time duration
    if df.columns[0] == 'timestamp' or 'time' in df.columns[0].lower():
        duration = df.iloc[-1, 0] - df.iloc[0, 0]
        stats_text.append(f"Duration: {duration:.1f} s")
    
    # average speed
    if 'duration' in locals() and duration > 0 and 'total_distance' in locals():
        avg_speed = total_distance / duration
        stats_text.append(f"Average speed: {avg_speed:.2f} m/s")
    
    # bounding box
    x_min, x_max = df[x_col].min(), df[x_col].max()
    y_min, y_max = df[y_col].min(), df[y_col].max()
    stats_text.append(f"X range: {x_min:.2f} to {x_max:.2f} m")
    stats_text.append(f"Y range: {y_min:.2f} to {y_max:.2f} m")
    
    # display statistics
    stats_text = "\n".join(stats_text)
    ax.text(0.1, 0.9, stats_text, fontfamily='monospace', fontsize=10,
            verticalalignment='top', transform=ax.transAxes)
    ax.set_title('Path Statistics')
    
    plt.tight_layout()
    
    # save figure
    output_path = os.path.join(output_dir, f'{robot_name}_basic_analysis.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Basic analysis saved: {output_path}")
    
    plt.show()
    return output_path

def plot_wheel_data(df, output_dir, robot_name):
    print("Looking for wheel/joint data...")
    
    # find wheel-related columns
    wheel_cols = []
    for col in df.columns:
        col_lower = col.lower()
        if 'wheel' in col_lower or 'joint' in col_lower:
            wheel_cols.append(col)
    
    if not wheel_cols:
        print("No wheel/joint data found")
        return None
    
    # create figure
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle(f'Wheel Analysis - {robot_name}', fontsize=16)
    
    # time axis
    if df.columns[0] == 'timestamp' or 'time' in df.columns[0].lower():
        time = df.iloc[:, 0] - df.iloc[0, 0]
    else:
        time = np.arange(len(df)) / 20.0
    
    # separate velocity and position columns
    vel_cols = [c for c in wheel_cols if 'vel' in c.lower()]
    pos_cols = [c for c in wheel_cols if 'pos' in c.lower() and 'vel' not in c.lower()]
    
    # wheel velocities
    ax = axes[0, 0]
    colors = ['red', 'blue', 'green', 'purple', 'orange', 'brown']
    
    for i, col in enumerate(vel_cols):
        if i < len(colors):
            ax.plot(time, df[col], color=colors[i], linewidth=1.5, label=col, alpha=0.8)
    
    if vel_cols:
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angular Velocity (rad/s)')
        ax.set_title('Wheel Velocities')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=8)
        ax.axhline(y=0, color='k', linestyle='-', alpha=0.2, linewidth=0.5)
    else:
        ax.text(0.5, 0.5, 'No velocity data', 
                ha='center', va='center', transform=ax.transAxes)
        ax.set_title('Wheel Velocities')
    
    # wheel positions
    ax = axes[0, 1]
    colors = ['red', 'blue', 'green', 'purple', 'orange', 'brown']
    
    for i, col in enumerate(pos_cols):
        if i < len(colors):
            ax.plot(time, df[col], color=colors[i], linewidth=1.5, label=col, alpha=0.8)
    
    if pos_cols:
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (rad)')
        ax.set_title('Wheel Positions')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=8)
    else:
        ax.text(0.5, 0.5, 'No position data', 
                ha='center', va='center', transform=ax.transAxes)
        ax.set_title('Wheel Positions')
    
    # velocity histogram
    ax = axes[1, 0]
    if vel_cols:
        # combine all velocities
        all_vel = []
        for col in vel_cols:
            all_vel.extend(df[col].dropna().values)
        
        if all_vel:
            ax.hist(all_vel, bins=30, edgecolor='black', alpha=0.7)
            ax.set_xlabel('Velocity (rad/s)')
            ax.set_ylabel('Frequency')
            ax.set_title('Velocity Distribution')
            ax.grid(True, alpha=0.3)
            
            # add statistics
            mean_vel = np.mean(all_vel)
            std_vel = np.std(all_vel)
            ax.axvline(mean_vel, color='red', linestyle='--', 
                      label=f'Mean: {mean_vel:.2f}')
            ax.legend()
    else:
        ax.text(0.5, 0.5, 'No velocity data', 
                ha='center', va='center', transform=ax.transAxes)
        ax.set_title('Velocity Distribution')
    
    # statistics
    ax = axes[1, 1]
    ax.axis('off')
    
    stats_text = []
    
    # wheel velocity statistics
    if vel_cols:
        stats_text.append("WHEEL VELOCITIES")
        for col in vel_cols:
            mean_val = df[col].mean()
            std_val = df[col].std()
            max_val = df[col].max()
            min_val = df[col].min()
            stats_text.append(f"{col}:")
            stats_text.append(f"  Mean: {mean_val:.3f} ± {std_val:.3f}")
            stats_text.append(f"  Range: [{min_val:.3f}, {max_val:.3f}]")
            stats_text.append("")
    
    # wheel position statistics
    if pos_cols:
        stats_text.append("WHEEL POSITIONS")
        for col in pos_cols:
            total_rot = df[col].iloc[-1] - df[col].iloc[0]
            revolutions = total_rot / (2 * math.pi)
            stats_text.append(f"{col}:")
            stats_text.append(f"Total rotation: {total_rot:.2f} rad")
            stats_text.append(f"Revolutions: {revolutions:.2f}")
            stats_text.append("")
    
    if stats_text:
        ax.text(0.05, 0.95, '\n'.join(stats_text), 
                fontfamily='monospace', fontsize=8,
                verticalalignment='top', transform=ax.transAxes)
        ax.set_title('Wheel Statistics')
    else:
        ax.text(0.5, 0.5, 'No wheel statistics', 
                ha='center', va='center', transform=ax.transAxes)
        ax.set_title('Wheel Statistics')
    
    plt.tight_layout()
    
    # save figure
    output_path = os.path.join(output_dir, f'{robot_name}_wheel_analysis.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Wheel analysis saved: {output_path}")
    
    plt.show()
    return output_path

def plot_3d_trajectory(df, output_dir, robot_name):
    print("Creating 3D trajectory plot...")
    
    # find position columns
    x_col = y_col = z_col = None
    
    for col in df.columns:
        col_lower = col.lower()
        if 'x' in col_lower and ('pos' in col_lower or col_lower == 'x'):
            x_col = col
        elif 'y' in col_lower and ('pos' in col_lower or col_lower == 'y'):
            y_col = col
        elif 'z' in col_lower and ('pos' in col_lower or col_lower == 'z'):
            z_col = col
    
    if x_col is None or y_col is None or z_col is None:
        print("3D position data not found")
        return None
    
    # create 3D plot
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # plot trajectory
    ax.plot(df[x_col], df[y_col], df[z_col], 'b-', alpha=0.6, linewidth=1.5)
    
    # start and end points
    ax.scatter(df[x_col].iloc[0], df[y_col].iloc[0], df[z_col].iloc[0], 
               color='green', s=100, marker='o', label='Start')
    ax.scatter(df[x_col].iloc[-1], df[y_col].iloc[-1], df[z_col].iloc[-1], 
               color='red', s=100, marker='s', label='End')
    
    # labels
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title(f'3D Trajectory - {robot_name}')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # set equal aspect ratio
    max_range = np.array([df[x_col].max()-df[x_col].min(), 
                          df[y_col].max()-df[y_col].min(), 
                          df[z_col].max()-df[z_col].min()]).max() / 2.0
    
    mid_x = (df[x_col].max()+df[x_col].min()) * 0.5
    mid_y = (df[y_col].max()+df[y_col].min()) * 0.5
    mid_z = (df[z_col].max()+df[z_col].min()) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    # set view angle
    ax.view_init(elev=30, azim=45)
    
    plt.tight_layout()
    
    # save figure
    output_path = os.path.join(output_dir, f'{robot_name}_3d_trajectory.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"3D trajectory saved: {output_path}")
    
    plt.show()
    return output_path

def main():
    parser = argparse.ArgumentParser(
        description='Visualize robot trajectory from CSV files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s robot_data.csv --robot_name robot_1
  %(prog)s odometry.csv --output_dir ./plots
  %(prog)s joints.csv --robot_name soccer_robot
        """
    )
    
    parser.add_argument('csv_file', help='Input CSV file path')
    parser.add_argument('--robot_name', default='robot', help='Name of the robot')
    parser.add_argument('--output_dir', default='./visualizations', help='Output directory')
    
    args = parser.parse_args()
    
    # check if input file exists
    if not os.path.exists(args.csv_file):
        print(f"ERROR: File not found: {args.csv_file}")
        sys.exit(1)
    
    # create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    print("=" * 60)
    print(f"ROBOT DATA VISUALIZATION")
    print("=" * 60)
    print(f"Input file: {args.csv_file}")
    print(f"Robot name: {args.robot_name}")
    print(f"Output directory: {args.output_dir}")
    print("=" * 60)
    
    # load data
    df = load_data(args.csv_file)
    if df is None:
        sys.exit(1)
    
    # create visualizations
    print("\nGenerating visualizations...")
    
    # trajectory analysis
    basic_plot = plot_trajectory(df, args.output_dir, args.robot_name)
    
    # wheel data analysis
    wheel_plot = plot_wheel_data(df, args.output_dir, args.robot_name)
    
    # 3D trajectory (if Z data exists)
    plot_3d = plot_3d_trajectory(df, args.output_dir, args.robot_name)
    
    print("\n" + "=" * 60)
    print("Visualization complete!")
    print(f"Output saved to: {args.output_dir}")
    print("=" * 60)
    
    # create summary file
    summary_file = os.path.join(args.output_dir, 'visualization_summary.txt')
    with open(summary_file, 'w') as f:
        f.write(f"Robot Data Visualization Summary\n")
        f.write(f"=" * 50 + "\n")
        f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Robot: {args.robot_name}\n")
        f.write(f"Input file: {args.csv_file}\n")
        f.write(f"Data points: {len(df)}\n")
        f.write(f"\nGenerated plots:\n")
        if basic_plot:
            f.write(f"  - Basic analysis: {os.path.basename(basic_plot)}\n")
        if wheel_plot:
            f.write(f"  - Wheel analysis: {os.path.basename(wheel_plot)}\n")
        if plot_3d:
            f.write(f"  - 3D trajectory: {os.path.basename(plot_3d)}\n")
    
    print(f"Summary saved: {summary_file}")

if __name__ == '__main__':
    main()