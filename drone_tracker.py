#!/usr/bin/env python3
"""
Drone Trajectory Tracker
Subscribes to drone odometry and plots real-time 3D trajectory
Works with PX4 SITL via ROS2
"""

import sys
import os
import subprocess

# ── Verify the workspace is properly sourced ─────────────────
# Injecting only site-packages is not enough — the ROS 2 type support
# .so files also need to be on LD_LIBRARY_PATH, which only happens via
# sourcing setup.bash. We detect this early and bail with a clear message.
_WS_CANDIDATES = [
    os.path.expanduser('~/px4_msgs_ws/install/setup.bash'),
    os.path.expanduser('~/colcon_ws/install/setup.bash'),
]

def _ld_has_px4_msgs():
    """Check if LD_LIBRARY_PATH contains a px4_msgs lib directory."""
    ld = os.environ.get('LD_LIBRARY_PATH', '')
    return 'px4_msgs' in ld

if not _ld_has_px4_msgs():
    print("\n[ERROR] px4_msgs type support libraries not found in LD_LIBRARY_PATH.")
    print("You must source your workspace before running this script:\n")
    for ws in _WS_CANDIDATES:
        if os.path.isfile(ws):
            print(f"  source {ws}")
    print("  python3 drone_tracker.py\n")
    print("The launch script sources this automatically — use it instead of")
    print("running drone_tracker.py directly.")
    sys.exit(1)

try:
    from px4_msgs.msg import VehicleOdometry
except ModuleNotFoundError:
    print("\n[ERROR] px4_msgs Python package not found even though LD_LIBRARY_PATH looks correct.")
    print("Try re-sourcing your workspace and running again.")
    sys.exit(1)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from collections import deque
import threading
from datetime import datetime
import json
import os
import subprocess
import time

# ── QoS matching PX4 uORB DDS bridge ────────────────────────
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

class DroneTrajectoryTracker(Node):
    def __init__(self):
        super().__init__('drone_trajectory_tracker')
        
        # Trajectory storage
        self.trajectory = {
            'x': deque(maxlen=10000),
            'y': deque(maxlen=10000),
            'z': deque(maxlen=10000),
            'time': deque(maxlen=10000)
        }
        
        self.start_time = None
        self.lock = threading.Lock()
        self.callback_count = {'odometry': 0}
        
        print("\n[INIT] Connecting to drone topics...\n")
        
        try:
            print("[SUBSCRIBE] Odometry: /fmu/out/vehicle_odometry")
            self.create_subscription(
                VehicleOdometry,
                '/fmu/out/vehicle_odometry',
                self.odometry_callback,
                PX4_QOS
            )
            print("  ✓ Successfully subscribed!\n")
        except Exception as e:
            print(f"  ✗ Failed: {e}\n")
        
        print("\n✓ Drone Trajectory Tracker initialized")
    
    def odometry_callback(self, msg: VehicleOdometry):
        """Callback for VehicleOdometry messages (px4_msgs)"""
        self.callback_count['odometry'] += 1
        
        # msg.timestamp is in microseconds
        if self.start_time is None:
            self.start_time = msg.timestamp / 1e6
            print("\n✓ ODOMETRY CALLBACK TRIGGERED - Receiving odometry data...\n")
            
        elapsed = msg.timestamp / 1e6 - self.start_time
        
        # VehicleOdometry uses position[0/1/2] and velocity[0/1/2]
        vel_x, vel_y, vel_z = msg.velocity[0], msg.velocity[1], msg.velocity[2]
        speed = np.sqrt(vel_x**2 + vel_y**2 + vel_z**2)

        # PX4 uses NED frame: Z is positive downward.
        # Negate Z so the plot shows altitude increasing upward.
        with self.lock:
            self.trajectory['x'].append(msg.position[0])
            self.trajectory['y'].append(msg.position[1])
            self.trajectory['z'].append(-msg.position[2])
            self.trajectory['time'].append(elapsed)
        
        if self.callback_count['odometry'] % 5 == 0:
            movement_indicator = "🔴 MOVING" if speed > 0.1 else "⭕ STATIONARY"
            print(
                f"[{movement_indicator}] Odometry #{self.callback_count['odometry']}: "
                f"Pos=(X:{msg.position[0]:7.3f}, Y:{msg.position[1]:7.3f}, Z:{-msg.position[2]:7.3f}) "
                f"Speed:{speed:.3f} m/s"
            )
    
    def save_trajectory(self, filename='drone_trajectory.json'):
        """Save trajectory to JSON file"""
        with self.lock:
            data = {
                'x': list(self.trajectory['x']),
                'y': list(self.trajectory['y']),
                'z': list(self.trajectory['z']),
                'time': list(self.trajectory['time']),
                'timestamp': datetime.now().isoformat()
            }
        
        filepath = os.path.expanduser(f'~/{filename}')
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        
        self.get_logger().info(f'Trajectory saved to {filepath}')
    
    def get_trajectory_copy(self):
        """Get a copy of current trajectory"""
        with self.lock:
            return {
                'x': list(self.trajectory['x']),
                'y': list(self.trajectory['y']),
                'z': list(self.trajectory['z']),
                'time': list(self.trajectory['time'])
            }
    
    def get_callback_status(self):
        """Get callback status for diagnostics"""
        return {
            'odometry': self.callback_count['odometry'],
            'trajectory_points': len(self.trajectory['x'])
        }
    

def plot_trajectory_3d(tracker_node):
    """Plot 3D trajectory in real-time"""
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.set_xlabel('X-axis (m)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y-axis (m)', fontsize=12, fontweight='bold')
    ax.set_zlabel('Z-axis (m)', fontsize=12, fontweight='bold')
    ax.set_title('Drone Trajectory Tracking - REAL-TIME (0 points)', fontsize=14, fontweight='bold')
    
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_zlim(0, 10)
    
    line, = ax.plot([], [], [], 'b-', linewidth=2.5, label='Trajectory')
    scatter = ax.scatter([], [], [], c='r', s=200, marker='o', label='Current Position', depthshade=False)
    
    ax.legend(loc='upper left', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    last_point_count = [0]
    update_count = [0]
    
    def update_plot(frame):
        traj = tracker_node.get_trajectory_copy()
        status = tracker_node.get_callback_status()
        update_count[0] += 1
        
        if update_count[0] % 10 == 0:
            print(f"[PLOT] Update #{update_count[0]}: {len(traj['x'])} points | "
                  f"Odometry CB: {status['odometry']}")
        
        if len(traj['x']) > 0:
            x_data = list(traj['x'])
            y_data = list(traj['y'])
            z_data = list(traj['z'])
            
            line.set_data(x_data, y_data)
            line.set_3d_properties(z_data)
            
            scatter.set_offsets([[x_data[-1], y_data[-1]]])
            scatter.set_3d_properties(zs=[z_data[-1]], zdir='z')
            
            if len(x_data) > 1:
                xs, ys, zs = np.array(x_data), np.array(y_data), np.array(z_data)
                
                x_min, x_max = float(xs.min()), float(xs.max())
                y_min, y_max = float(ys.min()), float(ys.max())
                z_min, z_max = float(zs.min()), float(zs.max())
                
                x_margin = max((x_max - x_min) * 0.25, 1.5)
                y_margin = max((y_max - y_min) * 0.25, 1.5)
                z_margin = max((z_max - z_min) * 0.25, 1.0)
                
                ax.set_xlim(x_min - x_margin, x_max + x_margin)
                ax.set_ylim(y_min - y_margin, y_max + y_margin)
                ax.set_zlim(max(z_min - z_margin, 0), z_max + z_margin)
            
            current_count = len(x_data)
            if current_count != last_point_count[0]:
                ax.set_title(
                    f'Drone Trajectory Tracking - REAL-TIME ({current_count} points)',
                    fontsize=14, fontweight='bold'
                )
                last_point_count[0] = current_count
        
        return line, scatter
    
    anim = FuncAnimation(fig, update_plot, interval=30, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


def plot_trajectory_static(filename='drone_trajectory.json'):
    """Plot trajectory from saved JSON file"""
    filepath = os.path.expanduser(f'~/{filename}')
    
    if not os.path.exists(filepath):
        print(f"Trajectory file not found: {filepath}")
        return
    
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(data['x'], data['y'], data['z'], 'b-', linewidth=2, label='Trajectory')
    
    ax.scatter(data['x'][0], data['y'][0], data['z'][0], 
              c='g', s=200, marker='o', label='Start', edgecolors='black', linewidth=2)
    
    ax.scatter(data['x'][-1], data['y'][-1], data['z'][-1], 
              c='r', s=200, marker='s', label='End', edgecolors='black', linewidth=2)
    
    ax.set_xlabel('X-axis (m)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y-axis (m)', fontsize=12, fontweight='bold')
    ax.set_zlabel('Z-axis (m)', fontsize=12, fontweight='bold')
    ax.set_title('Drone Trajectory - 3D Plot', fontsize=14, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    
    output_file = filepath.replace('.json', '_plot.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Plot saved to {output_file}")
    
    plt.show()


def main():
    """Main function"""
    rclpy.init()
    
    print("\n" + "="*70)
    print("🚁 DRONE TRAJECTORY TRACKER")
    print("="*70)
    print("\nOptions:")
    print("  1. View REAL-TIME 3D trajectory (shows live movement)")
    print("  2. Wait and save trajectory (then exit)")
    print("  3. Load and plot saved trajectory")
    print("  4. DIAGNOSTIC - Check if topics are publishing")
    print("\nEnter choice (1/2/3/4): ", end='', flush=True)
    
    choice = input().strip()
    
    if choice == '4':
        print("\n" + "="*70)
        print("DIAGNOSTIC MODE - Checking ROS2 Topics")
        print("="*70)
        
        try:
            result = subprocess.run(['ros2', 'topic', 'list', '-t'], capture_output=True, text=True, timeout=5)
            if result.stdout:
                print("All active topics:")
                for line in result.stdout.strip().split('\n'):
                    print(f"  {line}")
            else:
                print("No topics found")
        except Exception as e:
            print(f"Could not list topics: {e}")
        
        print("\nTrying to listen to /fmu/out/vehicle_odometry for 5 seconds...")
        try:
            result = subprocess.run(
                ['timeout', '5', 'ros2', 'topic', 'echo', '/fmu/out/vehicle_odometry', '--once'],
                capture_output=True, text=True
            )
            if result.stdout:
                print("✓ Data received:")
                print(result.stdout[:500])
            else:
                print("✗ No data received (timeout or no publisher)")
        except Exception as e:
            print(f"Error: {e}")
        
        rclpy.shutdown()
        return
    
    tracker = DroneTrajectoryTracker()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(tracker,), daemon=True)
    spin_thread.start()
    
    try:
        if choice == '1':
            print("\n" + "="*70)
            print("Starting real-time trajectory visualization...")
            print("="*70)
            print("Subscribed to:")
            print("  • /fmu/out/vehicle_odometry  (px4_msgs/VehicleOdometry)")
            print("\nClose the plot window to stop tracking.\n")
            plot_trajectory_3d(tracker)
            
        elif choice == '2':
            print("\n" + "="*70)
            print("Tracking drone trajectory...")
            print("="*70)
            print("Subscribed to:")
            print("  • /fmu/out/vehicle_odometry  (px4_msgs/VehicleOdometry)")
            print("\nPress Ctrl+C to stop and save the trajectory.\n")
            try:
                while True:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n\nStopping tracker...")
                tracker.save_trajectory()
                print("Trajectory saved!")
                
        elif choice == '3':
            print("\nLoading and plotting saved trajectory...")
            plot_trajectory_static()
        
        else:
            print("Invalid choice!")
    
    finally:
        rclpy.shutdown()
        print("Tracker node shut down.")


if __name__ == '__main__':
    main()