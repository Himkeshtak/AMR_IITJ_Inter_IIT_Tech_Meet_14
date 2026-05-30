#!/usr/bin/env python3
"""
Geotag Recorder for ROS2 - Records waypoints during SLAM mapping
Press SPACE to record current robot position
Saves to ~/ws/config/waypoints.yaml
"""

import rclpy
from rclpy.node import Node
import yaml
import os
import sys
import termios
import tty
import select
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math

class GeotagRecorder(Node):
    def __init__(self):
        super().__init__('geotag_recorder')
        
        # Waypoints file path
        self.waypoints_file = os.path.expanduser('~/ws/config/waypoints.yaml')
        self.waypoints = []
        self.max_waypoints = 10
        self.current_pose = None
        
        # Create config directory
        config_dir = os.path.dirname(self.waypoints_file)
        if not os.path.exists(config_dir):
            os.makedirs(config_dir)
            self.get_logger().info(f"✓ Created config directory: {config_dir}")
        
        # Subscribe to AMCL pose (from ros1_bridge or gmapping)
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        # Load existing waypoints if any
        self.load_waypoints()
        
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("GEOTAG RECORDER - Waypoint Recording for SLAM")
        self.get_logger().info("="*70)
        self.get_logger().info(f"Waypoints recorded: {len(self.waypoints)}/{self.max_waypoints}")
        self.get_logger().info("="*70)
        self.get_logger().info("CONTROLS:")
        self.get_logger().info("  SPACE     - Record current position as waypoint")
        self.get_logger().info("  S         - Save waypoints to file")
        self.get_logger().info("  L         - List all recorded waypoints")
        self.get_logger().info("  D         - Delete last waypoint")
        self.get_logger().info("  Q         - Quit (auto-saves)")
        self.get_logger().info("="*70 + "\n")
        
        # Wait for pose data
        self.wait_for_pose_data()
    
    def wait_for_pose_data(self):
        """Wait until AMCL pose data is available"""
        self.get_logger().info("⏳ Waiting for pose data from AMCL/SLAM...")
        self.get_logger().info("   → Make sure SLAM is running on Jetson")
        self.get_logger().info("   → Make sure ros1_bridge is running")
        self.get_logger().info("   → Drive robot around with teleop")
        self.get_logger().info("   → This may take a few seconds...\n")
        
        max_retries = 60  # 60 seconds timeout
        retry_count = 0
        
        while rclpy.ok() and retry_count < max_retries:
            if self.current_pose is not None:
                self.get_logger().info("✓ Pose data received! Ready to record waypoints.\n")
                return True
            
            retry_count += 1
            if retry_count % 5 == 0:
                self.get_logger().info(f"   Waiting ({retry_count}s)... Listening on /amcl_pose")
            rclpy.spin_once(self, timeout_sec=1.0)
        
        if retry_count >= max_retries:
            self.get_logger().error("\n" + "="*70)
            self.get_logger().error("✗ TIMEOUT: No pose data received after 60 seconds")
            self.get_logger().error("="*70)
            self.get_logger().error("TROUBLESHOOTING:")
            self.get_logger().error("1. Is SLAM running on Jetson?")
            self.get_logger().error("2. Is ros1_bridge running? Check: ros2 node list")
            self.get_logger().error("3. Is /amcl_pose topic available? Check: ros2 topic list | grep amcl")
            self.get_logger().error("4. Check ros1_bridge output for errors")
            self.get_logger().error("="*70 + "\n")
            return False
        
        return True
    
    def pose_callback(self, msg):
        """Handle PoseWithCovarianceStamped (from AMCL)"""
        self.current_pose = msg.pose.pose
    
    def get_pose_from_amcl(self):
        """Get current pose from AMCL subscription"""
        if self.current_pose is None:
            return None
        
        # Extract position
        trans = self.current_pose.position
        rot = self.current_pose.orientation
        
        # Calculate yaw from quaternion
        siny_cosp = 2 * (rot.w * rot.z + rot.x * rot.y)
        cosy_cosp = 1 - 2 * (rot.y * rot.y + rot.z * rot.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return {
            'x': trans.x,
            'y': trans.y,
            'z': trans.z,
            'yaw': yaw
        }
    
    def load_waypoints(self):
        """Load existing waypoints from file."""
        if os.path.exists(self.waypoints_file):
            with open(self.waypoints_file, 'r') as f:
                data = yaml.safe_load(f)
                if data and 'waypoints' in data:
                    self.waypoints = data['waypoints']
                    self.get_logger().info(f"✓ Loaded {len(self.waypoints)} existing waypoints")
    
    def save_waypoints(self):
        """Save waypoints to YAML file."""
        data = {
            'waypoints': self.waypoints,
            'total_count': len(self.waypoints),
            'home_position': {
                'x': 3.1,
                'y': -0.4,
                'z': 0.0,
                'yaw': 3.14159
            }
        }
        
        with open(self.waypoints_file, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        
        self.get_logger().info(f"✓ Saved {len(self.waypoints)} waypoints to {self.waypoints_file}")
    
    def record_waypoint(self):
        """Record current position as waypoint."""
        if len(self.waypoints) >= self.max_waypoints:
            self.get_logger().warn(f"✗ Maximum waypoints ({self.max_waypoints}) already recorded!")
            return
        
        # Get pose from AMCL
        pose_data = self.get_pose_from_amcl()
        
        if pose_data is None:
            self.get_logger().warn("✗ No pose data available yet.")
            self.get_logger().warn("   → Make sure you've driven the robot around")
            return
        
        waypoint_id = len(self.waypoints) + 1
        waypoint = {
            'id': waypoint_id,
            'name': f'Point_{waypoint_id}',
            'x': round(pose_data['x'], 3),
            'y': round(pose_data['y'], 3),
            'z': round(pose_data['z'], 3),
            'yaw': round(pose_data['yaw'], 3)
        }
        
        self.waypoints.append(waypoint)
        
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info(f"✓ WAYPOINT {waypoint_id} RECORDED")
        self.get_logger().info("="*70)
        self.get_logger().info(f"  Position: ({pose_data['x']:.3f}, {pose_data['y']:.3f}, {pose_data['z']:.3f})")
        self.get_logger().info(f"  Orientation: {pose_data['yaw']:.3f} rad ({pose_data['yaw']*180/3.14159:.1f}°)")
        self.get_logger().info(f"  Progress: {len(self.waypoints)}/{self.max_waypoints}")
        self.get_logger().info("="*70 + "\n")
        
        # Auto-save after each recording
        self.save_waypoints()
    
    def list_waypoints(self):
        """Display all recorded waypoints."""
        if not self.waypoints:
            self.get_logger().info("No waypoints recorded yet.")
            return
        
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info(f"RECORDED WAYPOINTS ({len(self.waypoints)}/{self.max_waypoints})")
        self.get_logger().info("="*70)
        for wp in self.waypoints:
            self.get_logger().info(f"  {wp['id']}. {wp['name']}: ({wp['x']}, {wp['y']}, {wp['z']}) @ {wp['yaw']:.2f} rad")
        self.get_logger().info("="*70 + "\n")
    
    def delete_last_waypoint(self):
        """Remove the last recorded waypoint."""
        if not self.waypoints:
            self.get_logger().warn("No waypoints to delete.")
            return
        
        deleted = self.waypoints.pop()
        self.save_waypoints()
        self.get_logger().info(f"✓ Deleted waypoint {deleted['id']}: {deleted['name']}")
    
    def get_key(self):
        """Get single keypress without waiting for Enter."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def run(self):
        """Main loop - listen for key presses."""
        self.get_logger().info("✓ Ready! Drive robot and press SPACE to record waypoints...\n")
        
        try:
            while rclpy.ok():
                # Non-blocking key check
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = self.get_key()
                    
                    if key == ' ':  # SPACE
                        self.record_waypoint()
                    elif key.lower() == 's':  # Save
                        self.save_waypoints()
                    elif key.lower() == 'l':  # List
                        self.list_waypoints()
                    elif key.lower() == 'd':  # Delete
                        self.delete_last_waypoint()
                    elif key.lower() == 'q':  # Quit
                        self.get_logger().info("Quitting and saving...")
                        self.save_waypoints()
                        break
                
                # Process ROS callbacks
                rclpy.spin_once(self, timeout_sec=0.01)
        except KeyboardInterrupt:
            self.get_logger().info("\nShutdown requested. Saving waypoints...")
            self.save_waypoints()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        recorder = GeotagRecorder()
        recorder.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
