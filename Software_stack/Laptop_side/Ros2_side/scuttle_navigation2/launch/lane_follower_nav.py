#!/usr/bin/env python3
"""
Lane Follower + Navigation for SCUTTLE
1. Follows predefined path from spawn into warehouse (lane following)
2. Once inside, switches to Nav2 navigation for waypoints
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
import math
import yaml
import os
import time

def quaternion_from_euler(roll, pitch, yaw):
    """Convert euler angles to quaternion"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return [x, y, z, w]

class LaneFollowerNavigator(Node):
    def __init__(self):
        super().__init__('lane_follower_navigator')
        
        # Publishers and Action Clients
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Waypoints file
        self.waypoints_file = os.path.join(os.path.expanduser('~'), 'ws', 'config', 'waypoints.yaml')
        self.waypoints = []
        
        # Predefined lane path: (3.1, -0.4) → west in 0.3m steps → (0.4, -0.4) → right turn north → (0.4, 0.4)
        self.lane_path = [
            {'x': 3.1, 'y': -0.4, 'duration': 0, 'action': 'start'},           # Start (spawn point)
            {'x': 2.8, 'y': -0.4, 'duration': 2.0, 'action': 'forward'},       # Step 1
            {'x': 2.5, 'y': -0.4, 'duration': 2.0, 'action': 'forward'},       # Step 2
            {'x': 2.2, 'y': -0.4, 'duration': 2.0, 'action': 'forward'},       # Step 3
            {'x': 1.9, 'y': -0.4, 'duration': 2.0, 'action': 'forward'},       # Step 4
            {'x': 1.6, 'y': -0.4, 'duration': 2.0, 'action': 'forward'},       # Step 5
            {'x': 1.3, 'y': -0.4, 'duration': 2.0, 'action': 'forward'},       # Step 6
            {'x': 1.0, 'y': -0.4, 'duration': 2.0, 'action': 'forward'},       # Step 7
            {'x': 0.7, 'y': -0.4, 'duration': 2.0, 'action': 'forward'},       # Step 8
            {'x': 0.4, 'y': -0.4, 'duration': 2.0, 'action': 'forward'},       # Reach (0.4, -0.4)
            {'x': 0.4, 'y': -0.4, 'duration': 3.14, 'action': 'rotate_right'}, # Right turn (90°)
            {'x': 0.4, 'y': -0.1, 'duration': 2.0, 'action': 'forward'},       # Enter warehouse
            {'x': 0.4, 'y': 0.1, 'duration': 1.5, 'action': 'forward'},        # Continue north
            {'x': 0.4, 'y': 0.4, 'duration': 2.0, 'action': 'forward'},        # Final position (0.4, 0.4)
        ]
        
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("LANE FOLLOWER + NAV2 NAVIGATOR")
        self.get_logger().info("="*70)
        self.get_logger().info("Phase 1: Lane following into warehouse")
        self.get_logger().info("Phase 2: Nav2 navigation to waypoints")
        self.get_logger().info("="*70 + "\n")
    
    def move_straight(self, duration, speed=0.2):
        """Move forward for specified duration"""
        msg = Twist()
        msg.linear.x = speed
        
        end_time = time.time() + duration
        rate = self.create_rate(10)
        
        while time.time() < end_time and rclpy.ok():
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop()
    
    def rotate_90_right(self, duration=3.14):
        """Rotate 90 degrees to the right (clockwise)"""
        msg = Twist()
        msg.angular.z = -0.5  # Negative for clockwise rotation
        
        end_time = time.time() + duration
        rate = self.create_rate(10)
        
        while time.time() < end_time and rclpy.ok():
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop()
    
    def stop(self):
        """Stop the robot"""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        time.sleep(0.2)
    
    def follow_lane_into_warehouse(self):
        """Execute predefined lane path to enter warehouse"""
        self.get_logger().info("\n=== PHASE 1: Lane Following ===")
        self.get_logger().info("Path: (3.1, -0.4) → west → (0.4, -0.4) → right turn → (0.4, 0.4)")
        
        for i, point in enumerate(self.lane_path):
            if i == 0:
                self.get_logger().info(f"Starting at ({point['x']:.2f}, {point['y']:.2f})")
                continue  # Skip start point
            
            action = point.get('action', 'forward')
            
            if action == 'rotate_right':
                self.get_logger().info(f"Segment {i}/{len(self.lane_path)-1}: Turning RIGHT 90°")
                self.rotate_90_right(point['duration'])
            else:
                self.get_logger().info(f"Segment {i}/{len(self.lane_path)-1}: Moving to ({point['x']:.2f}, {point['y']:.2f})")
                self.move_straight(point['duration'], speed=0.15)
            
            time.sleep(0.5)  # Brief pause between segments
        
        self.get_logger().info("✓ Reached warehouse position (0.4, 0.4)!")
        self.get_logger().info("Waiting 2 seconds before Nav2 navigation...\n")
        time.sleep(2.0)
    
    def load_waypoints(self):
        """Load waypoints from YAML file"""
        if not os.path.exists(self.waypoints_file):
            self.get_logger().error(f"Waypoints file not found: {self.waypoints_file}")
            return False
        
        with open(self.waypoints_file, 'r') as f:
            data = yaml.safe_load(f)
            if data and 'waypoints' in data:
                self.waypoints = data['waypoints']
                self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
                return True
        
        return False
    
    def is_goal_in_mapped_area(self, waypoint):
        """Check if goal is within mapped warehouse area"""
        x = waypoint['x']
        y = waypoint['y']
        
        # Warehouse mapped area boundaries (adjust based on your map)
        # X: -0.03 to 3.12, Y: 0.0 to 4.14 (interior only)
        x_min, x_max = -0.1, 3.2
        y_min, y_max = 0.0, 4.2
        
        if x_min <= x <= x_max and y_min <= y <= y_max:
            return True
        else:
            self.get_logger().warn(f"Goal ({x:.2f}, {y:.2f}) is OUTSIDE mapped area!")
            self.get_logger().warn(f"  Valid range: X=[{x_min}, {x_max}], Y=[{y_min}, {y_max}]")
            return False
    
    def send_nav_goal(self, waypoint):
        """Send navigation goal to Nav2"""
        # Check if goal is in mapped area
        if not self.is_goal_in_mapped_area(waypoint):
            self.get_logger().error(f"Skipping {waypoint['name']} - outside mapped area")
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(waypoint['x'])
        goal_msg.pose.pose.position.y = float(waypoint['y'])
        goal_msg.pose.pose.position.z = float(waypoint['z'])
        
        # Convert yaw to quaternion
        quat = quaternion_from_euler(0, 0, waypoint['yaw'])
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]
        
        self.get_logger().info(f"Sending goal: {waypoint['name']} at ({waypoint['x']:.2f}, {waypoint['y']:.2f})")
        
        # Wait for action server
        self._action_client.wait_for_server()
        
        # Send goal
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected for {waypoint['name']}")
            return False
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        status = result_future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f"✓ Reached {waypoint['name']}")
            return True
        else:
            self.get_logger().warn(f"✗ Failed to reach {waypoint['name']} (status: {status})")
            return False
    
    def navigate_waypoints(self):
        """Navigate through all waypoints using Nav2"""
        if not self.load_waypoints():
            self.get_logger().error("No waypoints to navigate!")
            return
        
        self.get_logger().info("\n=== PHASE 2: Nav2 Waypoint Navigation ===")
        
        success_count = 0
        for i, waypoint in enumerate(self.waypoints):
            self.get_logger().info(f"\nWaypoint {i+1}/{len(self.waypoints)}: {waypoint['name']}")
            
            if self.send_nav_goal(waypoint):
                success_count += 1
                time.sleep(1.0)  # Pause between waypoints
            else:
                user_input = input(f"Failed to reach {waypoint['name']}. Continue? (y/n): ")
                if user_input.lower() != 'y':
                    break
        
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info(f"Navigation Complete: {success_count}/{len(self.waypoints)} waypoints reached")
        self.get_logger().info("="*70)
    
    def execute(self):
        """Execute complete mission: lane following + navigation"""
        try:
            # Phase 1: Lane following into warehouse
            self.follow_lane_into_warehouse()
            
            # Phase 2: Nav2 navigation through waypoints
            self.navigate_waypoints()
            
        except KeyboardInterrupt:
            self.get_logger().info("\n\nMission interrupted by user")
            self.stop()

def main():
    rclpy.init()
    
    navigator = LaneFollowerNavigator()
    
    print("\n" + "="*70)
    print("LANE FOLLOWER + NAV2 NAVIGATOR")
    print("="*70)
    print("This will:")
    print("  1. Follow predefined path from spawn into warehouse")
    print("  2. Switch to Nav2 navigation for waypoints inside")
    print("\nMake sure:")
    print("  - Gazebo, ROS bridge, Nav2 are all running")
    print("  - Initial pose has been set in RViz2")
    print("  - Robot is at spawn position (3.1, -0.4)")
    print("="*70)
    
    input("\nPress ENTER to start mission...")
    
    try:
        navigator.execute()
    except Exception as e:
        navigator.get_logger().error(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
