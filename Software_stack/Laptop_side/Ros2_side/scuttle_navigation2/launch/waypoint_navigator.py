#!/usr/bin/env python3
"""
ROS2 Waypoint Navigator for Nav2 - FIXED VERSION
- Loads waypoints from ~/ws/config/waypoints.yaml
- Sends navigation goals to Nav2's NavigateToPose action
- NO BLOCKING DELAYS - processes goals immediately
- Proper async handling for fast navigation
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
import yaml
import os
import math

WAYPOINTS_FILE = os.path.expanduser('~/ws/config/waypoints.yaml')


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        self.declare_parameter('waypoints_file', WAYPOINTS_FILE)
        self.waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info(f"Loading waypoints from: {self.waypoints_file}")
        
        self.waypoints = self.load_waypoints()
        self.current_waypoint_idx = 0
        self.is_navigating = False
        self.goal_handle = None
        
        # Wait for action server
        self.get_logger().info("Waiting for NavigateToPose action server...")
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Navigation action server not available!")
            raise RuntimeError("Nav2 action server not found")
        
        self.get_logger().info("✓ Connected to Nav2 action server")
        
        # Start navigation immediately (no timer needed)
        if len(self.waypoints) > 0:
            self.send_next_goal()
        else:
            self.get_logger().error("No waypoints loaded!")
        
    def load_waypoints(self):
        """Load waypoints from YAML file"""
        try:
            with open(self.waypoints_file, 'r') as f:
                data = yaml.safe_load(f)
            waypoints = data.get('waypoints', [])
            self.get_logger().info(f"✓ Loaded {len(waypoints)} waypoints")
            return waypoints
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            return []
    
    def create_pose(self, wp):
        """Create PoseStamped from waypoint dict"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(wp.get('x', 0.0))
        pose.pose.position.y = float(wp.get('y', 0.0))
        pose.pose.position.z = float(wp.get('z', 0.0))
        
        # Convert yaw to quaternion
        yaw = float(wp.get('yaw', 0.0))
        q = self.yaw_to_quaternion(yaw)
        pose.pose.orientation = q
        
        return pose
    
    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q
    
    def send_next_goal(self):
        """Send the next waypoint goal"""
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info("=" * 60)
            self.get_logger().info("✓ ALL WAYPOINTS COMPLETED!")
            self.get_logger().info("=" * 60)
            return
        
        wp = self.waypoints[self.current_waypoint_idx]
        pose = self.create_pose(wp)
        name = wp.get('name', f'WP{self.current_waypoint_idx + 1}')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.get_logger().info(
            f"\n[{self.current_waypoint_idx + 1}/{len(self.waypoints)}] "
            f"Navigating to '{name}' at ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
        )
        
        # Send goal asynchronously
        self.is_navigating = True
        self.current_goal_name = name
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback (optional - for monitoring progress)"""
        # You can log distance remaining, ETA, etc. here if needed
        pass
    
    def goal_response_callback(self, future):
        """Handle goal response"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error(f"✗ Goal '{self.current_goal_name}' was REJECTED!")
            self.is_navigating = False
            # Try next waypoint anyway
            self.current_waypoint_idx += 1
            self.send_next_goal()
            return
        
        self.get_logger().info(f"✓ Goal '{self.current_goal_name}' accepted")
        
        # Get result asynchronously
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Handle goal result and immediately send next goal"""
        result = future.result()
        
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f"✓ ARRIVED at '{self.current_goal_name}'!")
        elif result.status == 5:  # CANCELED
            self.get_logger().warn(f"⚠ Goal '{self.current_goal_name}' was CANCELED")
        elif result.status == 6:  # ABORTED
            self.get_logger().error(f"✗ Goal '{self.current_goal_name}' ABORTED")
        else:
            self.get_logger().warn(f"⚠ Goal '{self.current_goal_name}' ended with status {result.status}")
        
        self.is_navigating = False
        self.current_waypoint_idx += 1
        
        # IMMEDIATELY send next goal (no delay!)
        self.send_next_goal()


def main():
    rclpy.init()
    
    try:
        navigator = WaypointNavigator()
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        print('\n✗ Navigation interrupted by user')
    except Exception as e:
        print(f'\n✗ Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()