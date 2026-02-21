#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class AutonomousNavigator(Node):
    def __init__(self):
        super().__init__('autonomous_navigator')
        
        # Publisher to control robot movement
        self.velocity_publisher = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        
        # Subscriber to get laser scan data (for obstacle detection)
        self.scan_subscriber = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        # Subscriber to get robot position
        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        # Robot state
        self.obstacle_detected = False
        self.min_distance = float('inf')
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Waypoints to visit (x, y coordinates)
        self.waypoints = [
            (2.0, 0.0),   # First goal: 2 meters forward
            (2.0, 2.0),   # Second goal: 2 meters forward, 2 meters left
            (0.0, 2.0),   # Third goal: back to x=0, stay at y=2
            (0.0, 0.0)    # Final goal: return to start
        ]
        self.current_waypoint_index = 0
        self.goal_reached_threshold = 0.3  # Consider goal reached if within 0.3m
        
        # Create timer for navigation loop (10 Hz)
        self.timer = self.create_timer(0.1, self.navigate)
        
        self.get_logger().info('Autonomous Navigator with Waypoints Started!')
        self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')
    
    def odom_callback(self, msg):
        """Update robot's current position from odometry"""
        # Get position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw angle (rotation around z-axis)
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def scan_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        # Check front 60 degrees for obstacles
        front_ranges = msg.ranges[0:30] + msg.ranges[-30:]
        
        # Filter out invalid readings
        valid_ranges = [r for r in front_ranges if r > 0.0 and r < 10.0]
        
        if valid_ranges:
            self.min_distance = min(valid_ranges)
            # Obstacle if something is within 0.5 meters
            self.obstacle_detected = self.min_distance < 0.5
        else:
            self.min_distance = float('inf')
            self.obstacle_detected = False
    
    def get_distance_to_goal(self, goal_x, goal_y):
        """Calculate Euclidean distance to goal"""
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        return math.sqrt(dx**2 + dy**2)
    
    def get_angle_to_goal(self, goal_x, goal_y):
        """Calculate angle to goal relative to current heading"""
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        goal_angle = math.atan2(dy, dx)
        
        # Calculate difference between goal angle and current heading
        angle_diff = goal_angle - self.current_yaw
        
        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        return angle_diff
    
    def navigate(self):
        """Main navigation logic with waypoint following"""
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = 'base_link'
        
        # Check if all waypoints completed
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached! Mission complete!', throttle_duration_sec=2.0)
            vel_msg.twist.linear.x = 0.0
            vel_msg.twist.angular.z = 0.0
            self.velocity_publisher.publish(vel_msg)
            return
        
        # Get current goal
        goal_x, goal_y = self.waypoints[self.current_waypoint_index]
        distance_to_goal = self.get_distance_to_goal(goal_x, goal_y)
        angle_to_goal = self.get_angle_to_goal(goal_x, goal_y)
        
        # Check if current waypoint is reached
        if distance_to_goal < self.goal_reached_threshold:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}!')
            self.current_waypoint_index += 1
            return
        
        # PRIORITY 1: Avoid obstacles (safety first!)
        if self.obstacle_detected:
            self.get_logger().info(f'Obstacle at {self.min_distance:.2f}m - Avoiding', throttle_duration_sec=1.0)
            vel_msg.twist.linear.x = 0.0
            vel_msg.twist.angular.z = -0.5  # Turn right to avoid
        
        # PRIORITY 2: Turn toward goal if facing wrong direction
        elif abs(angle_to_goal) > 0.2:  # If more than ~11 degrees off
            self.get_logger().info(
                f'Turning toward waypoint {self.current_waypoint_index + 1} '
                f'(distance: {distance_to_goal:.2f}m, angle: {math.degrees(angle_to_goal):.1f}°)',
                throttle_duration_sec=1.0
            )
            vel_msg.twist.linear.x = 0.1  # Move slowly while turning
            vel_msg.twist.angular.z = 2.0 * angle_to_goal  # Proportional turning
        
        # PRIORITY 3: Drive toward goal
        else:
            self.get_logger().info(
                f'Driving to waypoint {self.current_waypoint_index + 1} '
                f'(distance: {distance_to_goal:.2f}m)',
                throttle_duration_sec=1.0
            )
            vel_msg.twist.linear.x = 0.3  # Move forward
            vel_msg.twist.angular.z = 0.5 * angle_to_goal  # Minor corrections
        
        self.velocity_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    navigator = AutonomousNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
