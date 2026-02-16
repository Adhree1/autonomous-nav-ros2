#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class AutonomousNavigator(Node):
    def __init__(self):
        super().__init__('autonomous_navigator')
        
        # Publisher to control robot movement
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber to get laser scan data (for obstacle detection)
        self.scan_subscriber = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        # Robot state
        self.obstacle_detected = False
        self.min_distance = float('inf')
        
        # Create timer for navigation loop (10 Hz)
        self.timer = self.create_timer(0.1, self.navigate)
        
        self.get_logger().info('Autonomous Navigator Started!')
    
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
    
    def navigate(self):
        """Main navigation logic"""
        vel_msg = Twist()
        
        if self.obstacle_detected:
            # Obstacle ahead - turn right
            self.get_logger().info(f'Obstacle detected at {self.min_distance:.2f}m - Turning')
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = -0.5  # Turn right
        else:
            # No obstacle - move forward
            self.get_logger().info(f'Clear ahead ({self.min_distance:.2f}m) - Moving forward')
            vel_msg.linear.x = 0.2  # Move forward at 0.2 m/s
            vel_msg.angular.z = 0.0
        
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
