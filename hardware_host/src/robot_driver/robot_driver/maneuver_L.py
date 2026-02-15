#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

class TrajectoryMaster(Node):
    def __init__(self):
        super().__init__('trajectory_master')
        
        # Waypoints: (x, y)
        self.waypoints = [
            (3.0, 0.0),
            (3.0, -2.0),
            (3.0, 0.0),
            (0.0, 0.0)
        ]
        
        self.current_wp_index = 0
        self.goal_x, self.goal_y = self.waypoints[0]
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.is_waiting = False
        self.resume_timer = None

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        
        if not self.is_waiting:
            self.navigate()

    def navigate(self):
        msg = Twist()
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)

        if dist < 0.08:
            self.handle_waypoint_reached()
            return
            
        angle_diff = angle_to_goal - self.theta
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi # Normalize to [-pi, pi]
            
        # Movement logic: Stop and Rotate, then Drive
        if abs(angle_diff) > 0.15:
            msg.angular.z = 0.4 if angle_diff > 0 else -0.4
        else:
            msg.linear.x = 0.25
            msg.angular.z = 0.5 * angle_diff # P-controller for heading
            
        self.cmd_vel_pub.publish(msg)

    def handle_waypoint_reached(self):
        self.stop_robot()
        self.get_logger().info(f"Reached: {self.waypoints[self.current_wp_index]}")
        
        self.current_wp_index += 1
        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().info("Route Complete.")
            raise SystemExit
            
        self.goal_x, self.goal_y = self.waypoints[self.current_wp_index]
        self.is_waiting = True
        self.resume_timer = self.create_timer(2.0, self.resume_navigation)
        
    def resume_navigation(self):
        self.is_waiting = False
        if self.resume_timer:
            self.resume_timer.cancel()
            self.resume_timer = None

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(TrajectoryMaster())
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
