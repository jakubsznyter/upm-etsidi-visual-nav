#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

class CircleMaster(Node):
    def __init__(self):
        super().__init__('circle_master')
        
        self.get_logger().info("Start: Operation Donut (Radius 1m)")
        
        # Configuration
        self.target_radius = 1.0  # meters
        self.linear_speed = 0.3   # m/s
        
        # omega = v / r
        self.angular_speed = self.linear_speed / self.target_radius
        
        self.get_logger().info(f"Params: v={self.linear_speed}, w={self.angular_speed}")

        # Communication setup
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # State variables
        self.first_run = True
        self.previous_theta = 0.0
        self.total_angle_turned = 0.0
        self.odom_received = False
        
        # Control loop at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.odom_received = True
        
        # Extract Yaw from quaternion
        rot_q = msg.pose.pose.orientation
        (_, _, current_theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        
        if self.first_run:
            self.previous_theta = current_theta
            self.first_run = False
            return

        delta_theta = current_theta - self.previous_theta
        
        # Handle -pi to pi wrap around
        if delta_theta > math.pi:
            delta_theta -= 2 * math.pi
        elif delta_theta < -math.pi:
            delta_theta += 2 * math.pi
            
        self.total_angle_turned += abs(delta_theta)
        self.previous_theta = current_theta

    def control_loop(self):
        if not self.odom_received or self.first_run:
            return

        msg = Twist()
        
        # Stop slightly early to account for inertia
        if self.total_angle_turned < (2 * math.pi - 0.05):
            msg.linear.x = self.linear_speed
            msg.angular.z = self.angular_speed
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
            self.get_logger().info("Circle complete.")
            raise SystemExit

        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CircleMaster()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
