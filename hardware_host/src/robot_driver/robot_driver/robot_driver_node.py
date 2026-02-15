# This is the main node that integrates all the modules

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import math
from robot_driver.kinematics import Kinematics
from robot_driver.roboclaw_interface import RoboClawInterface
from robot_driver.odometry import RobotOdometry
import tf2_ros
from tf_transformations import quaternion_from_euler


class RobotDriverNode(Node):
    def __init__(self):
        super().__init__('robot_driver_node')
        # Kinematics setup: radius, wheel base, and track width (axle distance)
        self.kinematics = Kinematics(wheel_radius=0.06, wheel_base=0.34, axle_distance=0.48)
        # Interface setup: port and baudrate
        self.roboclaw = RoboClawInterface(front_port="/dev/ttyACM0", rear_port="/dev/ttyACM1", baudrate=115200)
        self.odometry = RobotOdometry()
        
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        self.PPR = 537.7   # Pulses Per Revolution of the encoder
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.target_v = 0.0
        self.target_omega = 0.0
        self.last_cmd_vel_time = self.get_clock().now()

        # Subscribe to cmd_vel and prepare odometry publisher 
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.timer_period = 0.05  # 20Hz
        self.timer = self.create_timer(self.timer_period, self.update_loop)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Check RoboClaw Battery
        voltage_real, battery_percentage = self.roboclaw.ReadVoltage()
        self.get_logger().info(f"Main battery voltage: {voltage_real} V")
        self.get_logger().info(f"Main battery percentage: {battery_percentage} %")

        # Reset Encoders on startup
        self.roboclaw.reset_encoders()

        self.last_odom_time = self.get_clock().now()
    
    def cmd_vel_callback(self, msg):
        """Callback for velocity commands."""
        self.target_v = msg.linear.x
        self.target_omega = msg.angular.z
        self.last_cmd_vel_time = self.get_clock().now()

    def update_loop(self):
        # Safety measure: if no cmd_vel is received within a timeout, stop the robot.
        current_time = self.get_clock().now()
        if (current_time - self.last_cmd_vel_time).nanoseconds / 1e9 > 0.5: # 0.5 second timeout
            self.target_v = 0.0
            self.target_omega = 0.0

        # Command the robot with the stored target velocity
        self.move_robot(self.target_v, self.target_omega)
        
        # Optional: self.roboclaw.check_status()

        # Update and publish odometry constantly
        self.update_and_publish_odometry(current_time)

    def rads_to_pps(self, rads):
        """Converts radians per second (rad/s) to pulses per second (PPS)."""
        return int((rads * self.PPR) / (2 * math.pi))
    
    def pps_to_rads(self, pps):
        """Converts pulses per second (PPS) to radians per second (rad/s)."""
        return (pps * 2 * math.pi) / self.PPR


    def move_robot(self, v, omega):
        """Moves the robot using inverse kinematics and RoboClaw."""
        omega1, omega2, omega3, omega4 = self.kinematics.inverse_kinematics(v, omega)

        # Convert wheel speeds from rad/s to PPS
        front_left_pps = self.rads_to_pps(omega1)
        front_right_pps = self.rads_to_pps(omega2)
        rear_left_pps = self.rads_to_pps(omega3)
        rear_right_pps = self.rads_to_pps(omega4)

        # Send speeds to the RoboClaw controllers
        self.roboclaw.set_wheel_speed(front_left_pps, front_right_pps, rear_left_pps, rear_right_pps)

    def update_and_publish_odometry(self, current_time):
        # Read encoder speeds
        enc_f1, enc_f2, enc_r1, enc_r2 = self.roboclaw.read_encoders_speed()

        # Convert encoder feedback from PPS to rad/s
        omega1_real = self.pps_to_rads(enc_f1)
        omega2_real = self.pps_to_rads(enc_f2)
        omega3_real = self.pps_to_rads(enc_r1)
        omega4_real = self.pps_to_rads(enc_r2)

        # Forward Kinematics: calculate real linear and angular velocity
        v_real_calc, omega_real_calc = self.kinematics.direct_kinematics(omega1_real, omega2_real, omega3_real, omega4_real)
        
        # Calculate dt (time elapsed)
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = current_time

        if dt <= 0: # Avoid division by zero or negative dt during clock jumps
            self.get_logger().warn("dt <= 0, skipping odometry update")
            return

        # Update the odometry tracker
        self.odometry.update(v_real_calc, omega_real_calc, dt)

        # Create and publish TF Transformations (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        # Position
        t.transform.translation.x = self.odometry.x
        t.transform.translation.y = self.odometry.y
        t.transform.translation.z = 0.0  # Planar robot

        # Orientation
        q = quaternion_from_euler(0, 0, self.odometry.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Broadcast the transformation
        self.tf_broadcaster.sendTransform(t)
        
        # Publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Pose and orientation
        odom_msg.pose.pose.position.x = self.odometry.x
        odom_msg.pose.pose.position.y = self.odometry.y
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Velocity in the child frame
        odom_msg.twist.twist.linear.x = v_real_calc
        odom_msg.twist.twist.linear.y = 0.0 # No lateral movement
        odom_msg.twist.twist.angular.z = omega_real_calc

        self.odom_pub.publish(odom_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = RobotDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
