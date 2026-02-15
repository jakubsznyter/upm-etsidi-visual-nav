import math

class Kinematics:
    def __init__(self, wheel_radius, wheel_base, axle_distance):
        """
        :param wheel_radius: Wheel radius in meters (e.g., 0.06)
        :param wheel_base: Front-to-rear axle distance (does not affect steering in this model)
        :param axle_distance: Left-to-right track width. THIS IS KEY FOR STEERING.
        """
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.axle_distance = axle_distance

    def inverse_kinematics(self, v, omega):
        """
        Inverse Kinematics: converts linear velocity (m/s) and angular velocity (rad/s) 
        into individual wheel velocities (rad/s).
        
        Standard ROS: Omega > 0 indicates a LEFT turn.
        """
        
        # Calculate tangential (linear) velocity for each side
        v_right = v + (omega * self.axle_distance / 2.0)
        v_left  = v - (omega * self.axle_distance / 2.0)

        # Convert to wheel rotational velocity (rad/s) = v / r
        w_right = v_right / self.wheel_radius
        w_left  = v_left  / self.wheel_radius

        # Assign to specific wheels (based on your previous code layout)
        # 1: Front Right, 2: Front Left, 3: Rear Right, 4: Rear Left
        
        omega1 = w_right  # Front Right
        omega2 = w_left   # Front Left
        omega3 = w_right  # Rear Right
        omega4 = w_left   # Rear Left

        return omega1, omega2, omega3, omega4

    def direct_kinematics(self, omega1, omega2, omega3, omega4):
        """
        Forward (Direct) Kinematics: converts wheel velocities (rad/s) read from encoders
        into the robot's overall linear velocity (m/s) and angular velocity (rad/s).
        
        omega1: Front Right
        omega2: Front Left
        omega3: Rear Right
        omega4: Rear Left
        """
        
        # Calculate average rotational velocity for the right and left sides
        w_right_avg = (omega1 + omega3) / 2.0
        w_left_avg  = (omega2 + omega4) / 2.0

        # Convert to linear velocity (m/s) for each side
        v_right = w_right_avg * self.wheel_radius
        v_left  = w_left_avg  * self.wheel_radius

        # 1. Robot linear velocity (average of both sides)
        v_real = (v_right + v_left) / 2.0

        # 2. Robot angular velocity (difference between sides divided by track width)
        omega_real = (v_right - v_left) / self.axle_distance

        return v_real, omega_real
