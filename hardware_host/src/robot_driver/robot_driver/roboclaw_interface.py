"""Robot RoboClaw Module:
This module handles communication with the RoboClaw motor controllers.
File: robot_roboclaw/robot_roboclaw/roboclaw_interface.py"""

from .roboclaw_3 import Roboclaw

class RoboClawInterface:
    
    def __init__(self, front_port, rear_port, baudrate):
        # RoboClaw Configuration
        self.address = 0x80 
        self.rc_front = Roboclaw(front_port, baudrate)  # Controller for front motors
        self.rc_rear = Roboclaw(rear_port, baudrate)    # Controller for rear motors
        
        if not self.rc_front.Open() or not self.rc_rear.Open():
            raise Exception("Error: Could not open connection with RoboClaw controllers.")
        

        self.ERROR_MESSAGES = {
            0x0000: "Normal",
            0x0001: "Warning: Motor 1 Overcurrent",
            0x0002: "Warning: Motor 2 Overcurrent",
            0x0004: "Error: Emergency Stop",
            0x0008: "Error: Temperature Fault",
            0x0010: "Error: Temperature 2 Fault",
            0x0020: "Error: Main Battery Low",
            0x0040: "Error: Logic Battery Low",
            0x0080: "Error: Logic Fault",
            0x0100: "Error: Motor 1 Driver Fault",
            0x0200: "Error: Motor 2 Driver Fault",
            0x0400: "Warning: Motor 1 Speed Limit",
            0x0800: "Warning: Motor 2 Speed Limit" 
        }
            
    def set_wheel_speed(self, front_left, front_right, rear_left, rear_right):
        # Send speeds to the motors
        self.rc_front.SpeedM1(self.address, front_left)
        self.rc_front.SpeedM2(self.address, front_right)
        self.rc_rear.SpeedM1(self.address, rear_left)
        self.rc_rear.SpeedM2(self.address, rear_right)

    def read_encoders_speed(self):
        # Read encoders and calculate real speeds in pulses/second
        status_f1, enc_f1, dir_f1 = self.rc_front.ReadSpeedM1(self.address)
        status_f2, enc_f2, dir_f2 = self.rc_front.ReadSpeedM2(self.address)
        status_r1, enc_r1, dir_r1 = self.rc_rear.ReadSpeedM1(self.address)
        status_r2, enc_r2, dir_r2 = self.rc_rear.ReadSpeedM2(self.address)
        return enc_f1, enc_f2, enc_r1, enc_r2
    
    def read_encoders_position(self):
        """Read absolute encoder position (pulses)."""
        status_f1, enc_f1, dir_f1 = self.rc_front.ReadEncM1(self.address)
        status_f2, enc_f2, dir_f2 = self.rc_front.ReadEncM2(self.address)
        status_r1, enc_r1, dir_r1 = self.rc_rear.ReadEncM1(self.address)
        status_r2, enc_r2, dir_r2 = self.rc_rear.ReadEncM2(self.address)
        return enc_f1, enc_f2, enc_r1, enc_r2

    def reset_encoders(self):
        """Reset the encoders."""
        self.rc_front.ResetEncoders(self.address)
        self.rc_rear.ResetEncoders(self.address)
        print("Encoders reset to 0.")
    
    def ReadVoltage(self):
        # Read main battery voltage
        voltage_max = 16.8 # Maximum battery voltage
        voltage_min = 13.2 # Minimum battery voltage
        voltage = self.rc_front.ReadMainBatteryVoltage(self.address)
        
        if voltage[0]:  # If reading was successful
            voltage_value = voltage[1] / 10.0 # Convert to Volts
            battery_percentage = ((voltage_value - voltage_min) / (voltage_max - voltage_min)) * 100
            return voltage_value, battery_percentage  
        else:
            raise Exception("Error reading main battery voltage")
            return None
    
    def check_status(self):
        """
        Reads the current and error status of both controllers
        and prints it to the console in a readable format.
        """
        # --- Front Controller ---
        status_c_f, current_f1, current_f2 = self.rc_front.ReadCurrents(self.address)
        status_e_f, error_f = self.rc_front.ReadError(self.address)
        
        print("\n--- RoboClaw Diagnostic ---")
        print("Front Controller:")
        if status_c_f:
            print(f"  Wheel Current (Left/Right): {current_f1 / 100.0:.2f} A / {current_f2 / 100.0:.2f} A")
        else:
            print("  Could not read current.")
        if status_e_f:
            error_msg = self.ERROR_MESSAGES.get(error_f, f"Unknown code: {hex(error_f)}")
            print(f"  Error Status: {error_msg}")
        else:
            print("  Could not read error status.")

        # --- Rear Controller ---
        status_c_r, current_r1, current_r2 = self.rc_rear.ReadCurrents(self.address)
        status_e_r, error_r = self.rc_rear.ReadError(self.address)
        print(f"Raw error: ({hex(error_r)})")
        
        print("Rear Controller:")
        if status_c_r:
            print(f"  Wheel Current (Left/Right): {current_r1 / 100.0:.2f} A / {current_r2 / 100.0:.2f} A")
        else:
            print("  Could not read current.")
        if status_e_r:
            error_msg = self.ERROR_MESSAGES.get(error_r, f"Unknown code: {hex(error_r)}")
            print(f"  Error Status: {error_msg}")
        else:
            print("  Could not read error status.")
        print("----------------------------")
