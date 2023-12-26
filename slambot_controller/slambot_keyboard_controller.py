#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial                                   # Import the serial library

class KeyboardSubscriber(Node):

    def __init__(self):
        super().__init__('slambot_keyboard_controller')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 3)

        # Serial communication configuration
        self.port = '/dev/ttyUSB0'  # Replace with your actual serial port
        self.baudrate = 115200  # Adjust to match your device's baud rate

        try:
            self.ser = serial.Serial(self.port, self.baudrate)
            print("Serial port opened successfully.")
        except serial.SerialException as e:
            print("Error opening serial port:", e)
            exit()

    def twist_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        print("Received keyboard input:")
        print(f"- Linear X: {linear_x:.2f}")
        print(f"- Angular Z: {angular_z:.2f}")

        # Format data as a string and send over serial
        data_to_send = f"{linear_x:.2f}  {angular_z:.2f}\n".encode()  # Add newline for clarity
        self.ser.write(data_to_send)
        print(f"Data sent over serial:{linear_x:.2f}  {angular_z:.2f}\n")

def main(args=None):
    rclpy.init(args=args)
    keyboard_subscriber = KeyboardSubscriber()
    rclpy.spin(keyboard_subscriber)
    keyboard_subscriber.destroy_node()
    keyboard_subscriber.ser.close()  # Close the serial port
    rclpy.shutdown()

if __name__ == '__main__':
    main()
