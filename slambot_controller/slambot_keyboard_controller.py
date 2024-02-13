#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial                                   # Import the serial library

class KeyboardSubscriber(Node):

    def __init__(self):
        super().__init__('slambot_keyboard_controller')
        self.subscription = self.create_subscription(Twist, '/diffbot_base_controller/cmd_vel_unstamped', self.twist_callback, 3)

        # Serial communication configuration
        self.port = '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-port0'  # Replace with your actual serial port
        self.baudrate = 115200  # Adjust to match your device's baud rate

        try:
            self.ser = serial.Serial(self.port, self.baudrate)
            print("Serial port opened successfully.")
        except serial.SerialException as e:
            print("Error opening serial port:", e)
            exit()

    def twist_callback(self, msg):
        raw_linear_x = int((msg.linear.x)*500)
        raw_angular_z = int((msg.angular.z)*250)
        
        if raw_linear_x>250:              #limiting linear_x values in range [-500, 500]
            raw_linear_x=250
        elif raw_linear_x<-250:
            raw_linear_x=-250

        if raw_angular_z>250:             #limiting angular_z values in range [-500, 500]
            raw_angular_z=250
        elif raw_angular_z<-250:
            raw_angular_z=-250

        if ((raw_linear_x>0) & (raw_linear_x < 10)):
            linear_x = f"   {raw_linear_x}"
        elif ((raw_linear_x>=10) & (raw_linear_x < 100)):
            linear_x = f"  {raw_linear_x}"
        elif (raw_linear_x>=100):
            linear_x = f" {raw_linear_x}"
        elif ((raw_linear_x<0) & (raw_linear_x > -10)):
            linear_x = f"  {raw_linear_x}"
        elif ((raw_linear_x<=-10) & (raw_linear_x > -100)):
            linear_x = f" {raw_linear_x}"
        elif ((raw_linear_x<=-100)):
            linear_x = f"{raw_linear_x}"
        else:
            linear_x = f"   0"

        if ((raw_angular_z>0) & (raw_angular_z < 10)):
            angular_z = f"   {raw_angular_z}"
        elif ((raw_angular_z>=10) & (raw_angular_z < 100)):
            angular_z = f"  {raw_angular_z}"
        elif (raw_angular_z>=100):
            angular_z = f" {raw_angular_z}"
        elif ((raw_angular_z<0) & (raw_angular_z > -10)):
            angular_z = f"  {raw_angular_z}"
        elif ((raw_angular_z<=-10) & (raw_angular_z > -100)):
            angular_z = f" {raw_angular_z}"
        elif ((raw_angular_z<=-100)):
            angular_z = f"{raw_angular_z}"
        else:
            angular_z = f"   0"

        # print(f"Received: linear_x : {linear_x} angular_z: {angular_z}" )
 
        # Format data as a string and send over serial
        data_to_send = f"{linear_x} | {angular_z}\n".encode()  # Add newline for clarity
        self.ser.write(data_to_send)
        print(f"Data sent over serial:{data_to_send}")


def main(args=None):
    rclpy.init(args=args)
    keyboard_subscriber = KeyboardSubscriber()
    rclpy.spin(keyboard_subscriber)
    keyboard_subscriber.destroy_node()
    keyboard_subscriber.ser.close()  # Close the serial port
    rclpy.shutdown()

if __name__ == '__main__':
    main()
