#!/usr/bin/env python3
import numpy as np
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Int32
from ackermann_msgs.msg import AckermannDriveStamped

class TestSafetyController(Node):

    def __init__(self):
        super().__init__("test_safety")
        # Declare parameters to make them available for use
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("velocity", 0.5)

        # Fetch constants from the ROS parameter server
        # This is necessary for the tests to be able to test varying parameters!
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        
        self.timer = self.create_timer(0.05, self.on_timer)
        self.safety_sub = self.create_subscription(
            Int32,
            "/constant_drive",
            self.listener_callback,
            10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, 
            self.DRIVE_TOPIC,
            10)
        self.STOPPED = 0

    def listener_callback(self, msg):
        self.STOPPED = msg.data

    def on_timer(self):
        if not self.STOPPED:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = self.get_clock().now().to_msg()
            drive_msg.header.frame_id = "base_link"

            drive_msg.drive.steering_angle = 0.0
            drive_msg.drive.steering_angle_velocity = 0.0 # change as quick as possible
            drive_msg.drive.speed = self.VELOCITY # m/s
            drive_msg.drive.acceleration = 0.0 # change as quick as possible
            drive_msg.drive.jerk = 0.0 # change as quick as possible        

            self.drive_pub.publish(drive_msg)

def main():
    rclpy.init()
    test_safety_controller = TestSafetyController()
    rclpy.spin(test_safety_controller)
    test_safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
