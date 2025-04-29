#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic")
        DRIVE_TOPIC = self.get_parameter("drive_topic").value # set in launch file; different for simulator vs racecar

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(ConeLocation, "/relative_cone",
            self.relative_cone_callback, 1)

        self.parking_distance = .75 # meters; try playing with this number!
        self.look_ahead = 0.5
        self.relative_x = 0
        self.relative_y = 0
        self.wheelbase = 0.34

        self.waypoints = None
        self.moving_backward = False
        self.backward_count = 0

        self.get_logger().info("Parking Controller Initialized")


    def find_next_waypoint(self, cone_x, cone_y):
        look_ahead_distance = self.look_ahead
        cone_pos = np.array([cone_x, cone_y])
        distance = np.linalg.norm(cone_pos)
        if distance <= look_ahead_distance:
            return cone_pos, distance
        else:
            return ((cone_pos / distance) * look_ahead_distance, distance)


    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #################################
        waypoint, cone_dist = self.find_next_waypoint(self.relative_x, self.relative_y)
        way_x, way_y = waypoint
        L = self.wheelbase
        look_ahead = self.look_ahead
        error_distance = cone_dist - self.parking_distance
        #print(self.moving_backward)
        alpha = np.arctan2(way_y, way_x)

        if self.moving_backward is False:
            if error_distance > 0.1:
                velo = 0.5
                steer_angle = np.arctan2(2*np.sin(alpha)*L, look_ahead)
            else:
                if np.abs(np.arctan2(self.relative_y, self.relative_x)) > .175: #10 degrees
                    self.moving_backward = True
                velo = 0.0
                steer_angle = 0.0
        else:
            velo = -0.5
            self.backward_count += 1
            if self.backward_count <= 5:
                steer_angle = -1*np.sign(way_x)* (0.35) #desired angle in radians
                self.get_logger().info(f"steering angle backward: {steer_angle}")
            elif self.backward_count == 10:
                self.get_logger().info(f"count: {self.backward_count}")
                self.moving_backward = False
                self.backward_count = 0
            steer_angle = 0.0





        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.header.frame_id = "base_link"

        drive_cmd.drive.steering_angle = steer_angle
        drive_cmd.drive.steering_angle_velocity = 0.0
        drive_cmd.drive.speed = velo
        drive_cmd.drive.acceleration = 0.
        drive_cmd.drive.jerk = 0.



        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd


        self.drive_pub.publish(drive_cmd)
        self.error_publisher(error_distance)

    def error_publisher(self, dist):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """


        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)
        error_msg = ParkingError()
        x, y = self.relative_x, self.relative_y
        error_msg.x_error = x
        error_msg.y_error = y
        error_msg.distance_error = np.sqrt(x**2 + y**2)

        #################################

        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
